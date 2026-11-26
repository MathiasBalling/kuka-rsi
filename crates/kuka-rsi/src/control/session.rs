//! One RSI session: from the first telegram to the loss of the link.

use std::time::Duration;

use arrayvec::{ArrayString, ArrayVec};

use super::{Blend, ClampCounts, Command, Interpolator, Limiter, MAX_QUEUE, Segment};
use crate::config::Limits;
use crate::state::{RobotState, SessionEnd, SessionState, deg2rad6, rad2deg6};
use crate::stats::{ClampKind, Event};
use crate::telegram::{RobotTelegram, SensorTelegram};
use crate::{Error, Result};

/// Cycles a message or stop pulse is repeated for.
pub const MESSAGE_CYCLES: u32 = 25;
/// How long after a stop request the end of the session counts as requested. A KR 10 sent its
/// last telegram 116 ms after the pulse: the time its program takes to reach `RSI_OFF`, which
/// the KRL interpreter does not bound. A robot that still sends after this did not stop on the
/// pulse.
pub const STOP_GRACE: Duration = Duration::from_secs(1);
/// How long playback waits after the robot program resumes. `ProgStatus` reports the program
/// running before the robot can move: after the enabling switch was released, the drives of a
/// KR 10 came back on 72 to 120 ms later, and RSI stops with "command velocity" if the command
/// has moved away from the robot by then.
pub const RESUME_DELAY: Duration = Duration::from_millis(500);
/// How long playback takes to return to full speed after `RESUME_DELAY`, so the robot leaves
/// its standstill with the acceleration of a move that starts, not of one under way.
pub const RESUME_RAMP: Duration = Duration::from_millis(500);
/// How far back a stop looks for the point of the path where the robot came to stand. The
/// command runs ahead of the robot by its following lag and by the cycles the stop is reported
/// late: 18 cycles together on a KR 10.
pub const REWIND_MAX: Duration = Duration::from_secs(1);
/// Cycles the search for that point goes on past the closest one found.
const REWIND_PATIENCE: u32 = 25;

pub struct Session {
    state: SessionState,
    limits: Limits,
    q_start: [f64; 6],
    interp: Interpolator,
    limiter: Limiter,
    speed_scale: f64,
    estr: ArrayString<64>,
    estr_cycles: u32,
    stop_slot: Option<usize>,
    stop_cycles: u32,
    /// `STOP_GRACE` in cycles.
    stop_grace_cycles: u32,
    /// Cycles left of `stop_grace_cycles` since the last stop request.
    stop_grace: u32,
    frozen: Option<[f64; 6]>,
    cycles: u64,
    /// Cycles of playback since the driver started: every cycle adds its playback rate.
    played: f64,
    /// The last cycle's playback rate was 0.
    paused: bool,
    /// `RESUME_DELAY` in cycles.
    resume_delay: u32,
    /// Cycles left of `resume_delay` since the program last reported running again.
    resume_wait: u32,
    /// `RESUME_RAMP` and `REWIND_MAX` in cycles.
    resume_ramp: u32,
    rewind_max: u32,
    /// Cycles of the ramp played since the robot last stood; `resume_ramp` once at full speed.
    ramp: u32,
    /// The robot does not follow the command: its program is stopped, or resumed less than
    /// `RESUME_DELAY` ago. The command is the measured position and nothing is played.
    standing: bool,
    /// What `is_settled` reports while `standing`: the command had reached the last target when
    /// the robot stopped, or the motion was replaced since.
    standing_settled: bool,
    last_command: [f64; 6],
    tracking_alarm: bool,
    starved_alarm: bool,
    /// One rising-edge latch per `ClampKind` (position, velocity, acceleration, correction),
    /// so a continuous clamp burst emits a single `Event::Clamped` instead of one per cycle.
    clamp_active: [bool; 4],
    prev_counts: ClampCounts,
    events: ArrayVec<Event, 16>,
    /// Points still to come from a `stream` call one of whose chunks was rejected; they are
    /// rejected too, so a call is played whole or not at all. Cleared by the first chunk of the
    /// next call, in case the rest of the rejected call never arrives.
    reject_remaining: u32,
}

impl Session {
    pub fn new(limits: Limits, cycle: Duration, stop_slot: Option<usize>) -> Session {
        if let Some(s) = stop_slot {
            assert!(s < crate::telegram::MAX_SLOTS, "stop slot {s} outside 0..{}", crate::telegram::MAX_SLOTS);
        }
        Session {
            state: SessionState::Listening,
            limits,
            q_start: [0.0; 6],
            interp: Interpolator::new([0.0; 6]),
            limiter: Limiter::new(limits, cycle),
            speed_scale: 1.0,
            estr: ArrayString::new(),
            estr_cycles: 0,
            stop_slot,
            stop_cycles: 0,
            stop_grace: 0,
            frozen: None,
            cycles: 0,
            played: 0.0,
            paused: false,
            stop_grace_cycles: (STOP_GRACE.as_secs_f64() / cycle.as_secs_f64()).ceil() as u32,
            resume_delay: (RESUME_DELAY.as_secs_f64() / cycle.as_secs_f64()).ceil() as u32,
            resume_wait: 0,
            resume_ramp: (RESUME_RAMP.as_secs_f64() / cycle.as_secs_f64()).ceil() as u32,
            rewind_max: (REWIND_MAX.as_secs_f64() / cycle.as_secs_f64()).ceil() as u32,
            ramp: 0,
            standing: false,
            standing_settled: false,
            last_command: [0.0; 6],
            tracking_alarm: false,
            starved_alarm: false,
            clamp_active: [false; 4],
            prev_counts: ClampCounts::default(),
            events: ArrayVec::new(),
            reject_remaining: 0,
        }
    }

    pub fn state(&self) -> SessionState {
        self.state
    }

    pub fn q_start(&self) -> [f64; 6] {
        self.q_start
    }

    pub fn cycles(&self) -> u64 {
        self.cycles
    }

    pub fn queue_len(&self) -> usize {
        self.interp.len()
    }

    pub fn is_idle(&self) -> bool {
        self.interp.is_idle()
    }

    /// A `Fault` froze the command. It stays frozen, and motion and messages are rejected, until
    /// the next session (`start`).
    pub fn is_faulted(&self) -> bool {
        self.frozen.is_some()
    }

    /// The target queue is empty and the command has reached the last target (the limiter is not
    /// still catching up), or nothing has been commanded yet. A move that had finished stays
    /// settled while the robot stands with its program stopped, although the command follows
    /// the robot then; it returns to the target after the resume.
    pub fn is_settled(&self) -> bool {
        if !self.interp.is_idle() {
            return false;
        }
        if self.standing {
            return self.standing_settled;
        }
        match self.limiter.last() {
            None => true,
            Some(last) => {
                let target = self.interp.current();
                (0..6).all(|i| (last[i] - target[i]).abs() <= 1e-9)
            }
        }
    }

    /// Playback time in cycles since the driver started. A cycle adds its playback rate: 1 at
    /// full speed, the pendant override times the speed scale otherwise, 0 while the program is
    /// stopped. A move of `n` cycles is played once this has advanced by `n`.
    pub fn played(&self) -> f64 {
        self.played
    }

    /// Playback stands still: in the last cycle the program was stopped, or the override or the
    /// speed scale was 0. False from the start of a session until its first cycle.
    pub fn is_paused(&self) -> bool {
        self.paused
    }

    /// Why the session would have ended if the telegrams stopped now.
    pub fn end_reason(&self) -> SessionEnd {
        if self.frozen.is_some() {
            SessionEnd::Fault
        } else if self.stop_grace > 0 {
            SessionEnd::StopRequested
        } else {
            SessionEnd::LinkTimeout
        }
    }

    pub fn starved(&self) -> u64 {
        self.interp.starved()
    }

    pub fn clamp_counts(&self) -> ClampCounts {
        self.limiter.counts()
    }

    /// Events emitted since the last call.
    pub fn take_events(&mut self) -> ArrayVec<Event, 16> {
        std::mem::take(&mut self.events)
    }

    fn event(&mut self, e: Event) {
        let _ = self.events.try_push(e);
    }

    /// First telegram of a session: capture the start pose (programmed setpoint) and hold the
    /// robot at its actual position.
    pub fn start(&mut self, tele: &RobotTelegram) {
        self.q_start = deg2rad6(&tele.aspos);
        let q_actual = deg2rad6(&tele.aipos);
        self.interp.reset(q_actual);
        self.limiter.reset(q_actual);
        self.frozen = None;
        self.estr.clear();
        self.estr_cycles = 0;
        self.stop_cycles = 0;
        self.stop_grace = 0;
        self.cycles = 0;
        self.paused = false;
        self.resume_wait = 0;
        self.ramp = self.resume_ramp;
        self.standing = false;
        self.last_command = q_actual;
        self.tracking_alarm = false;
        self.starved_alarm = false;
        self.clamp_active = [false; 4];
        self.reject_remaining = 0;
        self.state = SessionState::Active;
        self.event(Event::RobotConnected { ipoc: tele.ipoc });
    }

    /// Drop the queue: playback starts again from the last command, which is where the robot
    /// stands if its program is stopped.
    fn drop_queue(&mut self) -> [f64; 6] {
        let last = self.limiter.last().unwrap_or(self.q_start);
        self.interp.reset(last);
        self.standing_settled = true;
        last
    }

    /// The robot came to stand at `q` with a move under way: take the playback back to the
    /// point of the path closest to `q`, so the move goes on from where the robot is. The stop
    /// is reported some cycles late and the robot follows the command with a lag, so the
    /// command has run ahead; without this the limiter would close that gap after the resume
    /// at the velocity limit. A queue that had emptied is left alone: that move is played, and
    /// the limiter closes what is left of it.
    fn rewind_to(&mut self, q: &[f64; 6]) {
        if self.interp.is_idle() || self.frozen.is_some() {
            return;
        }
        let distance = |p: [f64; 6]| (0..6).map(|i| (p[i] - q[i]) * (p[i] - q[i])).sum::<f64>();
        let (mut best, mut best_d) = (0, distance(self.interp.current()));
        for n in 1..=self.rewind_max {
            let d = distance(self.interp.position_behind(f64::from(n)));
            if d < best_d {
                (best, best_d) = (n, d);
            } else if n - best > REWIND_PATIENCE {
                break;
            }
        }
        self.played = (self.played - self.interp.rewind(f64::from(best))).max(0.0);
    }

    /// The session is over: drop the queue and wait for a new session.
    pub fn lost(&mut self, reason: SessionEnd) {
        self.state = SessionState::Lost;
        self.drop_queue();
        self.frozen = None;
        self.event(Event::SessionEnded(reason));
    }

    pub fn apply(&mut self, cmd: Command) -> Result<()> {
        match cmd {
            Command::Stream { points, cycles_per_point, replace, batch_remaining, first } => {
                // A new call starts clean: the rest of an earlier rejected call may never arrive
                // (the driver stops pushing when the command queue fills or it closes).
                if first {
                    self.reject_remaining = 0;
                }
                // Once a chunk of a `stream` call is rejected, the rest of the call is rejected
                // too: playing the later chunks would draw a straight line across the gap.
                if self.reject_remaining > 0 {
                    self.reject_remaining = self.reject_remaining.saturating_sub(points.len() as u32);
                    return Err(Error::QueueFull);
                }
                let r = self.push_stream(&points, cycles_per_point, replace, batch_remaining);
                if r.is_err() {
                    self.reject_remaining = batch_remaining;
                }
                r
            }
            Command::MoveTo { q, cycles } => {
                if self.frozen.is_some() {
                    return Err(Error::Faulted);
                }
                self.limits.check_position(&q)?;
                self.drop_queue();
                if !self.interp.push(Segment { q_end: q, cycles, blend: Blend::Quintic }) {
                    return Err(Error::QueueFull);
                }
                Ok(())
            }
            Command::Hold => {
                self.drop_queue();
                Ok(())
            }
            Command::Fault(msg) => {
                self.estr.clear();
                self.estr.push_str("Error: ");
                let room = self.estr.capacity() - self.estr.len();
                self.estr.push_str(&msg[..msg.len().min(room)]);
                self.estr_cycles = MESSAGE_CYCLES;
                self.frozen = Some(self.drop_queue());
                self.event(Event::Fault);
                Ok(())
            }
            Command::Notify(msg) => {
                if self.frozen.is_some() {
                    return Err(Error::Faulted);
                }
                self.estr.clear();
                self.estr.push_str(&msg);
                self.estr_cycles = MESSAGE_CYCLES;
                self.event(Event::Notified);
                Ok(())
            }
            Command::StopProgram => {
                if self.stop_slot.is_none() {
                    return Err(Error::Config("no stop channel configured (DriverConfig::stop_channel)".into()));
                }
                self.stop_cycles = MESSAGE_CYCLES;
                self.stop_grace = self.stop_grace_cycles;
                self.event(Event::StopPulsed);
                Ok(())
            }
            Command::SetSpeedScale(s) => {
                if !(0.0..=1.0).contains(&s) {
                    return Err(Error::Limit(format!("speed scale {s} outside 0..=1")));
                }
                self.speed_scale = s;
                Ok(())
            }
            Command::SetLimits(l) => {
                l.validate()?;
                self.limits = l;
                self.limiter.set_limits(l);
                Ok(())
            }
        }
    }

    /// One chunk of a `stream` call, accepted whole or rejected whole.
    fn push_stream(
        &mut self,
        points: &[[f64; 6]],
        cycles_per_point: f64,
        replace: bool,
        batch_remaining: u32,
    ) -> Result<()> {
        if self.frozen.is_some() {
            return Err(Error::Faulted);
        }
        for p in points {
            self.limits.check_position(p)?;
        }
        // Room for this chunk and the rest of its call, so the call is never cut short midway.
        let used = if replace { 0 } else { self.interp.len() };
        if used + points.len() + batch_remaining as usize > MAX_QUEUE {
            return Err(Error::QueueFull);
        }
        if replace {
            self.drop_queue();
        }
        for &p in points {
            if !self.interp.push(Segment { q_end: p, cycles: cycles_per_point, blend: Blend::Linear }) {
                return Err(Error::QueueFull);
            }
        }
        Ok(())
    }

    /// Produce this cycle's reply. Returns the joint command (rad).
    pub fn step(&mut self, tele: &RobotTelegram, state: &RobotState, out: &mut SensorTelegram) -> [f64; 6] {
        self.cycles += 1;
        // A robot whose program is stopped ignores the correction and takes all of it in one
        // step when the program resumes, so the command stays where the robot stands: there is
        // nothing to jump across then.
        if !state.program_running {
            self.resume_wait = self.resume_delay;
        }
        let standing = self.resume_wait > 0;
        if state.program_running {
            self.resume_wait = self.resume_wait.saturating_sub(1);
        }
        if standing && !self.standing {
            self.standing_settled = self.is_settled();
            self.rewind_to(&state.q);
        }
        self.standing = standing;
        let rate = if standing {
            self.ramp = 0;
            0.0
        } else {
            self.ramp = (self.ramp + 1).min(self.resume_ramp);
            self.speed_scale * state.speed_override * f64::from(self.ramp) / f64::from(self.resume_ramp.max(1))
        };
        self.played += rate;
        self.paused = rate <= 0.0;
        self.stop_grace = self.stop_grace.saturating_sub(1);
        let starved_before = self.interp.starved();
        let target = self.interp.step(rate);
        if self.interp.starved() > starved_before {
            if !self.starved_alarm {
                self.starved_alarm = true;
                self.event(Event::Starved);
            }
        } else if !self.interp.is_idle() {
            self.starved_alarm = false;
        }

        let q_cmd = match self.frozen {
            Some(f) => f,
            None if standing => {
                self.limiter.reset(state.q);
                state.q
            }
            None => self.limiter.step(target, self.q_start),
        };
        let c = self.limiter.counts();
        let p = self.prev_counts;
        let hits = [
            c.position > p.position,
            c.velocity > p.velocity,
            c.acceleration > p.acceleration,
            c.correction > p.correction,
        ];
        let kinds = [ClampKind::Position, ClampKind::Velocity, ClampKind::Acceleration, ClampKind::Correction];
        for i in 0..4 {
            if hits[i] {
                if !self.clamp_active[i] {
                    self.clamp_active[i] = true;
                    self.event(Event::Clamped(kinds[i]));
                }
            } else {
                self.clamp_active[i] = false;
            }
        }
        self.prev_counts = c;

        out.ipoc = tele.ipoc;
        let mut corr = [0.0; 6];
        for i in 0..6 {
            corr[i] = q_cmd[i] - self.q_start[i];
        }
        out.ak = rad2deg6(&corr);

        if self.estr_cycles > 0 {
            self.estr_cycles -= 1;
            out.estr.clear();
            out.estr.push_str(&self.estr);
        } else {
            out.estr.clear();
        }
        if let Some(slot) = self.stop_slot {
            out.slots[slot] = if self.stop_cycles > 0 {
                self.stop_cycles -= 1;
                1.0
            } else {
                0.0
            };
        }

        let mut err = 0.0f64;
        for i in 0..6 {
            err = err.max((state.q[i] - self.last_command[i]).abs());
        }
        if err > self.limits.max_tracking_error {
            if !self.tracking_alarm {
                self.tracking_alarm = true;
                self.event(Event::TrackingError { error_rad: err });
            }
        } else if err < 0.5 * self.limits.max_tracking_error {
            self.tracking_alarm = false;
        }
        self.last_command = q_cmd;
        q_cmd
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Limits;
    use crate::control::{MAX_CHUNK, MAX_QUEUE};
    use crate::state::{RobotState, SessionEnd, deg2rad6};
    use crate::telegram::{RobotTelegram, SensorTelegram};
    use arrayvec::{ArrayString, ArrayVec};
    use std::time::Duration;

    const START_DEG: [f64; 6] = [10.0, -90.0, 90.0, 0.0, 90.0, 0.0];
    /// `STOP_GRACE`, `RESUME_DELAY` and `RESUME_RAMP` at the 4 ms cycle of these tests.
    const STOP_GRACE_CYCLES: u32 = 250;
    const RESUME_DELAY_CYCLES: u32 = 125;
    const RESUME_RAMP_CYCLES: u32 = 125;

    fn tele(aspos_deg: [f64; 6], aipos_deg: [f64; 6], ipoc: u64) -> RobotTelegram {
        RobotTelegram { ipoc, aspos: aspos_deg, aipos: aipos_deg, ..Default::default() }
    }

    fn state(t: &RobotTelegram) -> RobotState {
        RobotState {
            q: deg2rad6(&t.aipos),
            q_setpoint: deg2rad6(&t.aspos),
            speed_override: 1.0,
            program_running: true,
            ..Default::default()
        }
    }

    fn session() -> Session {
        let limits = Limits::kr10_defaults().with_max_velocity_all(1.0).with_max_acceleration_all(None);
        Session::new(limits, Duration::from_millis(4), Some(6))
    }

    fn started() -> (Session, RobotTelegram) {
        let mut s = session();
        let t = tele(START_DEG, START_DEG, 1000);
        s.start(&t);
        (s, t)
    }

    fn stream(q: [f64; 6], cycles: f64, replace: bool) -> Command {
        let mut points = ArrayVec::new();
        points.push(q);
        Command::Stream { points, cycles_per_point: cycles, replace, batch_remaining: 0, first: true }
    }

    fn run(s: &mut Session, t: &RobotTelegram, n: usize) -> SensorTelegram {
        let mut out = SensorTelegram::default();
        let st = state(t);
        for _ in 0..n {
            s.step(t, &st, &mut out);
        }
        out
    }

    #[test]
    fn start_captures_setpoint_and_first_step_is_zero_correction() {
        let (mut s, t) = started();
        assert_eq!(s.state(), SessionState::Active);
        assert_eq!(s.q_start(), deg2rad6(&START_DEG));
        assert!(s.take_events().contains(&Event::RobotConnected { ipoc: 1000 }));
        let mut out = SensorTelegram::default();
        let q = s.step(&t, &state(&t), &mut out);
        assert_eq!(q, deg2rad6(&START_DEG));
        assert!(out.ak.iter().all(|a| a.abs() < 1e-9), "{:?}", out.ak);
        assert_eq!(out.ipoc, 1000);
        assert_eq!(s.cycles(), 1);
        assert!(out.estr.is_empty());
        assert_eq!(out.slots[6], 0.0);
    }

    #[test]
    fn stream_moves_linearly_to_the_target() {
        let (mut s, t) = started();
        let mut q = s.q_start();
        q[0] += 0.02;
        s.apply(stream(q, 10.0, false)).unwrap();
        assert_eq!(s.queue_len(), 1);
        let out = run(&mut s, &t, 5);
        assert!((out.ak[0] - 0.01f64.to_degrees()).abs() < 1e-9, "{:?}", out.ak);
        let out = run(&mut s, &t, 5);
        assert!((out.ak[0] - 0.02f64.to_degrees()).abs() < 1e-9);
        assert!(s.is_idle());
        assert_eq!(s.clamp_counts(), ClampCounts::default());
    }

    #[test]
    fn override_and_program_stop_scale_playback() {
        let (mut s, t) = started();
        let mut q = s.q_start();
        q[0] += 0.02;
        s.apply(stream(q, 10.0, false)).unwrap();
        let mut st = state(&t);
        st.speed_override = 0.5;
        let mut out = SensorTelegram::default();
        for _ in 0..10 {
            st.q = s.step(&t, &st, &mut out);
        }
        assert!((out.ak[0] - 0.01f64.to_degrees()).abs() < 1e-9, "half speed: {:?}", out.ak);
        st.program_running = false;
        for _ in 0..10 {
            st.q = s.step(&t, &st, &mut out);
        }
        assert!((out.ak[0] - 0.01f64.to_degrees()).abs() < 1e-9, "paused: {:?}", out.ak);
        assert_eq!(s.queue_len(), 1, "the rest of the move is kept");
        assert_eq!(s.starved(), 0);
    }

    #[test]
    fn fault_freezes_and_sends_error_message() {
        let (mut s, t) = started();
        let mut q = s.q_start();
        q[0] += 0.02;
        s.apply(stream(q, 10.0, false)).unwrap();
        run(&mut s, &t, 3);
        s.apply(Command::Fault(ArrayString::from("bad").unwrap())).unwrap();
        let frozen = run(&mut s, &t, 1);
        assert_eq!(frozen.estr.as_str(), "Error: bad");
        assert!((frozen.ak[0] - 0.006f64.to_degrees()).abs() < 1e-9, "frozen at cycle 3: {:?}", frozen.ak);
        let later = run(&mut s, &t, 24);
        assert_eq!(later.estr.as_str(), "Error: bad", "message repeats for 25 cycles");
        let after = run(&mut s, &t, 1);
        assert!(after.estr.is_empty());
        assert_eq!(after.ak, frozen.ak, "still frozen");
        assert!(matches!(s.apply(stream(q, 10.0, false)), Err(Error::Faulted)), "targets rejected after a fault");
        assert!(s.take_events().contains(&Event::Fault));
    }

    #[test]
    fn notify_sends_plain_message() {
        let (mut s, t) = started();
        s.apply(Command::Notify(ArrayString::from("hello").unwrap())).unwrap();
        let out = run(&mut s, &t, 1);
        assert_eq!(out.estr.as_str(), "hello");
        assert!(s.take_events().contains(&Event::Notified));
    }

    #[test]
    fn stop_program_pulses_the_stop_slot() {
        let (mut s, t) = started();
        s.apply(Command::StopProgram).unwrap();
        let out = run(&mut s, &t, 1);
        assert_eq!(out.slots[6], 1.0);
        let out = run(&mut s, &t, 24);
        assert_eq!(out.slots[6], 1.0);
        let out = run(&mut s, &t, 1);
        assert_eq!(out.slots[6], 0.0);
        assert!(s.take_events().contains(&Event::StopPulsed));
        let mut none = Session::new(Limits::kr10_defaults(), Duration::from_millis(4), None);
        none.start(&t);
        assert!(none.apply(Command::StopProgram).is_err());
    }

    #[test]
    fn apply_rejects_bad_commands() {
        let (mut s, _) = started();
        let mut q = s.q_start();
        q[0] = 4.0;
        assert!(matches!(s.apply(stream(q, 10.0, false)), Err(Error::Limit(_))));
        q[0] = f64::NAN;
        assert!(matches!(s.apply(stream(q, 10.0, false)), Err(Error::Limit(_))));
        assert!(matches!(s.apply(Command::SetSpeedScale(1.5)), Err(Error::Limit(_))));
        assert!(matches!(s.apply(Command::SetSpeedScale(-0.1)), Err(Error::Limit(_))));
        let mut bad = Limits::kr10_defaults();
        bad.max_velocity[0] = 0.0;
        assert!(matches!(s.apply(Command::SetLimits(bad)), Err(Error::Config(_))));
        assert_eq!(s.queue_len(), 0, "rejected commands leave the queue alone");
    }

    #[test]
    fn queue_full_is_reported() {
        let (mut s, _) = started();
        let q = s.q_start();
        let mut points = ArrayVec::new();
        for _ in 0..MAX_CHUNK {
            points.push(q);
        }
        for _ in 0..(MAX_QUEUE / MAX_CHUNK) {
            s.apply(Command::Stream {
                points: points.clone(),
                cycles_per_point: 1.0,
                replace: false,
                batch_remaining: 0,
                first: true,
            })
            .unwrap();
        }
        assert!(matches!(s.apply(stream(q, 1.0, false)), Err(Error::QueueFull)));
    }

    #[test]
    fn replace_drops_pending_points() {
        let (mut s, t) = started();
        let mut a = s.q_start();
        a[0] += 0.02;
        let mut b = s.q_start();
        b[0] += 0.04;
        s.apply(stream(a, 10.0, false)).unwrap();
        s.apply(stream(b, 10.0, false)).unwrap();
        run(&mut s, &t, 5);
        let mut c = s.q_start();
        c[0] += 0.01;
        s.apply(stream(c, 5.0, true)).unwrap();
        assert_eq!(s.queue_len(), 1);
        let out = run(&mut s, &t, 5);
        assert!((out.ak[0] - 0.01f64.to_degrees()).abs() < 1e-9, "{:?}", out.ak);
        assert!(s.is_idle());
    }

    #[test]
    fn hold_and_move_to() {
        let (mut s, t) = started();
        let mut q = s.q_start();
        q[0] += 0.02;
        s.apply(stream(q, 10.0, false)).unwrap();
        run(&mut s, &t, 2);
        s.apply(Command::Hold).unwrap();
        assert!(s.is_idle());
        let held = run(&mut s, &t, 3);
        assert!((held.ak[0] - 0.004f64.to_degrees()).abs() < 1e-9, "{:?}", held.ak);
        let mut m = s.q_start();
        m[0] += 0.008;
        s.apply(Command::MoveTo { q: m, cycles: 4.0 }).unwrap();
        let out = run(&mut s, &t, 4);
        assert!((out.ak[0] - 0.008f64.to_degrees()).abs() < 1e-9, "{:?}", out.ak);
    }

    #[test]
    fn tracking_error_event_fires_once_per_excursion() {
        let (mut s, t) = started();
        run(&mut s, &t, 1);
        s.take_events();
        let mut far = t;
        far.aipos[0] += 5.0;
        let mut out = SensorTelegram::default();
        s.step(&far, &state(&far), &mut out);
        let ev = s.take_events();
        assert!(ev.iter().any(|e| matches!(e, Event::TrackingError { .. })), "{ev:?}");
        s.step(&far, &state(&far), &mut out);
        assert!(!s.take_events().iter().any(|e| matches!(e, Event::TrackingError { .. })));
        s.step(&t, &state(&t), &mut out);
        s.step(&far, &state(&far), &mut out);
        assert!(s.take_events().iter().any(|e| matches!(e, Event::TrackingError { .. })), "fires again after recovery");
    }

    #[test]
    fn lost_and_restart_recaptures_start() {
        let (mut s, t) = started();
        let mut q = s.q_start();
        q[0] += 0.02;
        s.apply(stream(q, 10.0, false)).unwrap();
        run(&mut s, &t, 3);
        s.lost(SessionEnd::LinkTimeout);
        assert_eq!(s.state(), SessionState::Lost);
        assert!(s.take_events().contains(&Event::SessionEnded(SessionEnd::LinkTimeout)));
        let mut new_start = START_DEG;
        new_start[1] = -80.0;
        let t2 = tele(new_start, new_start, 5000);
        s.start(&t2);
        assert_eq!(s.q_start(), deg2rad6(&new_start));
        assert_eq!(s.cycles(), 0);
        let out = run(&mut s, &t2, 1);
        assert!(out.ak.iter().all(|a| a.abs() < 1e-9));
        assert_eq!(out.ipoc, 5000);
    }

    #[test]
    fn playback_time_counts_the_cycles_played() {
        let (mut s, t) = started();
        let mut st = state(&t);
        let mut out = SensorTelegram::default();
        assert!(!s.is_paused(), "a new session is not paused before its first telegram says so");
        st.speed_override = 0.5;
        for _ in 0..10 {
            s.step(&t, &st, &mut out);
        }
        assert!((s.played() - 5.0).abs() < 1e-12, "10 cycles at half speed: {}", s.played());
        assert!(!s.is_paused());
        st.program_running = false;
        for _ in 0..10 {
            s.step(&t, &st, &mut out);
        }
        assert!((s.played() - 5.0).abs() < 1e-12, "a stopped program plays nothing: {}", s.played());
        assert!(s.is_paused());
        st.program_running = true;
        for _ in 0..RESUME_DELAY_CYCLES {
            s.step(&t, &st, &mut out);
        }
        assert!((s.played() - 5.0).abs() < 1e-12, "nor does the wait after the resume: {}", s.played());
        assert!(s.is_paused());
        st.speed_override = 0.0;
        for _ in 0..RESUME_RAMP_CYCLES {
            s.step(&t, &st, &mut out);
        }
        assert!(s.is_paused(), "override 0 is a pause too");
        st.speed_override = 1.0;
        s.step(&t, &st, &mut out);
        assert!((s.played() - 6.0).abs() < 1e-12);
        assert!(!s.is_paused());
        s.apply(Command::SetSpeedScale(0.0)).unwrap();
        s.step(&t, &st, &mut out);
        assert!(s.is_paused(), "speed scale 0 is a pause too");
    }

    /// A session with the default acceleration limit, a robot that follows the command exactly
    /// while its program runs, and a move of A1 under way: 0.2 rad over 200 cycles, 60 played.
    fn moving() -> (Session, RobotTelegram, RobotState, SensorTelegram) {
        let mut s = Session::new(Limits::kr10_defaults(), Duration::from_millis(4), Some(6));
        let t = tele(START_DEG, START_DEG, 1000);
        s.start(&t);
        let mut q = s.q_start();
        q[0] += 0.2;
        s.apply(Command::MoveTo { q, cycles: 200.0 }).unwrap();
        let mut st = state(&t);
        let mut out = SensorTelegram::default();
        for _ in 0..60 {
            st.q = s.step(&t, &st, &mut out);
        }
        (s, t, st, out)
    }

    #[test]
    fn a_stopped_program_is_commanded_to_where_the_robot_stands() {
        let (mut s, t, mut st, mut out) = moving();
        // The robot stops short of the command, and reports the stopped program afterwards.
        let stood = st.q[0] - 0.004;
        st.q[0] = stood;
        st.program_running = false;
        for _ in 0..50 {
            let q = s.step(&t, &st, &mut out);
            assert_eq!(q[0], stood, "the command is the measured position");
        }
        assert!((out.ak[0] - (stood - s.q_start()[0]).to_degrees()).abs() < 1e-9, "{:?}", out.ak);
        // The brakes let the axis sag: the command follows.
        st.q[0] = stood - 0.001;
        let q = s.step(&t, &st, &mut out);
        assert_eq!(q[0], stood - 0.001);
        assert!(s.is_paused());
        assert_eq!(s.clamp_counts(), ClampCounts::default(), "following the robot is not a clamp");
    }

    #[test]
    fn playback_waits_for_the_drives_after_the_program_resumes() {
        let (mut s, t, mut st, mut out) = moving();
        let stood = st.q[0] - 0.004;
        st.q[0] = stood;
        st.program_running = false;
        s.step(&t, &st, &mut out);
        let played = s.played();
        // "Running" again, but the drives are not on yet: the robot stands still.
        st.program_running = true;
        for i in 0..RESUME_DELAY_CYCLES {
            let q = s.step(&t, &st, &mut out);
            assert_eq!(q[0], stood, "cycle {i} after the resume");
            assert!(s.is_paused(), "cycle {i} after the resume");
        }
        assert_eq!(s.played(), played, "nothing is played while waiting");
        // Then the command leaves the measured position without a step and reaches the target.
        let amax = 300f64.to_radians() * 0.004 * 0.004;
        let (mut prev, mut prev_delta) = (stood, 0.0);
        for i in 0..400 {
            let q = s.step(&t, &st, &mut out);
            st.q = q;
            let delta = q[0] - prev;
            assert!((delta - prev_delta).abs() <= amax + 1e-12, "cycle {i}: step of {delta} after {prev_delta}");
            (prev, prev_delta) = (q[0], delta);
            if i == 0 {
                assert!(!s.is_paused());
            }
        }
        assert!((prev - (s.q_start()[0] + 0.2)).abs() < 1e-9, "reached the target: {prev}");
        assert!(s.is_settled());
    }

    #[test]
    fn a_stop_takes_the_move_back_to_where_the_robot_stands() {
        let mut s = Session::new(Limits::kr10_defaults(), Duration::from_millis(4), Some(6));
        let t = tele(START_DEG, START_DEG, 1000);
        s.start(&t);
        let mut q = s.q_start();
        q[0] += 0.2;
        s.apply(Command::MoveTo { q, cycles: 200.0 }).unwrap();
        let mut st = state(&t);
        let mut out = SensorTelegram::default();
        let mut peak = 0f64;
        for _ in 0..60 {
            let cmd = s.step(&t, &st, &mut out);
            peak = peak.max(cmd[0] - st.q[0]);
            st.q = cmd;
        }
        // The robot stands, and 15 cycles go by before its program reports that.
        let stood = st.q[0];
        let mut ahead = stood;
        for _ in 0..15 {
            ahead = s.step(&t, &st, &mut out)[0];
        }
        assert!(ahead - stood > 0.015, "the command ran ahead by {}", ahead - stood);
        st.program_running = false;
        s.step(&t, &st, &mut out);
        assert!((s.played() - 60.0).abs() < 1e-9, "the 15 cycles are to be played again: {}", s.played());
        assert!(!s.is_idle());
        st.program_running = true;
        for _ in 0..RESUME_DELAY_CYCLES {
            s.step(&t, &st, &mut out);
        }
        // The move goes on from the robot: never faster than the move itself, and as slowly
        // out of the standstill as a move that starts.
        let mut cycles = 0;
        let mut first = 0.0;
        while !s.is_settled() {
            let cmd = s.step(&t, &st, &mut out);
            let step = cmd[0] - st.q[0];
            if cycles == 0 {
                first = step;
            }
            peak = peak.max(0.2 * 1.875 / 200.0);
            assert!(step <= peak + 1e-12, "cycle {cycles}: a step of {step}, the move peaks at {peak}");
            st.q = cmd;
            cycles += 1;
            assert!(cycles < 1000, "the move never ended");
        }
        assert!(first < 2e-5, "the first step after the standstill: {first}");
        assert!((st.q[0] - q[0]).abs() < 1e-9);
        // 140 cycles of the move were left, and the ramp back to full speed costs 62 more.
        assert!((200..=206).contains(&cycles), "{cycles} cycles after the resume");
        assert!((s.played() - 200.0).abs() < 1e-6, "a move of 200 cycles: {}", s.played());
    }

    #[test]
    fn playback_returns_to_full_speed_over_the_resume_ramp() {
        let (mut s, t, mut st, mut out) = moving();
        st.program_running = false;
        s.step(&t, &st, &mut out);
        st.program_running = true;
        for _ in 0..RESUME_DELAY_CYCLES {
            s.step(&t, &st, &mut out);
        }
        let played = s.played();
        s.step(&t, &st, &mut out);
        assert!((s.played() - played - 1.0 / 125.0).abs() < 1e-12, "the first cycle of the ramp");
        for _ in 1..RESUME_RAMP_CYCLES {
            s.step(&t, &st, &mut out);
        }
        assert!((s.played() - played - 63.0).abs() < 1e-9, "the sum of 1/125..=125/125: {}", s.played() - played);
        s.step(&t, &st, &mut out);
        assert!((s.played() - played - 64.0).abs() < 1e-9, "full speed from here");
    }

    #[test]
    fn a_stop_during_the_wait_for_the_drives_starts_the_wait_again() {
        let (mut s, t, mut st, mut out) = moving();
        let stood = st.q[0];
        st.program_running = false;
        s.step(&t, &st, &mut out);
        st.program_running = true;
        for _ in 0..RESUME_DELAY_CYCLES - 1 {
            s.step(&t, &st, &mut out);
        }
        st.program_running = false;
        s.step(&t, &st, &mut out);
        st.program_running = true;
        for i in 0..RESUME_DELAY_CYCLES {
            assert_eq!(s.step(&t, &st, &mut out)[0], stood, "cycle {i} after the second resume");
        }
        assert!(s.step(&t, &st, &mut out)[0] > stood);
    }

    #[test]
    fn a_move_that_had_finished_stays_settled_through_a_stop() {
        let (mut s, t, mut st, mut out) = moving();
        for _ in 0..200 {
            st.q = s.step(&t, &st, &mut out);
        }
        assert!(s.is_settled());
        let target = st.q[0];
        // Stopped with a servo error: the command follows the robot, the move is still done.
        st.q[0] = target - 1e-4;
        st.program_running = false;
        for _ in 0..10 {
            s.step(&t, &st, &mut out);
        }
        assert!(s.is_settled(), "the move finished before the stop");
        st.program_running = true;
        for _ in 0..RESUME_DELAY_CYCLES + 50 {
            st.q = s.step(&t, &st, &mut out);
        }
        assert!((st.q[0] - target).abs() < 1e-12, "back on the target after the resume: {}", st.q[0] - target);
        assert!(s.is_settled());
    }

    #[test]
    fn a_move_stopped_under_way_is_not_settled() {
        let (mut s, t, mut st, mut out) = moving();
        st.program_running = false;
        s.step(&t, &st, &mut out);
        assert!(!s.is_settled());
        s.apply(Command::Hold).unwrap();
        s.step(&t, &st, &mut out);
        assert!(s.is_settled(), "held where the robot stands");
        assert!(s.is_idle());
    }

    #[test]
    fn a_fault_stays_frozen_through_a_stop() {
        let (mut s, t, mut st, mut out) = moving();
        s.apply(Command::Fault(ArrayString::from("x").unwrap())).unwrap();
        let frozen = s.step(&t, &st, &mut out);
        st.q[0] -= 0.004;
        st.program_running = false;
        assert_eq!(s.step(&t, &st, &mut out), frozen);
    }

    #[test]
    fn end_reason_is_a_timeout_unless_the_driver_caused_it() {
        let (s, _) = started();
        assert_eq!(s.end_reason(), SessionEnd::LinkTimeout);
    }

    #[test]
    fn end_reason_after_a_stop_pulse_is_stop_requested_for_a_while() {
        let (mut s, t) = started();
        s.apply(Command::StopProgram).unwrap();
        assert_eq!(s.end_reason(), SessionEnd::StopRequested);
        run(&mut s, &t, STOP_GRACE_CYCLES as usize - 1);
        assert_eq!(s.end_reason(), SessionEnd::StopRequested);
        // The robot still sends well after the pulse: it did not stop on it, so a silence that
        // follows later has another cause.
        run(&mut s, &t, 1);
        assert_eq!(s.end_reason(), SessionEnd::LinkTimeout);
    }

    #[test]
    fn end_reason_of_a_faulted_session_is_the_fault() {
        let (mut s, _) = started();
        s.apply(Command::Fault(ArrayString::from("bad").unwrap())).unwrap();
        assert_eq!(s.end_reason(), SessionEnd::Fault);
        s.apply(Command::StopProgram).unwrap();
        assert_eq!(s.end_reason(), SessionEnd::Fault, "the stop that ends a faulted session does not hide the fault");
    }

    #[test]
    fn a_new_session_forgets_the_stop_request() {
        let (mut s, t) = started();
        s.apply(Command::StopProgram).unwrap();
        s.lost(s.end_reason());
        assert!(s.take_events().contains(&Event::SessionEnded(SessionEnd::StopRequested)));
        s.start(&t);
        assert_eq!(s.end_reason(), SessionEnd::LinkTimeout);
    }

    #[test]
    fn speed_scale_zero_pauses_without_starving() {
        let (mut s, t) = started();
        let mut q = s.q_start();
        q[0] += 0.02;
        s.apply(stream(q, 10.0, false)).unwrap();
        s.apply(Command::SetSpeedScale(0.0)).unwrap();
        let out = run(&mut s, &t, 10);
        assert!(out.ak[0].abs() < 1e-9);
        assert_eq!(s.starved(), 0);
        s.apply(Command::SetSpeedScale(1.0)).unwrap();
        run(&mut s, &t, 10);
        assert!(s.is_idle());
        run(&mut s, &t, 1);
        assert_eq!(s.starved(), 1);
        assert!(s.take_events().contains(&Event::Starved));
    }

    #[test]
    fn clamp_event_fires_once_per_burst() {
        let (mut s, t) = started();
        let mut q = s.q_start();
        q[0] += 0.2;
        s.apply(stream(q, 1.0, false)).unwrap();
        let mut out = SensorTelegram::default();
        for _ in 0..10 {
            s.step(&t, &state(&t), &mut out);
        }
        let events = s.take_events();
        let clamps = events.iter().filter(|e| matches!(e, Event::Clamped(ClampKind::Velocity))).count();
        assert_eq!(clamps, 1, "{events:?}");
    }

    #[test]
    fn hold_stops_at_the_last_command() {
        let (mut s, t) = started();
        let mut q = s.q_start();
        q[0] += 0.2;
        s.apply(stream(q, 1.0, false)).unwrap();
        let out = run(&mut s, &t, 10);
        assert!((out.ak[0] - 0.04f64.to_degrees()).abs() < 1e-9, "{:?}", out.ak);
        s.apply(Command::Hold).unwrap();
        let out = run(&mut s, &t, 20);
        assert!((out.ak[0] - 0.04f64.to_degrees()).abs() < 1e-9, "{:?}", out.ak);
    }

    #[test]
    fn partial_chunk_is_rejected_whole() {
        let (mut s, _) = started();
        let q = s.q_start();
        let mut full = ArrayVec::new();
        for _ in 0..MAX_CHUNK {
            full.push(q);
        }
        for _ in 0..15 {
            s.apply(Command::Stream {
                points: full.clone(),
                cycles_per_point: 1.0,
                replace: false,
                batch_remaining: 0,
                first: true,
            })
            .unwrap();
        }
        let mut partial = ArrayVec::new();
        for _ in 0..40 {
            partial.push(q);
        }
        s.apply(Command::Stream {
            points: partial,
            cycles_per_point: 1.0,
            replace: false,
            batch_remaining: 0,
            first: true,
        })
        .unwrap();
        assert_eq!(s.queue_len(), 1000);
        assert!(matches!(
            s.apply(Command::Stream {
                points: full.clone(),
                cycles_per_point: 1.0,
                replace: false,
                batch_remaining: 0,
                first: true
            }),
            Err(Error::QueueFull)
        ));
        assert_eq!(s.queue_len(), 1000);
    }

    /// The first chunk of a `stream` call.
    fn chunk(q: [f64; 6], n: usize, batch_remaining: u32) -> Command {
        let mut points = ArrayVec::new();
        for _ in 0..n {
            points.push(q);
        }
        Command::Stream { points, cycles_per_point: 1.0, replace: false, batch_remaining, first: true }
    }

    /// A later chunk of the same `stream` call.
    fn rest(q: [f64; 6], n: usize, batch_remaining: u32) -> Command {
        match chunk(q, n, batch_remaining) {
            Command::Stream { points, cycles_per_point, replace, batch_remaining, .. } => {
                Command::Stream { points, cycles_per_point, replace, batch_remaining, first: false }
            }
            _ => unreachable!(),
        }
    }

    fn filled_to(n: usize) -> Session {
        let (mut s, _) = started();
        let q = s.q_start();
        for _ in 0..n / MAX_CHUNK {
            s.apply(chunk(q, MAX_CHUNK, 0)).unwrap();
        }
        s.apply(chunk(q, n % MAX_CHUNK, 0)).unwrap();
        assert_eq!(s.queue_len(), n);
        s
    }

    #[test]
    fn partial_batch_rejects_the_rest() {
        let mut s = filled_to(1000);
        let q = s.q_start();
        assert!(matches!(s.apply(chunk(q, 40, 30)), Err(Error::QueueFull)), "40 + 30 more do not fit");
        assert!(matches!(s.apply(rest(q, 30, 0)), Err(Error::QueueFull)), "the rest of the batch is rejected");
        assert_eq!(s.queue_len(), 1000);
        s.apply(chunk(q, 20, 0)).unwrap();
        assert_eq!(s.queue_len(), 1020, "the next batch is accepted");

        // The tail would fit on its own here (990 + 30), and is still rejected with its batch.
        let mut s = filled_to(990);
        assert!(matches!(s.apply(chunk(q, 40, 30)), Err(Error::QueueFull)));
        assert!(matches!(s.apply(rest(q, 30, 0)), Err(Error::QueueFull)));
        assert_eq!(s.queue_len(), 990);
        s.apply(chunk(q, 20, 0)).unwrap();
        assert_eq!(s.queue_len(), 1010);
    }

    #[test]
    fn rejected_batch_does_not_outlive_the_session() {
        let mut s = filled_to(1000);
        let q = s.q_start();
        assert!(matches!(s.apply(chunk(q, 40, 30)), Err(Error::QueueFull)));
        s.lost(SessionEnd::LinkTimeout);
        s.start(&tele(START_DEG, START_DEG, 2000));
        // A later chunk, so only the session reset (not the `first` flag) can clear the rejection.
        s.apply(rest(q, 30, 0)).unwrap();
        assert_eq!(s.queue_len(), 30, "a new session starts without a pending batch rejection");
    }

    #[test]
    fn stale_batch_rejection_does_not_reject_the_next_call() {
        let mut s = filled_to(1000);
        let q = s.q_start();
        // The first chunk of a call is rejected with 30 points to follow; the call then fails in
        // the driver before pushing them (another caller filled the command queue), so the rest
        // never arrives to use up the rejection.
        assert!(matches!(s.apply(chunk(q, 40, 30)), Err(Error::QueueFull)));
        s.apply(chunk(q, 20, 0)).unwrap();
        assert_eq!(s.queue_len(), 1020, "the next call's first chunk starts clean");
    }

    #[test]
    fn is_faulted_until_the_next_session() {
        let (mut s, t) = started();
        assert!(!s.is_faulted());
        s.apply(Command::Fault(ArrayString::from("x").unwrap())).unwrap();
        assert!(s.is_faulted());
        run(&mut s, &t, 30);
        assert!(s.is_faulted(), "the fault outlives its message");
        s.lost(SessionEnd::LinkTimeout);
        s.start(&tele(START_DEG, START_DEG, 2000));
        assert!(!s.is_faulted());
    }

    #[test]
    fn settled_once_the_command_reaches_the_last_target() {
        let (mut s, t) = started();
        assert!(s.is_settled());
        let mut q = s.q_start();
        q[0] += 0.02;
        s.apply(stream(q, 10.0, false)).unwrap();
        assert!(!s.is_settled());
        run(&mut s, &t, 9);
        assert!(!s.is_settled());
        run(&mut s, &t, 1);
        assert!(s.is_settled());

        // A velocity-limited jump: the queue drains in one cycle, the command takes longer.
        let mut far = q;
        far[0] += 0.02;
        s.apply(stream(far, 1.0, false)).unwrap();
        run(&mut s, &t, 1);
        assert_eq!(s.queue_len(), 0);
        assert!(!s.is_settled(), "limiter still catching up");
        run(&mut s, &t, 10);
        assert!(s.is_settled());
    }

    #[test]
    fn notify_after_fault_is_rejected() {
        let (mut s, t) = started();
        s.apply(Command::Fault(ArrayString::from("x").unwrap())).unwrap();
        assert!(matches!(s.apply(Command::Notify(ArrayString::from("y").unwrap())), Err(Error::Faulted)));
        let out = run(&mut s, &t, 1);
        assert_eq!(out.estr.as_str(), "Error: x");
    }

    #[test]
    fn fresh_session_and_hold_do_not_starve() {
        let (mut s, t) = started();
        run(&mut s, &t, 5);
        assert_eq!(s.starved(), 0);
        assert!(!s.take_events().iter().any(|e| matches!(e, Event::Starved)));
        let mut q = s.q_start();
        q[0] += 0.001;
        s.apply(stream(q, 2.0, false)).unwrap();
        run(&mut s, &t, 2);
        s.apply(Command::Hold).unwrap();
        run(&mut s, &t, 5);
        assert_eq!(s.starved(), 0);
        assert!(!s.take_events().iter().any(|e| matches!(e, Event::Starved)));
    }

    #[test]
    #[should_panic(expected = "stop slot")]
    fn stop_slot_out_of_range_panics() {
        Session::new(Limits::kr10_defaults(), Duration::from_millis(4), Some(64));
    }

    #[test]
    fn start_holds_the_actual_position() {
        let mut s = session();
        let mut aipos = START_DEG;
        aipos[0] += 0.5;
        let t = tele(START_DEG, aipos, 1000);
        s.start(&t);
        let mut out = SensorTelegram::default();
        let q = s.step(&t, &state(&t), &mut out);
        assert_eq!(q, deg2rad6(&aipos));
        assert!((out.ak[0] - 0.5).abs() < 1e-9, "{:?}", out.ak);
        for i in 1..6 {
            assert!(out.ak[i].abs() < 1e-9, "{:?}", out.ak);
        }
    }
}
