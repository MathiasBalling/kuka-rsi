//! One RSI session: from the first telegram to the loss of the link.

use std::time::Duration;

use arrayvec::{ArrayString, ArrayVec};

use super::{Blend, ClampCounts, Command, Interpolator, Limiter, MAX_QUEUE, Segment};
use crate::config::Limits;
use crate::state::{RobotState, SessionState, deg2rad6, rad2deg6};
use crate::stats::{ClampKind, Event};
use crate::telegram::{RobotTelegram, SensorTelegram};
use crate::{Error, Result};

/// Cycles a message or stop pulse is repeated for.
pub const MESSAGE_CYCLES: u32 = 25;

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
    frozen: Option<[f64; 6]>,
    cycles: u64,
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
            frozen: None,
            cycles: 0,
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
    /// still catching up), or nothing has been commanded yet.
    pub fn is_settled(&self) -> bool {
        if !self.interp.is_idle() {
            return false;
        }
        match self.limiter.last() {
            None => true,
            Some(last) => {
                let target = self.interp.current();
                (0..6).all(|i| (last[i] - target[i]).abs() <= 1e-9)
            }
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
        self.cycles = 0;
        self.last_command = q_actual;
        self.tracking_alarm = false;
        self.starved_alarm = false;
        self.clamp_active = [false; 4];
        self.reject_remaining = 0;
        self.state = SessionState::Active;
        self.event(Event::RobotConnected { ipoc: tele.ipoc });
    }

    /// The link went silent: drop the queue and wait for a new session.
    pub fn lost(&mut self) {
        self.state = SessionState::Lost;
        self.interp.reset(self.limiter.last().unwrap_or(self.q_start));
        self.frozen = None;
        self.event(Event::LinkLost);
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
                self.interp.reset(self.limiter.last().unwrap_or(self.q_start));
                if !self.interp.push(Segment { q_end: q, cycles, blend: Blend::Quintic }) {
                    return Err(Error::QueueFull);
                }
                Ok(())
            }
            Command::Hold => {
                self.interp.reset(self.limiter.last().unwrap_or(self.q_start));
                Ok(())
            }
            Command::Fault(msg) => {
                self.estr.clear();
                self.estr.push_str("Error: ");
                let room = self.estr.capacity() - self.estr.len();
                self.estr.push_str(&msg[..msg.len().min(room)]);
                self.estr_cycles = MESSAGE_CYCLES;
                let last = self.limiter.last().unwrap_or(self.q_start);
                self.frozen = Some(last);
                self.interp.reset(last);
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
            self.interp.reset(self.limiter.last().unwrap_or(self.q_start));
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
        let rate = if state.program_running { self.speed_scale * state.speed_override } else { 0.0 };
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
    use crate::state::{RobotState, deg2rad6};
    use crate::telegram::{RobotTelegram, SensorTelegram};
    use arrayvec::{ArrayString, ArrayVec};
    use std::time::Duration;

    const START_DEG: [f64; 6] = [10.0, -90.0, 90.0, 0.0, 90.0, 0.0];

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
            s.step(&t, &st, &mut out);
        }
        assert!((out.ak[0] - 0.01f64.to_degrees()).abs() < 1e-9, "half speed: {:?}", out.ak);
        st.program_running = false;
        for _ in 0..10 {
            s.step(&t, &st, &mut out);
        }
        assert!((out.ak[0] - 0.01f64.to_degrees()).abs() < 1e-9, "paused: {:?}", out.ak);
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
        s.lost();
        assert_eq!(s.state(), SessionState::Lost);
        assert!(s.take_events().contains(&Event::LinkLost));
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
        s.lost();
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
        s.lost();
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
