//! The driver: one real-time thread answering telegrams, and the handle callers use.

use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use arrayvec::{ArrayString, ArrayVec};
use crossbeam_queue::ArrayQueue;

use crate::config::{DriverConfig, Limits};
use crate::control::{Command, Session, MAX_CHUNK};
use crate::link::UdpLink;
use crate::state::{RobotState, SessionState, StateMap};
use crate::stats::{Event, Stats};
use crate::telegram::{ParseError, Parser, RobotTelegram, Schema, SensorTelegram, Writer};
use crate::{Error, Result};

pub const COMMAND_QUEUE: usize = 256;
pub const EVENT_QUEUE: usize = 1024;
/// Every-cycle log ring: 32 s at 250 Hz.
pub const LOG_CAPACITY: usize = 8192;

struct Shared {
    commands: ArrayQueue<Command>,
    events: ArrayQueue<Event>,
    stop: AtomicBool,
    session: AtomicU8,
    queue_len: AtomicU32,
    /// Commands pushed by the API and not yet applied and published by the real-time thread.
    /// Raised before the push, lowered after the `publish` that reflects the command, so
    /// `wait_idle` never sees an empty command queue together with a `queue_len` (or state)
    /// from before the command was applied.
    pending: AtomicU32,
    start: Instant,
}

fn session_from_u8(v: u8) -> SessionState {
    match v {
        1 => SessionState::Active,
        2 => SessionState::Lost,
        _ => SessionState::Listening,
    }
}

fn session_to_u8(s: SessionState) -> u8 {
    match s {
        SessionState::Listening => 0,
        SessionState::Active => 1,
        SessionState::Lost => 2,
    }
}

/// Try to make the current thread real-time. Returns whether it worked.
pub(crate) fn raise_priority() -> bool {
    #[cfg(target_os = "linux")]
    {
        use thread_priority::{
            set_thread_priority_and_policy, thread_native_id, RealtimeThreadSchedulePolicy, ThreadPriority, ThreadPriorityValue, ThreadSchedulePolicy,
        };
        let Ok(p) = ThreadPriorityValue::try_from(80u8) else { return false };
        set_thread_priority_and_policy(
            thread_native_id(),
            ThreadPriority::Crossplatform(p),
            ThreadSchedulePolicy::Realtime(RealtimeThreadSchedulePolicy::Fifo),
        )
        .is_ok()
    }
    #[cfg(target_os = "macos")]
    {
        thread_priority::set_current_thread_priority(thread_priority::ThreadPriority::Max).is_ok()
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos")))]
    {
        false
    }
}

/// State owned by the real-time thread.
struct Rt {
    link: UdpLink,
    parser: Parser,
    writer: Writer,
    onlysend: bool,
    cycle: Duration,
    link_timeout: Duration,
    map: StateMap,
    session: Session,
    tele: RobotTelegram,
    /// The last telegram that parsed. A failed parse may leave `tele` partly updated; it is
    /// restored from here so nothing of a bad telegram reaches the state later.
    tele_good: RobotTelegram,
    sen: SensorTelegram,
    state: RobotState,
    stats: Stats,
    rx: Vec<u8>,
    tx: Vec<u8>,
    shared: Arc<Shared>,
    state_in: triple_buffer::Input<RobotState>,
    stats_in: triple_buffer::Input<Stats>,
    log_tx: rtrb::Producer<RobotState>,
    last_rx: Option<Instant>,
    last_ipoc: Option<u64>,
    ipoc_step: Option<u64>,
    latency_sum_us: u64,
    /// Commands popped from the queue and not yet released from `Shared::pending`.
    unsettled: u32,
}

impl Rt {
    fn event(&self, e: Event) {
        let _ = self.shared.events.push(e);
    }

    /// Takes effect for callers at the next `publish`, after the state that carries it.
    fn set_session(&mut self, s: SessionState) {
        self.state.session = s;
    }

    fn drain_session_events(&mut self) {
        for e in self.session.take_events() {
            self.event(e);
        }
    }

    fn publish(&mut self) {
        let c = self.session.clamp_counts();
        self.stats.position_clamps = c.position;
        self.stats.velocity_clamps = c.velocity;
        self.stats.acceleration_clamps = c.acceleration;
        self.stats.correction_clamps = c.correction;
        self.stats.starved_cycles = self.session.starved();
        self.stats.queue_len = self.session.queue_len() as u32;
        self.state_in.write(self.state);
        self.stats_in.write(self.stats);
        // The atomics after the buffers: a caller that acts on them reads a state at least as new,
        // e.g. `wait_for_robot` sees `Active` only after the session's first reply was sent and
        // its state published, and `wait_idle` sees an empty queue with the final command.
        self.shared.queue_len.store(self.stats.queue_len, Ordering::Release);
        self.shared.session.store(session_to_u8(self.state.session), Ordering::Release);
    }

    fn pop_command(&mut self) -> Option<Command> {
        let cmd = self.shared.commands.pop()?;
        self.unsettled += 1;
        Some(cmd)
    }

    /// Release the popped commands from `pending`. Called after the `publish` that reflects them.
    fn settle(&mut self) {
        if self.unsettled > 0 {
            self.shared.pending.fetch_sub(self.unsettled, Ordering::Release);
            self.unsettled = 0;
        }
    }

    fn lose_link(&mut self) {
        self.session.lost();
        self.link.reset_peer();
        // The silence is not an inter-arrival time within a session.
        self.last_rx = None;
        self.set_session(SessionState::Lost);
        self.drain_session_events();
        self.publish();
    }

    /// Commands that arrive while no session is active: settings apply, motion is rejected.
    fn drain_commands_inactive(&mut self) {
        while let Some(cmd) = self.pop_command() {
            match cmd {
                Command::SetLimits(_) | Command::SetSpeedScale(_) | Command::Hold => {
                    if self.session.apply(cmd).is_err() {
                        self.event(Event::CommandRejected);
                    }
                }
                _ => self.event(Event::CommandRejected),
            }
        }
    }

    fn cycle(&mut self) -> Result<()> {
        let Some(rx) = self.link.recv_newest(&mut self.rx)? else {
            if self.session.state() == SessionState::Active && self.last_rx.is_some_and(|t| t.elapsed() > self.link_timeout) {
                self.lose_link();
            }
            // Inside a session, commands wait for the next telegram: a silence shorter than the
            // link timeout must not reject them.
            if self.session.state() != SessionState::Active {
                self.drain_commands_inactive();
            }
            return Ok(());
        };
        self.stats.skipped_datagrams += rx.skipped as u64;
        if let Some(prev) = self.last_rx {
            let ia = rx.arrived.duration_since(prev).as_micros() as u64;
            self.stats.interarrival_max_us = self.stats.interarrival_max_us.max(ia);
        }
        self.last_rx = Some(rx.arrived);

        match self.parser.parse(&self.rx[..rx.len], &mut self.tele) {
            Ok(()) => self.tele_good = self.tele,
            Err(e) => {
                self.tele = self.tele_good;
                // Outside a session, do not stay locked to a sender that does not speak this
                // protocol: the next datagram from anyone may start the session.
                if self.session.state() != SessionState::Active {
                    self.link.reset_peer();
                }
                if matches!(e, ParseError::Sentype { .. }) {
                    self.stats.sentype_mismatches += 1;
                    self.event(Event::SentypeMismatch);
                } else {
                    self.stats.parse_errors += 1;
                    self.event(Event::ParseError);
                }
                self.publish();
                return Ok(());
            }
        }

        if self.session.state() != SessionState::Active {
            // Whatever is still queued was sent for an earlier session (the API rejects motion
            // while no session is active): settings apply, motion is rejected.
            self.drain_commands_inactive();
            self.session.start(&self.tele);
            self.stats.sessions += 1;
            self.last_ipoc = None;
            self.ipoc_step = None;
            self.set_session(SessionState::Active);
        }
        if let Some(prev) = self.last_ipoc {
            let d = self.tele.ipoc.wrapping_sub(prev);
            match self.ipoc_step {
                None => self.ipoc_step = Some(d),
                Some(s) if d != s => self.stats.ipoc_gaps += 1,
                _ => {}
            }
        }
        self.last_ipoc = Some(self.tele.ipoc);

        self.state.update_from(&self.tele, &self.map);
        self.state.t = rx.arrived.duration_since(self.shared.start).as_secs_f64();
        self.stats.robot_delay = self.tele.delay;

        while let Some(cmd) = self.pop_command() {
            if self.session.apply(cmd).is_err() {
                self.event(Event::CommandRejected);
            }
        }
        let q_cmd = self.session.step(&self.tele, &self.state, &mut self.sen);
        self.state.cycle = self.session.cycles();

        if !self.onlysend {
            self.writer.write(&self.sen, &mut self.tx);
            self.link.send(&self.tx)?;
            let lat = rx.arrived.elapsed();
            let us = lat.as_micros() as u64;
            self.stats.replies += 1;
            self.latency_sum_us += us;
            self.stats.reply_latency_mean_us = self.latency_sum_us as f64 / self.stats.replies as f64;
            self.stats.reply_latency_max_us = self.stats.reply_latency_max_us.max(us);
            if lat > self.cycle {
                self.stats.late_replies += 1;
                self.event(Event::LateReply { latency_us: us });
            }
        }
        self.state.q_command = q_cmd;
        self.state.ak = self.sen.ak;
        self.stats.cycles += 1;
        if self.log_tx.push(self.state).is_err() {
            self.stats.log_drops += 1;
            if self.stats.log_drops == 1 {
                self.event(Event::LogDropped);
            }
        }
        self.drain_session_events();
        self.publish();
        Ok(())
    }
}

fn run_rt(mut rt: Rt) {
    while !rt.shared.stop.load(Ordering::Relaxed) {
        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| rt.cycle())) {
            Ok(Ok(())) => {}
            Ok(Err(_)) => {
                rt.stats.io_errors += 1;
                rt.event(Event::IoError);
                rt.publish();
            }
            Err(_) => {
                rt.event(Event::Panic);
                rt.lose_link();
            }
        }
        // After every path's last `publish`, including a cycle cut short by an error or a panic.
        rt.settle();
    }
}

/// Handle to a running driver. Cheap to share behind an `Arc`.
pub struct Driver {
    cfg: DriverConfig,
    limits: Mutex<Limits>,
    shared: Arc<Shared>,
    state_out: Mutex<triple_buffer::Output<RobotState>>,
    stats_out: Mutex<triple_buffer::Output<Stats>>,
    log_rx: Mutex<rtrb::Consumer<RobotState>>,
    rt: Mutex<Option<JoinHandle<()>>>,
    local_addr: SocketAddr,
}

const _: () = {
    const fn assert_send_sync<T: Send + Sync>() {}
    assert_send_sync::<Driver>();
};

fn cycles_for(d: Duration, cycle: Duration, what: &str) -> Result<u32> {
    if d.is_zero() {
        return Err(Error::Limit(format!("{what} must be positive")));
    }
    let n = (d.as_secs_f64() / cycle.as_secs_f64()).round().max(1.0);
    Ok(n as u32)
}

fn short(msg: &str) -> ArrayString<56> {
    let mut s = ArrayString::new();
    let mut end = msg.len().min(56);
    while !msg.is_char_boundary(end) {
        end -= 1;
    }
    s.push_str(&msg[..end]);
    s
}

impl Driver {
    /// Bind the socket, start the real-time thread, return immediately (`Listening`).
    pub fn spawn(cfg: DriverConfig) -> Result<Driver> {
        cfg.limits.validate()?;
        if cfg.cycle.is_zero() {
            return Err(Error::Config("cycle must be positive".into()));
        }
        if cfg.link_timeout.is_zero() {
            return Err(Error::Config("link_timeout must be positive".into()));
        }
        let schema = Schema::compile(&cfg.ethernet)?;
        let stop_slot = match &cfg.stop_channel {
            Some(tag) => Some(
                schema
                    .sen_slot(tag, None)
                    .ok_or_else(|| Error::Config(format!("stop channel {tag:?} is not a text element of the RECEIVE section")))?,
            ),
            None => None,
        };
        let addr = cfg.bind_addr()?;
        let tick = cfg.link_timeout.min(Duration::from_millis(50)).max(Duration::from_millis(1));
        let link = UdpLink::bind(addr, tick)?;
        let local_addr = link.local_addr()?;
        let shared = Arc::new(Shared {
            commands: ArrayQueue::new(COMMAND_QUEUE),
            events: ArrayQueue::new(EVENT_QUEUE),
            stop: AtomicBool::new(false),
            session: AtomicU8::new(0),
            queue_len: AtomicU32::new(0),
            pending: AtomicU32::new(0),
            start: Instant::now(),
        });
        let (state_in, state_out) = triple_buffer::triple_buffer(&RobotState::default());
        let (stats_in, stats_out) = triple_buffer::triple_buffer(&Stats::default());
        let (log_tx, log_rx) = rtrb::RingBuffer::<RobotState>::new(LOG_CAPACITY);
        let rt = Rt {
            parser: Parser::new(&schema),
            writer: Writer::new(&schema),
            onlysend: schema.onlysend,
            cycle: cfg.cycle,
            link_timeout: cfg.link_timeout,
            map: StateMap::from_schema(&schema),
            session: Session::new(cfg.limits, cfg.cycle, stop_slot),
            link,
            tele: RobotTelegram::default(),
            tele_good: RobotTelegram::default(),
            sen: SensorTelegram::default(),
            state: RobotState::default(),
            stats: Stats::default(),
            rx: vec![0u8; 65536],
            tx: Vec::with_capacity(8192),
            shared: shared.clone(),
            state_in,
            stats_in,
            log_tx,
            last_rx: None,
            last_ipoc: None,
            ipoc_step: None,
            latency_sum_us: 0,
            unsettled: 0,
        };
        let want_rt = cfg.rt_priority;
        let shared_rt = shared.clone();
        let handle = std::thread::Builder::new().name("kuka-rsi-rt".into()).spawn(move || {
            let ok = if want_rt { raise_priority() } else { false };
            let _ = shared_rt.events.push(Event::RtPriority { ok });
            run_rt(rt);
        })?;
        Ok(Driver {
            limits: Mutex::new(cfg.limits),
            cfg,
            shared,
            state_out: Mutex::new(state_out),
            stats_out: Mutex::new(stats_out),
            log_rx: Mutex::new(log_rx),
            rt: Mutex::new(Some(handle)),
            local_addr,
        })
    }

    pub fn config(&self) -> &DriverConfig {
        &self.cfg
    }

    pub fn local_addr(&self) -> SocketAddr {
        self.local_addr
    }

    pub fn session(&self) -> SessionState {
        session_from_u8(self.shared.session.load(Ordering::Acquire))
    }

    /// Latest published state (wait-free on the real-time side).
    pub fn state(&self) -> RobotState {
        *self.state_out.lock().unwrap().read()
    }

    pub fn stats(&self) -> Stats {
        *self.stats_out.lock().unwrap().read()
    }

    /// Events since the last call.
    pub fn events(&self) -> Vec<Event> {
        let mut v = Vec::new();
        while let Some(e) = self.shared.events.pop() {
            v.push(e);
        }
        v
    }

    /// Move every-cycle states logged since the last call into `out`; returns the count.
    pub fn drain_log(&self, out: &mut Vec<RobotState>) -> usize {
        let mut rx = self.log_rx.lock().unwrap();
        let mut n = 0;
        while let Ok(s) = rx.pop() {
            out.push(s);
            n += 1;
        }
        n
    }

    /// Block until the first telegram of a session has been answered.
    pub fn wait_for_robot(&self, timeout: Duration) -> Result<()> {
        let t0 = Instant::now();
        loop {
            if self.shared.stop.load(Ordering::Relaxed) {
                return Err(Error::Closed);
            }
            if self.session() == SessionState::Active {
                return Ok(());
            }
            if t0.elapsed() >= timeout {
                return Err(Error::RobotTimeout);
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    /// Block until every queued command has been applied and the target queue is empty.
    pub fn wait_idle(&self, timeout: Duration) -> Result<()> {
        let t0 = Instant::now();
        loop {
            if self.shared.stop.load(Ordering::Relaxed) {
                return Err(Error::Closed);
            }
            match self.session() {
                SessionState::Active => {}
                SessionState::Lost => return Err(Error::LinkLost(self.cfg.link_timeout)),
                SessionState::Listening => return Err(Error::Protocol("robot not connected".into())),
            }
            // `pending` first: once it reads zero, `queue_len` includes every applied command.
            if self.shared.pending.load(Ordering::Acquire) == 0 && self.shared.queue_len.load(Ordering::Acquire) == 0 {
                return Ok(());
            }
            if t0.elapsed() >= timeout {
                return Err(Error::IdleTimeout);
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    fn require_active(&self) -> Result<()> {
        if self.shared.stop.load(Ordering::Relaxed) {
            return Err(Error::Closed);
        }
        match self.session() {
            SessionState::Active => Ok(()),
            SessionState::Lost => Err(Error::LinkLost(self.cfg.link_timeout)),
            SessionState::Listening => Err(Error::Protocol("robot not connected".into())),
        }
    }

    fn send(&self, cmd: Command) -> Result<()> {
        if self.shared.stop.load(Ordering::Relaxed) {
            return Err(Error::Closed);
        }
        self.shared.pending.fetch_add(1, Ordering::AcqRel);
        if self.shared.commands.push(cmd).is_err() {
            self.shared.pending.fetch_sub(1, Ordering::AcqRel);
            return Err(Error::QueueFull);
        }
        Ok(())
    }

    /// Queue absolute joint targets (rad), `dt` apart. `replace` drops what was queued.
    pub fn stream(&self, points: &[[f64; 6]], dt: Duration, replace: bool) -> Result<()> {
        self.require_active()?;
        let cycles = cycles_for(dt, self.cfg.cycle, "dt")?;
        {
            let limits = self.limits.lock().unwrap();
            for p in points {
                limits.check_position(p)?;
            }
        }
        let mut first = true;
        for chunk in points.chunks(MAX_CHUNK) {
            let mut arr: ArrayVec<[f64; 6], MAX_CHUNK> = ArrayVec::new();
            arr.extend(chunk.iter().copied());
            self.send(Command::Stream { points: arr, cycles_per_point: cycles, replace: replace && first })?;
            first = false;
        }
        Ok(())
    }

    /// Smooth (quintic) move from the current command to `q` (rad) over `duration`.
    pub fn move_to(&self, q: [f64; 6], duration: Duration) -> Result<()> {
        self.require_active()?;
        let cycles = cycles_for(duration, self.cfg.cycle, "duration")?;
        self.limits.lock().unwrap().check_position(&q)?;
        self.send(Command::MoveTo { q, cycles })
    }

    /// Drop the queue and hold the current command.
    pub fn hold(&self) -> Result<()> {
        self.send(Command::Hold)
    }

    /// Stop the robot through `EStr` and freeze the command.
    pub fn fault(&self, msg: &str) -> Result<()> {
        self.require_active()?;
        self.send(Command::Fault(short(msg)))
    }

    /// Show a notification on the smartHMI.
    pub fn notify(&self, msg: &str) -> Result<()> {
        self.require_active()?;
        self.send(Command::Notify(short(msg)))
    }

    /// Pulse the stop channel so the KRL program leaves `RSI_MOVECORR()`.
    pub fn stop_program(&self) -> Result<()> {
        if self.cfg.stop_channel.is_none() {
            return Err(Error::Config("no stop channel configured (DriverConfig::stop_channel)".into()));
        }
        self.require_active()?;
        self.send(Command::StopProgram)
    }

    pub fn set_speed_scale(&self, s: f64) -> Result<()> {
        if !(0.0..=1.0).contains(&s) {
            return Err(Error::Limit(format!("speed scale {s} outside 0..=1")));
        }
        self.send(Command::SetSpeedScale(s))
    }

    pub fn set_limits(&self, l: Limits) -> Result<()> {
        l.validate()?;
        // Hold the lock across the push: a concurrent `stream` validates against the limits the
        // real-time thread will apply, and a failed push leaves both sides unchanged.
        let mut cur = self.limits.lock().unwrap();
        self.send(Command::SetLimits(l))?;
        *cur = l;
        Ok(())
    }

    /// End the session cleanly if possible, then stop the thread. Idempotent.
    pub fn close(&self) -> Result<()> {
        if self.shared.stop.load(Ordering::Relaxed) {
            return Ok(());
        }
        if self.cfg.stop_channel.is_some() && self.session() == SessionState::Active {
            let _ = self.send(Command::StopProgram);
            let t0 = Instant::now();
            while self.session() == SessionState::Active && t0.elapsed() < Duration::from_secs(1) {
                std::thread::sleep(Duration::from_millis(5));
            }
        }
        self.shared.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.rt.lock().unwrap().take() {
            h.join().map_err(|_| Error::Protocol("real-time thread panicked".into()))?;
        }
        Ok(())
    }
}

impl Drop for Driver {
    fn drop(&mut self) {
        let _ = self.close();
    }
}
