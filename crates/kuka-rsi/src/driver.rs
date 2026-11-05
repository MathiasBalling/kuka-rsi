//! The driver: one real-time thread answering telegrams, and the handle callers use.

use std::collections::{HashMap, VecDeque};
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4};
use std::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use arrayvec::{ArrayString, ArrayVec};
use crossbeam_queue::ArrayQueue;

use crate::config::{DriverConfig, Limits};
use crate::control::{Command, MAX_CHUNK, MAX_QUEUE, Session};
use crate::link::UdpLink;
use crate::state::{RobotState, SessionState, StateMap};
use crate::stats::{Event, Stats};
use crate::telegram::{Keyword, ParseError, Parser, RobotTelegram, Schema, SensorTelegram, Writer};
use crate::{Error, Result};

pub const COMMAND_QUEUE: usize = 256;
pub const EVENT_QUEUE: usize = 1024;
/// Every-cycle log ring: 32 s at 250 Hz.
pub const LOG_CAPACITY: usize = 8192;
/// Events kept for `Driver::events` until read; the oldest are dropped first.
pub const EVENT_HISTORY: usize = 1024;

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
    /// The session's command has reached the last target (`Session::is_settled`).
    settled: AtomicBool,
    /// The session is faulted (`Session::is_faulted`): motion and messages are refused until
    /// the next session.
    faulted: AtomicBool,
    /// Points of `Stream` commands pushed by `Driver::stream` and not yet counted in `queue_len`.
    /// Raised before the push, lowered by the real-time thread after the `publish` that follows
    /// the command's application or rejection, so `stream`'s room check never misses a point.
    in_flight: AtomicUsize,
    /// Set by `close` once the real-time thread has been joined: the monitor thread records the
    /// last events and exits.
    monitor_stop: AtomicBool,
    /// The robot's address during a session, packed by `pack_addr` (0 outside a session).
    robot_addr: AtomicU64,
    start: Instant,
}

/// Pack an IPv4 socket address into a `u64` (bit 48 set, address, port) so the real-time thread
/// can publish it without a lock. RSI runs over IPv4; an IPv6 peer packs to 0 (unknown).
fn pack_addr(addr: Option<SocketAddr>) -> u64 {
    match addr {
        Some(SocketAddr::V4(a)) => 1 << 48 | u64::from(a.ip().to_bits()) << 16 | u64::from(a.port()),
        _ => 0,
    }
}

fn unpack_addr(v: u64) -> Option<SocketAddr> {
    (v >> 48 == 1).then(|| SocketAddrV4::new(Ipv4Addr::from_bits((v >> 16) as u32), v as u16).into())
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
            RealtimeThreadSchedulePolicy, ThreadPriority, ThreadPriorityValue, ThreadSchedulePolicy,
            set_thread_priority_and_policy, thread_native_id,
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
    /// IPOC increment between consecutive telegrams: the cycle in milliseconds.
    ipoc_step: u64,
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
    latency_sum_us: u64,
    /// Commands popped from the queue and not yet released from `Shared::pending`.
    unsettled: u32,
    /// Points of the `Stream` commands among them, not yet released from `Shared::in_flight`.
    unsettled_points: usize,
    /// Set by the first late reply of a burst, cleared by the next on-time reply.
    late_alarm: bool,
    /// Set by the first I/O error of a burst, cleared by an iteration without one.
    io_alarm: bool,
    /// An I/O error happened in the current iteration of `run_rt`.
    io_failed: bool,
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
        // `settled` before `queue_len`, which `wait_idle` reads first. `faulted` here too: `settle`
        // releases the fault command from `pending` only after this store.
        self.shared.settled.store(self.session.is_settled(), Ordering::Release);
        self.shared.faulted.store(self.session.is_faulted(), Ordering::Release);
        self.shared.queue_len.store(self.stats.queue_len, Ordering::Release);
        self.shared.session.store(session_to_u8(self.state.session), Ordering::Release);
    }

    fn pop_command(&mut self) -> Option<Command> {
        let cmd = self.shared.commands.pop()?;
        self.unsettled += 1;
        if let Command::Stream { points, .. } = &cmd {
            self.unsettled_points += points.len();
        }
        Some(cmd)
    }

    /// Release the popped commands from `pending` and their points from `in_flight`, accepted or
    /// rejected. Called after the `publish` that reflects them.
    fn settle(&mut self) {
        if self.unsettled_points > 0 {
            self.shared.in_flight.fetch_sub(self.unsettled_points, Ordering::Release);
            self.unsettled_points = 0;
        }
        if self.unsettled > 0 {
            self.shared.pending.fetch_sub(self.unsettled, Ordering::Release);
            self.unsettled = 0;
        }
    }

    /// Count a socket error; one `IoError` event per burst.
    fn io_error(&mut self) {
        self.stats.io_errors += 1;
        self.io_failed = true;
        if !self.io_alarm {
            self.io_alarm = true;
            self.event(Event::IoError);
        }
    }

    fn lose_link(&mut self) {
        self.session.lost();
        self.link.reset_peer();
        self.shared.robot_addr.store(0, Ordering::Release);
        self.set_session(SessionState::Lost);
        self.drain_session_events();
        self.publish();
    }

    /// No telegram this iteration (receive timeout or error): detect a lost link, and outside a
    /// session drain the command queue. Inside a session, commands wait for the next telegram: a
    /// silence shorter than the link timeout must not reject them.
    fn no_telegram(&mut self) {
        if self.session.state() == SessionState::Active && self.last_rx.is_some_and(|t| t.elapsed() > self.link_timeout)
        {
            self.lose_link();
        }
        if self.session.state() != SessionState::Active {
            self.drain_commands_inactive();
        }
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
            self.no_telegram();
            return Ok(());
        };
        self.stats.skipped_datagrams += rx.skipped as u64;
        // Inter-arrival times are measured within a session only: the first telegram of a
        // session is the new reference, so gaps before it are not recorded.
        if self.session.state() == SessionState::Active {
            if let Some(prev) = self.last_rx {
                let ia = rx.arrived.duration_since(prev).as_micros() as u64;
                self.stats.interarrival_max_us = self.stats.interarrival_max_us.max(ia);
            }
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
            // Absolute corrections: the session takes `q_start` from `ASPos` and holds the robot at
            // `AIPos`. A first telegram without both would start it from stale or zero values and
            // command a jump, so it is treated like a bad telegram: counted, not answered. (The
            // config validation already requires both unless ONLYSEND, which never answers.)
            if !self.onlysend && !(self.parser.seen(Keyword::AIPos) && self.parser.seen(Keyword::ASPos)) {
                self.link.reset_peer();
                self.stats.parse_errors += 1;
                self.event(Event::ParseError);
                self.publish();
                return Ok(());
            }
            // Whatever is still queued was sent for an earlier session (the API rejects motion
            // while no session is active): settings apply, motion is rejected.
            self.drain_commands_inactive();
            self.shared.robot_addr.store(pack_addr(self.link.peer()), Ordering::Release);
            self.session.start(&self.tele);
            self.stats.sessions += 1;
            self.last_ipoc = None;
            self.set_session(SessionState::Active);
        }
        if let Some(prev) = self.last_ipoc {
            if self.tele.ipoc.wrapping_sub(prev) != self.ipoc_step {
                self.stats.ipoc_gaps += 1;
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
            // A failed reply is counted; the cycle's bookkeeping below still runs.
            match self.link.send(&self.tx) {
                Ok(()) => {
                    let lat = rx.arrived.elapsed();
                    let us = lat.as_micros() as u64;
                    self.stats.replies += 1;
                    self.latency_sum_us += us;
                    self.stats.reply_latency_mean_us = self.latency_sum_us as f64 / self.stats.replies as f64;
                    self.stats.reply_latency_max_us = self.stats.reply_latency_max_us.max(us);
                    if lat > self.cycle {
                        self.stats.late_replies += 1;
                        if !self.late_alarm {
                            self.late_alarm = true;
                            self.event(Event::LateReply { latency_us: us });
                        }
                    } else {
                        self.late_alarm = false;
                    }
                }
                Err(_) => self.io_error(),
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
    let mut recv_errors = 0u32;
    while !rt.shared.stop.load(Ordering::Relaxed) {
        rt.io_failed = false;
        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| rt.cycle())) {
            Ok(Ok(())) => recv_errors = 0,
            Ok(Err(_)) => {
                // Receiving failed: still watch the link and the command queue.
                rt.io_error();
                rt.no_telegram();
                rt.publish();
                recv_errors += 1;
            }
            Err(_) => {
                recv_errors = 0;
                rt.event(Event::Panic);
                rt.lose_link();
            }
        }
        // After every path's last `publish`, including a cycle cut short by an error or a panic.
        rt.settle();
        if !rt.io_failed {
            rt.io_alarm = false;
        }
        // A receive error that repeats at once must not spin the loop.
        if recv_errors >= 2 {
            std::thread::sleep(Duration::from_millis(1));
        }
    }
    // Stopped by `close`: nobody answers the robot any more.
    rt.set_session(SessionState::Lost);
    rt.publish();
}

/// Allows one log line per event kind (`Clamped(_)` is one kind) per `min_interval`.
pub(crate) struct RateLimiter {
    min_interval: Duration,
    last: HashMap<std::mem::Discriminant<Event>, Instant>,
}

impl RateLimiter {
    pub fn new(min_interval: Duration) -> RateLimiter {
        RateLimiter { min_interval, last: HashMap::new() }
    }

    pub fn allow(&mut self, e: &Event, now: Instant) -> bool {
        let key = std::mem::discriminant(e);
        match self.last.get(&key) {
            Some(t) if now.duration_since(*t) < self.min_interval => false,
            _ => {
                self.last.insert(key, now);
                true
            }
        }
    }
}

fn level_of(e: &Event) -> log::Level {
    match e {
        Event::LinkLost
        | Event::LateReply { .. }
        | Event::ParseError
        | Event::SentypeMismatch
        | Event::IoError
        | Event::TrackingError { .. }
        | Event::CommandRejected
        | Event::Fault
        | Event::Panic
        | Event::LogDropped
        | Event::Starved
        | Event::RtPriority { ok: false } => log::Level::Warn,
        _ => log::Level::Info,
    }
}

/// Moves events from the real-time thread's queue into the history, logging them rate-limited.
/// The real-time thread never logs. Runs until `close` has joined the real-time thread, so its
/// last events are recorded too.
fn run_monitor(shared: Arc<Shared>, history: Arc<Mutex<VecDeque<Event>>>) {
    let mut rl = RateLimiter::new(Duration::from_secs(1));
    loop {
        // Read before draining: every event pushed before `close` set the flag is drained below.
        let last = shared.monitor_stop.load(Ordering::Acquire);
        while let Some(e) = shared.events.pop() {
            if rl.allow(&e, Instant::now()) {
                log::log!(target: "kuka_rsi", level_of(&e), "{e:?}");
            }
            let mut h = history.lock().unwrap_or_else(PoisonError::into_inner);
            if h.len() == EVENT_HISTORY {
                h.pop_front();
            }
            h.push_back(e);
        }
        if last {
            return;
        }
        // `close` unparks the thread so it does not wait out the poll interval.
        std::thread::park_timeout(Duration::from_millis(50));
    }
}

/// Handle to a running driver. Cheap to share behind an `Arc`.
pub struct Driver {
    cfg: DriverConfig,
    /// The API-side copy of the limits. Held by every call that queues motion or limits
    /// (`stream`, `move_to`, `hold`, `set_limits`) across its push, so none lands between the
    /// chunks of a `stream` call.
    motion: Mutex<Limits>,
    shared: Arc<Shared>,
    state_out: Mutex<triple_buffer::Output<RobotState>>,
    stats_out: Mutex<triple_buffer::Output<Stats>>,
    log_rx: Mutex<rtrb::Consumer<RobotState>>,
    rt: Mutex<Option<JoinHandle<()>>>,
    history: Arc<Mutex<VecDeque<Event>>>,
    monitor: Mutex<Option<JoinHandle<()>>>,
    local_addr: SocketAddr,
}

const _: () = {
    const fn assert_send_sync<T: Send + Sync>() {}
    assert_send_sync::<Driver>();
};

/// `d` in cycles, fractional: the interpolator plays fractions exactly.
fn cycles_for(d: Duration, cycle: Duration, what: &str) -> Result<f64> {
    if d.is_zero() {
        return Err(Error::Limit(format!("{what} must be positive")));
    }
    Ok(d.as_secs_f64() / cycle.as_secs_f64())
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
            Some(tag) => Some(schema.sen_slot(tag, None).ok_or_else(|| {
                Error::Config(format!("stop channel {tag:?} is not a text element of the RECEIVE section"))
            })?),
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
            settled: AtomicBool::new(true),
            faulted: AtomicBool::new(false),
            in_flight: AtomicUsize::new(0),
            monitor_stop: AtomicBool::new(false),
            robot_addr: AtomicU64::new(0),
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
            ipoc_step: cfg.cycle.as_millis().max(1) as u64,
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
            latency_sum_us: 0,
            unsettled: 0,
            unsettled_points: 0,
            late_alarm: false,
            io_alarm: false,
            io_failed: false,
        };
        let want_rt = cfg.rt_priority;
        let shared_rt = shared.clone();
        let handle = std::thread::Builder::new().name("kuka-rsi-rt".into()).spawn(move || {
            let ok = if want_rt { raise_priority() } else { false };
            let _ = shared_rt.events.push(Event::RtPriority { ok });
            run_rt(rt);
        })?;
        let history = Arc::new(Mutex::new(VecDeque::with_capacity(EVENT_HISTORY)));
        let shared_mon = shared.clone();
        let history_mon = history.clone();
        let monitor = match std::thread::Builder::new()
            .name("kuka-rsi-monitor".into())
            .spawn(move || run_monitor(shared_mon, history_mon))
        {
            Ok(m) => m,
            Err(e) => {
                // No `Driver` owns the real-time thread yet: stop it here.
                shared.stop.store(true, Ordering::Relaxed);
                let _ = handle.join();
                return Err(e.into());
            }
        };
        Ok(Driver {
            motion: Mutex::new(cfg.limits),
            cfg,
            shared,
            state_out: Mutex::new(state_out),
            stats_out: Mutex::new(stats_out),
            log_rx: Mutex::new(log_rx),
            rt: Mutex::new(Some(handle)),
            history,
            monitor: Mutex::new(Some(monitor)),
            local_addr,
        })
    }

    /// The configuration the driver was spawned with. Its `limits` stay the constructor value;
    /// `limits()` returns the live ones.
    pub fn config(&self) -> &DriverConfig {
        &self.cfg
    }

    /// The limits in force: the constructor's, or the last accepted `set_limits`. The real-time
    /// thread applies a new value with the next telegram; targets are checked against it at once.
    pub fn limits(&self) -> Limits {
        *self.motion.lock().unwrap_or_else(PoisonError::into_inner)
    }

    pub fn local_addr(&self) -> SocketAddr {
        self.local_addr
    }

    /// The address the robot's telegrams come from during a session (its RSI sensor interface),
    /// `None` outside a session.
    pub fn robot_addr(&self) -> Option<SocketAddr> {
        unpack_addr(self.shared.robot_addr.load(Ordering::Acquire))
    }

    pub fn session(&self) -> SessionState {
        session_from_u8(self.shared.session.load(Ordering::Acquire))
    }

    /// A `fault` has been applied: the command is frozen, and `stream`, `move_to`, `notify` and
    /// `wait_idle` return `Error::Faulted` for the rest of the session. Becomes true with the
    /// first telegram after `fault` (the real-time thread applies it) and false when the session
    /// ends: once the robot stops on the fault the link is lost and those calls return
    /// `Error::LinkLost` until the KRL program restarts (a new session).
    pub fn is_faulted(&self) -> bool {
        self.shared.faulted.load(Ordering::Acquire)
    }

    /// Latest published state (wait-free on the real-time side).
    pub fn state(&self) -> RobotState {
        *self.state_out.lock().unwrap().read()
    }

    pub fn stats(&self) -> Stats {
        *self.stats_out.lock().unwrap().read()
    }

    /// Events recorded by the monitor thread since the last call (oldest first). The monitor
    /// polls every 50 ms and keeps at most `EVENT_HISTORY` events, dropping the oldest.
    pub fn events(&self) -> Vec<Event> {
        self.history.lock().unwrap_or_else(PoisonError::into_inner).drain(..).collect()
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

    /// Block until every queued command has been applied, the target queue is empty and the
    /// command has reached the last target. Returns `Error::Faulted` once the session is faulted:
    /// the frozen command will not reach any target.
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
            // `pending` first: once it reads zero, `queue_len` and `settled` include every applied
            // command; `settled` after `queue_len` (the reverse of the order `publish` stores them).
            let idle = self.shared.pending.load(Ordering::Acquire) == 0
                && self.shared.queue_len.load(Ordering::Acquire) == 0
                && self.shared.settled.load(Ordering::Acquire);
            // After `pending`: a fault applied before `pending` read zero is seen here.
            if self.is_faulted() {
                return Err(Error::Faulted);
            }
            if idle {
                return Ok(());
            }
            if t0.elapsed() >= timeout {
                return Err(Error::IdleTimeout);
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    /// The driver is open and a session is active (faulted or not).
    fn require_session(&self) -> Result<()> {
        if self.shared.stop.load(Ordering::Relaxed) {
            return Err(Error::Closed);
        }
        match self.session() {
            SessionState::Active => Ok(()),
            SessionState::Lost => Err(Error::LinkLost(self.cfg.link_timeout)),
            SessionState::Listening => Err(Error::Protocol("robot not connected".into())),
        }
    }

    /// A session is active and not faulted: motion and messages are accepted.
    fn require_active(&self) -> Result<()> {
        self.require_session()?;
        if self.is_faulted() {
            return Err(Error::Faulted);
        }
        Ok(())
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

    /// Queue absolute joint targets (rad), `dt` apart. `replace` drops what was queued. The call
    /// is accepted whole or rejected before anything is queued. No other `stream`, `hold`,
    /// `move_to` or `set_limits` can land between its chunks; a `fault` or a lost link between them
    /// truncates the call but never cuts across the path. Right after a call that drops the queue
    /// (a `replace` call, `hold`, `move_to`), the room check may count the dropped points for up to
    /// one cycle, so a call immediately following it can be refused with `QueueFull` although it
    /// would fit; retrying after a cycle succeeds.
    pub fn stream(&self, points: &[[f64; 6]], dt: Duration, replace: bool) -> Result<()> {
        self.require_active()?;
        let cycles = cycles_for(dt, self.cfg.cycle, "dt")?;
        if points.len() > MAX_QUEUE {
            return Err(Error::Limit(format!("{} points exceed the {MAX_QUEUE}-point target queue", points.len())));
        }
        // Held from validation through the last push: no other motion or limits command can land
        // between this call's chunks in the command queue.
        let limits = self.motion.lock().unwrap();
        for p in points {
            limits.check_position(p)?;
        }
        // Room for the whole call before anything is pushed: the points in the target queue plus
        // those of earlier calls still on their way to it. `in_flight` is read first: a chunk the
        // real-time thread has released from it is already in the `queue_len` read after. With
        // `replace`, everything ahead of the call is dropped by its first chunk.
        if !replace {
            let in_flight = self.shared.in_flight.load(Ordering::Acquire);
            let queued = self.shared.queue_len.load(Ordering::Acquire) as usize;
            if queued + in_flight + points.len() > MAX_QUEUE {
                return Err(Error::QueueFull);
            }
        }
        let chunks = points.len().div_ceil(MAX_CHUNK);
        if COMMAND_QUEUE.saturating_sub(self.shared.commands.len()) < chunks {
            return Err(Error::QueueFull);
        }
        self.shared.in_flight.fetch_add(points.len(), Ordering::AcqRel);
        let mut sent = 0;
        for (i, chunk) in points.chunks(MAX_CHUNK).enumerate() {
            let mut arr: ArrayVec<[f64; 6], MAX_CHUNK> = ArrayVec::new();
            arr.extend(chunk.iter().copied());
            let batch_remaining = (points.len() - sent - chunk.len()) as u32;
            let cmd = Command::Stream {
                points: arr,
                cycles_per_point: cycles,
                replace: replace && i == 0,
                batch_remaining,
                first: i == 0,
            };
            if let Err(e) = self.send(cmd) {
                // Another caller filled the command queue or the driver closed: the points not
                // pushed never reach the real-time thread.
                self.shared.in_flight.fetch_sub(points.len() - sent, Ordering::AcqRel);
                return Err(e);
            }
            sent += chunk.len();
        }
        drop(limits);
        Ok(())
    }

    /// Smooth (quintic) move from the current command to `q` (rad) over `duration`.
    pub fn move_to(&self, q: [f64; 6], duration: Duration) -> Result<()> {
        self.require_active()?;
        let cycles = cycles_for(duration, self.cfg.cycle, "duration")?;
        // Held across the push: the move never lands between the chunks of a `stream` call.
        let limits = self.motion.lock().unwrap();
        limits.check_position(&q)?;
        self.send(Command::MoveTo { q, cycles })
    }

    /// Drop the queue and hold the current command.
    pub fn hold(&self) -> Result<()> {
        // Held across the push: the hold never lands between the chunks of a `stream` call, whose
        // later chunks would then play from the hold point. A poisoned lock must not prevent a hold.
        let _motion = self.motion.lock().unwrap_or_else(PoisonError::into_inner);
        self.send(Command::Hold)
    }

    /// Stop the robot through `EStr` and freeze the command until the next session. Allowed on
    /// a faulted session too (the message is sent again).
    pub fn fault(&self, msg: &str) -> Result<()> {
        self.require_session()?;
        self.send(Command::Fault(short(msg)))
    }

    /// Show a notification on the smartHMI.
    pub fn notify(&self, msg: &str) -> Result<()> {
        self.require_active()?;
        self.send(Command::Notify(short(msg)))
    }

    /// Pulse the stop channel so the KRL program leaves `RSI_MOVECORR()`. Allowed on a faulted
    /// session too: it is the way out when the robot did not stop on the fault message.
    pub fn stop_program(&self) -> Result<()> {
        if self.cfg.stop_channel.is_none() {
            return Err(Error::Config("no stop channel configured (DriverConfig::stop_channel)".into()));
        }
        self.require_session()?;
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
        // The API-side copy changes only once the command is queued, so a failed push leaves both
        // sides unchanged. `stream` holds this lock from validation through its last push, so the
        // new limits land before or after a whole call, never between its chunks, and every point
        // queued after them was validated against them.
        let mut cur = self.motion.lock().unwrap();
        self.send(Command::SetLimits(l))?;
        *cur = l;
        Ok(())
    }

    /// End the session cleanly if possible, then stop the thread. Idempotent; a concurrent second
    /// call returns once the thread has been joined.
    pub fn close(&self) -> Result<()> {
        let mut rt = self.rt.lock().unwrap_or_else(PoisonError::into_inner);
        let Some(handle) = rt.take() else {
            return Ok(());
        };
        if self.cfg.stop_channel.is_some() && self.session() == SessionState::Active {
            let _ = self.send(Command::StopProgram);
            let t0 = Instant::now();
            while self.session() == SessionState::Active && t0.elapsed() < Duration::from_secs(1) {
                std::thread::sleep(Duration::from_millis(5));
            }
        }
        self.shared.stop.store(true, Ordering::Relaxed);
        let joined = handle.join();
        // The monitor outlives the real-time thread so its last events reach the history.
        self.shared.monitor_stop.store(true, Ordering::Release);
        if let Some(m) = self.monitor.lock().unwrap_or_else(PoisonError::into_inner).take() {
            m.thread().unpark();
            let _ = m.join();
        }
        joined.map_err(|_| Error::Protocol("real-time thread panicked".into()))
    }
}

impl Drop for Driver {
    fn drop(&mut self) {
        let _ = self.close();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::stats::ClampKind;

    #[test]
    fn robot_addr_packs_ipv4_and_drops_ipv6() {
        for a in ["192.168.10.147:59152", "0.0.0.0:0", "255.255.255.255:65535"] {
            let a: SocketAddr = a.parse().unwrap();
            assert_eq!(unpack_addr(pack_addr(Some(a))), Some(a));
        }
        assert_eq!(unpack_addr(pack_addr(None)), None);
        assert_eq!(unpack_addr(pack_addr(Some("[::1]:5".parse().unwrap()))), None);
    }

    #[test]
    fn rate_limiter_allows_one_per_kind_per_interval() {
        let mut rl = RateLimiter::new(Duration::from_secs(1));
        let t0 = Instant::now();
        assert!(rl.allow(&Event::LateReply { latency_us: 1 }, t0));
        assert!(!rl.allow(&Event::LateReply { latency_us: 2 }, t0 + Duration::from_millis(500)));
        assert!(rl.allow(&Event::Clamped(ClampKind::Velocity), t0 + Duration::from_millis(500)), "different kind");
        assert!(!rl.allow(&Event::Clamped(ClampKind::Position), t0 + Duration::from_millis(600)), "same kind: Clamped");
        assert!(rl.allow(&Event::LateReply { latency_us: 3 }, t0 + Duration::from_millis(1001)));
    }

    #[test]
    fn event_level_split() {
        assert_eq!(level_of(&Event::LinkLost), log::Level::Warn);
        assert_eq!(level_of(&Event::RobotConnected { ipoc: 1 }), log::Level::Info);
        assert_eq!(level_of(&Event::RtPriority { ok: false }), log::Level::Warn);
        assert_eq!(level_of(&Event::RtPriority { ok: true }), log::Level::Info);
    }
}
