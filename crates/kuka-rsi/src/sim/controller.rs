//! Free-running fake controller: sends `<Rob>` every cycle, expects `<Sen>` in time.

use std::io::ErrorKind;
use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use super::wire::{parse_sen, write_rob};
use crate::config::EthernetConfig;
use crate::state::StateMap;
use crate::telegram::{RobotTelegram, Schema, SensorTelegram};
use crate::{Error, Result};

#[derive(Clone, Debug)]
pub struct SimConfig {
    pub ethernet: EthernetConfig,
    /// Where telegrams are sent: the driver's bind address.
    pub target: SocketAddr,
    pub bind: SocketAddr,
    pub cycle: Duration,
    /// Late packets tolerated before `RsiBad` (ETHERNET `Timeout`).
    pub timeout: u32,
    pub q_start_deg: [f64; 6],
    pub axiscorr_lower_deg: [f64; 6],
    pub axiscorr_upper_deg: [f64; 6],
    pub axiscorrmon_deg: [f64; 6],
    /// First-order lag of the actual position behind the setpoint, in cycles (0 = none).
    pub lag_cycles: f64,
    pub max_velocity_deg_s: Option<f64>,
    pub ipoc_start: u64,
    pub precision: usize,
    /// Uniform ±jitter added to each send time.
    pub jitter: Duration,
    pub drop_probability: f64,
    pub override_percent: f64,
    pub program_running: bool,
    pub stop_channel: Option<String>,
    pub max_cycles: Option<u64>,
}

impl SimConfig {
    pub fn new(ethernet: EthernetConfig, target: SocketAddr) -> SimConfig {
        SimConfig {
            ethernet,
            target,
            bind: "0.0.0.0:0".parse().unwrap(),
            cycle: Duration::from_millis(4),
            timeout: 100,
            q_start_deg: [0.0, -90.0, 90.0, 0.0, 90.0, 0.0],
            axiscorr_lower_deg: [-170.0, -190.0, -120.0, -185.0, -120.0, -350.0],
            axiscorr_upper_deg: [170.0, 45.0, 156.0, 185.0, 120.0, 350.0],
            axiscorrmon_deg: [340.0, 235.0, 276.0, 370.0, 240.0, 700.0],
            lag_cycles: 0.0,
            max_velocity_deg_s: None,
            ipoc_start: 1_000_000,
            precision: 4,
            jitter: Duration::ZERO,
            drop_probability: 0.0,
            override_percent: 100.0,
            program_running: true,
            stop_channel: Some("MoveStop".into()),
            max_cycles: None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SimExit {
    Stopped,
    RsiBad { late: u32 },
    CorrMonExceeded { axis: usize },
    EStrError,
    MaxCycles,
    Shutdown,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SimStatus {
    pub cycles: u64,
    pub replies: u64,
    pub late: u32,
    pub ipoc: u64,
    pub q_setpoint_deg: [f64; 6],
    pub q_actual_deg: [f64; 6],
    pub last_ak_deg: [f64; 6],
    pub clamped: u64,
    pub exit: Option<SimExit>,
}

struct XorShift(u64);

impl XorShift {
    fn next_f64(&mut self) -> f64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        (x >> 11) as f64 / (1u64 << 53) as f64
    }
}

pub struct ControllerSim {
    cfg: SimConfig,
    schema: Schema,
    socket: UdpSocket,
    status: Arc<Mutex<SimStatus>>,
    stop: Arc<AtomicBool>,
}

impl ControllerSim {
    pub fn new(cfg: SimConfig) -> Result<ControllerSim> {
        let schema = Schema::compile(&cfg.ethernet)?;
        let socket = UdpSocket::bind(cfg.bind)?;
        Ok(ControllerSim { cfg, schema, socket, status: Arc::new(Mutex::new(SimStatus::default())), stop: Arc::new(AtomicBool::new(false)) })
    }

    pub fn local_addr(&self) -> Result<SocketAddr> {
        Ok(self.socket.local_addr()?)
    }

    /// Run in the current thread until an exit condition.
    pub fn run(self) -> Result<SimExit> {
        let exit = self.run_inner();
        let mut st = self.status.lock().unwrap();
        st.exit = exit.as_ref().ok().copied();
        exit
    }

    fn run_inner(&self) -> Result<SimExit> {
        let cfg = &self.cfg;
        let map = StateMap::from_schema(&self.schema);
        let stop_slot = cfg.stop_channel.as_deref().and_then(|t| self.schema.sen_slot(t, None));
        let mut rng = XorShift(0x9E37_79B9_7F4A_7C15);
        let mut tele = RobotTelegram { aspos: cfg.q_start_deg, aipos: cfg.q_start_deg, ..Default::default() };
        if let Some(s) = map.prog_status {
            tele.slots[s] = if cfg.program_running { 3.0 } else { 4.0 };
        }
        if let Some(s) = map.ov_pro {
            tele.slots[s] = cfg.override_percent;
        }
        let mut sen = SensorTelegram::default();
        let mut out = Vec::with_capacity(4096);
        let mut rx = vec![0u8; 65536];
        let mut ipoc = cfg.ipoc_start;
        let ipoc_step = (cfg.cycle.as_millis() as u64).max(1);
        let mut last_ak = [0.0; 6];
        let mut stop_prev = false;
        let mut late: u32 = 0;
        let mut q_actual = cfg.q_start_deg;
        let mut cycles: u64 = 0;
        let mut replies: u64 = 0;
        let mut clamped: u64 = 0;
        let alpha = if cfg.lag_cycles > 0.0 { 1.0 - (-1.0 / cfg.lag_cycles).exp() } else { 1.0 };
        let mut next = Instant::now();
        loop {
            if self.stop.load(Ordering::Relaxed) {
                return Ok(SimExit::Shutdown);
            }
            if cfg.max_cycles.is_some_and(|m| cycles >= m) {
                return Ok(SimExit::MaxCycles);
            }
            tele.ipoc = ipoc;
            tele.delay = late;
            tele.aipos = q_actual;
            write_rob(&self.schema, &tele, cfg.precision, &mut out);
            if cfg.drop_probability <= 0.0 || rng.next_f64() >= cfg.drop_probability {
                self.socket.send_to(&out, cfg.target)?;
            }
            cycles += 1;
            let deadline = next + cfg.cycle;
            let mut got = false;
            if !self.schema.onlysend {
                loop {
                    let now = Instant::now();
                    if now >= deadline {
                        break;
                    }
                    self.socket.set_read_timeout(Some(deadline - now))?;
                    match self.socket.recv_from(&mut rx) {
                        Ok((n, _)) => {
                            if parse_sen(&self.schema, &rx[..n], &mut sen).is_ok() && sen.ipoc == ipoc {
                                got = true;
                                break;
                            }
                        }
                        Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => break,
                        Err(e) => return Err(Error::Io(e)),
                    }
                }
            }
            if got {
                replies += 1;
                last_ak = sen.ak;
                if let Some(slot) = stop_slot {
                    let stop_now = sen.slots[slot] >= 0.5;
                    if stop_now && !stop_prev {
                        return Ok(SimExit::Stopped);
                    }
                    stop_prev = stop_now;
                }
                if sen.estr.starts_with("Error:") {
                    return Ok(SimExit::EStrError);
                }
            } else if !self.schema.onlysend {
                late += 1;
                stop_prev = false; // HOLDON="0" resets the stop channel
                if late > cfg.timeout {
                    return Ok(SimExit::RsiBad { late });
                }
            }
            let mut setpoint = [0.0; 6];
            for i in 0..6 {
                let ak = last_ak[i];
                let c = ak.clamp(cfg.axiscorr_lower_deg[i], cfg.axiscorr_upper_deg[i]);
                if c != ak {
                    clamped += 1;
                }
                if c.abs() > cfg.axiscorrmon_deg[i] {
                    return Ok(SimExit::CorrMonExceeded { axis: i });
                }
                setpoint[i] = cfg.q_start_deg[i] + c;
                let mut step = (setpoint[i] - q_actual[i]) * alpha;
                if let Some(v) = cfg.max_velocity_deg_s {
                    let vmax = v * cfg.cycle.as_secs_f64();
                    step = step.clamp(-vmax, vmax);
                }
                q_actual[i] += step;
            }
            {
                let mut st = self.status.lock().unwrap();
                st.cycles = cycles;
                st.replies = replies;
                st.late = late;
                st.ipoc = ipoc;
                st.q_setpoint_deg = setpoint;
                st.q_actual_deg = q_actual;
                st.last_ak_deg = last_ak;
                st.clamped = clamped;
            }
            ipoc += ipoc_step;
            next += cfg.cycle;
            let mut wake = next;
            if !cfg.jitter.is_zero() {
                let j = (rng.next_f64() * 2.0 - 1.0) * cfg.jitter.as_secs_f64();
                wake = if j >= 0.0 { next + Duration::from_secs_f64(j) } else { next - Duration::from_secs_f64(-j) };
            }
            let now = Instant::now();
            if wake > now {
                std::thread::sleep(wake - now);
            }
        }
    }

    /// Run in a background thread.
    pub fn spawn(cfg: SimConfig) -> Result<SimHandle> {
        let sim = ControllerSim::new(cfg)?;
        let addr = sim.local_addr()?;
        let status = sim.status.clone();
        let stop = sim.stop.clone();
        let thread = std::thread::Builder::new().name("rsi-sim".into()).spawn(move || sim.run())?;
        Ok(SimHandle { status, stop, thread: Some(thread), addr })
    }
}

pub struct SimHandle {
    status: Arc<Mutex<SimStatus>>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<Result<SimExit>>>,
    addr: SocketAddr,
}

impl SimHandle {
    pub fn status(&self) -> SimStatus {
        *self.status.lock().unwrap()
    }

    pub fn local_addr(&self) -> SocketAddr {
        self.addr
    }

    pub fn is_finished(&self) -> bool {
        self.thread.as_ref().is_none_or(|t| t.is_finished())
    }

    /// Poll until the simulator exits on its own.
    pub fn wait_exit(&self, timeout: Duration) -> Option<SimExit> {
        let t0 = Instant::now();
        while t0.elapsed() < timeout {
            if let Some(e) = self.status().exit {
                return Some(e);
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        self.status().exit
    }

    /// Ask the simulator to stop and wait for it.
    pub fn stop(&mut self) -> Result<SimExit> {
        self.stop.store(true, Ordering::Relaxed);
        match self.thread.take() {
            Some(t) => t.join().map_err(|_| Error::Protocol("simulator thread panicked".into()))?,
            None => self.status().exit.ok_or(Error::Closed),
        }
    }
}

impl Drop for SimHandle {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::EthernetConfig;
    use crate::telegram::{Parser, RobotTelegram, Schema, SensorTelegram, Writer};
    use std::net::UdpSocket;

    const SHIPPED: &str = include_str!("../../../../controller/kuka_rsi_ethernet.xml");

    struct Pc {
        sock: UdpSocket,
        parser: Parser,
        writer: Writer,
        tele: RobotTelegram,
        buf: Vec<u8>,
        out: Vec<u8>,
    }

    impl Pc {
        fn new() -> Pc {
            let schema = Schema::compile(&EthernetConfig::from_str(SHIPPED).unwrap()).unwrap();
            let sock = UdpSocket::bind("127.0.0.1:0").unwrap();
            sock.set_read_timeout(Some(Duration::from_millis(500))).unwrap();
            Pc { sock, parser: Parser::new(&schema), writer: Writer::new(&schema), tele: RobotTelegram::default(), buf: vec![0; 65536], out: Vec::new() }
        }

        fn sim_config(&self, cycle_ms: u64) -> SimConfig {
            let mut c = SimConfig::new(EthernetConfig::from_str(SHIPPED).unwrap(), self.sock.local_addr().unwrap());
            c.bind = "127.0.0.1:0".parse().unwrap();
            c.cycle = Duration::from_millis(cycle_ms);
            c
        }

        /// Receive one telegram; return None on timeout.
        fn recv(&mut self) -> Option<SocketAddr> {
            let (n, from) = self.sock.recv_from(&mut self.buf).ok()?;
            self.parser.parse(&self.buf[..n], &mut self.tele).unwrap();
            Some(from)
        }

        fn reply(&mut self, to: SocketAddr, f: impl FnOnce(&mut SensorTelegram)) {
            let mut sen = SensorTelegram { ipoc: self.tele.ipoc, ..Default::default() };
            f(&mut sen);
            self.writer.write(&sen, &mut self.out);
            self.sock.send_to(&self.out, to).unwrap();
        }
    }

    #[test]
    fn streams_and_applies_ak() {
        let mut pc = Pc::new();
        let mut h = ControllerSim::spawn(pc.sim_config(2)).unwrap();
        let first_ipoc = { let from = pc.recv().unwrap(); pc.reply(from, |s| s.ak[0] = 1.0); pc.tele.ipoc };
        for _ in 0..20 {
            let from = pc.recv().unwrap();
            pc.reply(from, |s| s.ak[0] = 1.0);
        }
        assert_eq!(pc.tele.ipoc, first_ipoc + 20 * 2, "IPOC advances by the cycle in ms");
        let st = h.status();
        assert!((st.q_setpoint_deg[0] - 1.0).abs() < 1e-9, "{st:?}");
        assert!((st.q_actual_deg[0] - 1.0).abs() < 1e-9, "no lag");
        assert!((pc.tele.aipos[0] - 1.0).abs() < 1e-9, "reported back");
        assert_eq!(pc.tele.aspos[0], 0.0, "setpoint stays at the start pose");
        assert!(st.replies >= 20 && st.late <= 1, "{st:?}");
        assert_eq!(h.stop().unwrap(), SimExit::Shutdown);
    }

    #[test]
    fn counts_late_and_exits_rsibad() {
        let mut pc = Pc::new();
        let mut c = pc.sim_config(2);
        c.timeout = 5;
        let h = ControllerSim::spawn(c).unwrap();
        for _ in 0..3 {
            pc.recv().unwrap();
        }
        let exit = h.wait_exit(Duration::from_secs(2)).expect("exits");
        assert_eq!(exit, SimExit::RsiBad { late: 6 });
        assert_eq!(h.status().exit, Some(exit));
        assert!(pc.tele.delay >= 2, "late count is reported in Delay: {}", pc.tele.delay);
    }

    #[test]
    fn holds_last_ak_on_late_replies() {
        let mut pc = Pc::new();
        let mut h = ControllerSim::spawn(pc.sim_config(2)).unwrap();
        for _ in 0..5 {
            let from = pc.recv().unwrap();
            pc.reply(from, |s| s.ak[0] = 1.0);
        }
        for _ in 0..3 {
            pc.recv().unwrap();
        }
        let st = h.status();
        assert!((st.q_setpoint_deg[0] - 1.0).abs() < 1e-9);
        assert!(st.late >= 2 && st.late <= 4, "{st:?}");
        h.stop().unwrap();
    }

    #[test]
    fn exits_on_stop_pulse_and_on_estr_error() {
        let mut pc = Pc::new();
        let h = ControllerSim::spawn(pc.sim_config(2)).unwrap();
        let from = pc.recv().unwrap();
        pc.reply(from, |s| s.slots[6] = 1.0);
        assert_eq!(h.wait_exit(Duration::from_secs(1)), Some(SimExit::Stopped));

        let mut pc = Pc::new();
        let h = ControllerSim::spawn(pc.sim_config(2)).unwrap();
        let from = pc.recv().unwrap();
        pc.reply(from, |s| s.estr.push_str("Error: nope"));
        assert_eq!(h.wait_exit(Duration::from_secs(1)), Some(SimExit::EStrError));
    }

    #[test]
    fn clamps_axiscorr_and_trips_the_monitor() {
        let mut pc = Pc::new();
        let mut c = pc.sim_config(2);
        c.axiscorr_upper_deg[0] = 2.0;
        let mut h = ControllerSim::spawn(c).unwrap();
        for _ in 0..3 {
            let from = pc.recv().unwrap();
            pc.reply(from, |s| s.ak[0] = 5.0);
        }
        let st = h.status();
        assert!((st.q_setpoint_deg[0] - 2.0).abs() < 1e-9, "{st:?}");
        assert!(st.clamped >= 1);
        h.stop().unwrap();

        let mut pc = Pc::new();
        let mut c = pc.sim_config(2);
        c.axiscorrmon_deg[2] = 1.0;
        let h = ControllerSim::spawn(c).unwrap();
        let from = pc.recv().unwrap();
        pc.reply(from, |s| s.ak[2] = 5.0);
        assert_eq!(h.wait_exit(Duration::from_secs(1)), Some(SimExit::CorrMonExceeded { axis: 2 }));
    }

    #[test]
    fn lag_model_moves_actual_toward_setpoint() {
        let mut pc = Pc::new();
        let mut c = pc.sim_config(2);
        c.lag_cycles = 5.0;
        let mut h = ControllerSim::spawn(c).unwrap();
        let from = pc.recv().unwrap();
        pc.reply(from, |s| s.ak[0] = 1.0);
        pc.recv().unwrap();
        let st = h.status();
        assert!(st.q_actual_deg[0] > 0.05 && st.q_actual_deg[0] < 0.5, "{st:?}");
        for _ in 0..60 {
            let from = pc.recv().unwrap();
            pc.reply(from, |s| s.ak[0] = 1.0);
        }
        assert!((h.status().q_actual_deg[0] - 1.0).abs() < 1e-3);
        h.stop().unwrap();
    }

    #[test]
    fn max_cycles_ends_the_run() {
        let mut pc = Pc::new();
        let mut c = pc.sim_config(1);
        c.max_cycles = Some(10);
        c.ethernet.onlysend = true;
        let h = ControllerSim::spawn(c).unwrap();
        assert_eq!(h.wait_exit(Duration::from_secs(1)), Some(SimExit::MaxCycles));
        assert_eq!(h.status().cycles, 10);
        while pc.recv().is_some() {}
    }
}
