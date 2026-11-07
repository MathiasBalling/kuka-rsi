//! Setup checks for the PC side of an RSI connection: the config, the network address and port,
//! the route and reachability of the robot, real-time priority, and (with the robot program
//! running) the live link and its timing. `rsi-doctor` and Python's `kuka_rsi.check_setup` run
//! them.
//!
//! The live check answers the robot the way `rsi-monitor` does: it holds the robot where it
//! stands for [`Options::hold`], then ends the program with the stop channel (`MoveStop`).

use std::io::ErrorKind;
use std::net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket};
use std::path::PathBuf;
use std::process::Command;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use serde::Serialize;

use crate::config::{DriverConfig, EthernetConfig};
use crate::driver::Driver;
use crate::state::SessionState;
use crate::stats::Stats;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum Status {
    Pass,
    /// Works, but something may bite later.
    Warn,
    Fail,
    /// Not run: missing input or an earlier failure.
    Skip,
}

impl Status {
    pub fn label(self) -> &'static str {
        match self {
            Status::Pass => "PASS",
            Status::Warn => "WARN",
            Status::Fail => "FAIL",
            Status::Skip => "SKIP",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Check {
    pub name: &'static str,
    pub status: Status,
    pub detail: String,
    /// What to do about a failure or warning.
    pub hint: Option<String>,
}

impl Check {
    fn new(name: &'static str, status: Status, detail: impl Into<String>) -> Check {
        Check { name, status, detail: detail.into(), hint: None }
    }

    fn hint(mut self, hint: impl Into<String>) -> Check {
        self.hint = Some(hint.into());
        self
    }
}

/// What `run` reports while it works.
#[derive(Clone, Copy, Debug)]
pub enum Progress<'a> {
    /// The live check is listening: start the robot program now.
    Listening { addr: SocketAddr, wait: Duration },
    /// A check completed.
    Check(&'a Check),
}

#[derive(Clone, Debug)]
pub struct Options {
    /// The RSI Ethernet config XML, the same file as on the robot; `None` checks the shipped one
    /// ([`EthernetConfig::shipped`](crate::config::EthernetConfig::shipped)).
    pub config: Option<PathBuf>,
    /// Bind address override; default `IP_NUMBER:PORT` from the config.
    pub bind: Option<SocketAddr>,
    /// The robot's RSI address (the KLI sensor interface); enables the route and ping checks.
    pub robot: Option<IpAddr>,
    /// How long to wait for the robot's first telegram; zero skips the live checks.
    pub wait: Duration,
    /// How long to hold the robot and measure the timing once it is connected.
    pub hold: Duration,
    /// Channel that ends `RSI_MOVECORR()` when the live check is done.
    pub stop_channel: String,
    /// Set to end the live checks early (Ctrl-C); the program is still ended cleanly.
    pub cancel: Option<Arc<AtomicBool>>,
}

impl Default for Options {
    /// The shipped config, no robot address, no live checks.
    fn default() -> Options {
        Options {
            config: None,
            bind: None,
            robot: None,
            wait: Duration::ZERO,
            hold: Duration::from_secs(3),
            stop_channel: "MoveStop".into(),
            cancel: None,
        }
    }
}

impl Options {
    /// [`Options::default`] with the config file at `config`.
    pub fn new(config: impl Into<PathBuf>) -> Options {
        Options { config: Some(config.into()), ..Options::default() }
    }

    fn cancelled(&self) -> bool {
        self.cancel.as_ref().is_some_and(|c| c.load(Ordering::SeqCst))
    }
}

/// Run every check in order, calling `report` as each one completes (and when the live check
/// starts listening), and return them all.
pub fn run(opts: &Options, mut report: impl FnMut(Progress)) -> Vec<Check> {
    let mut checks = Vec::new();
    let mut add = |c: Check, report: &mut dyn FnMut(Progress)| {
        report(Progress::Check(&c));
        checks.push(c);
    };

    let (cfg, check) = config(opts);
    add(check, &mut report);
    let Some(cfg) = cfg else { return checks };
    let bind = match opts.bind.map(Ok).unwrap_or_else(|| cfg.bind_addr()) {
        Ok(b) => b,
        Err(e) => {
            add(Check::new("address", Status::Fail, format!("no bind address: {e}")), &mut report);
            return checks;
        }
    };

    let address_ok = {
        let c = address(bind.ip());
        let ok = c.status == Status::Pass;
        add(c, &mut report);
        ok
    };
    add(
        if address_ok { port(bind) } else { Check::new("port", Status::Skip, "needs the address on this PC") },
        &mut report,
    );
    add(
        if address_ok {
            route(bind.ip(), opts.robot)
        } else {
            Check::new("route", Status::Skip, "needs the address on this PC")
        },
        &mut report,
    );
    add(ping(opts.robot), &mut report);
    add(realtime(), &mut report);
    if let Some(c) = firewall() {
        add(c, &mut report);
    }
    if !address_ok {
        add(Check::new("robot link", Status::Skip, "needs the address on this PC"), &mut report);
        return checks;
    }
    for c in live(opts, cfg.bind(bind), &mut report) {
        add(c, &mut report);
    }
    checks
}

fn config(opts: &Options) -> (Option<DriverConfig>, Check) {
    let source = match &opts.config {
        Some(p) => p.display().to_string(),
        None => "built-in kuka_rsi_ethernet.xml".to_string(),
    };
    let cfg = match EthernetConfig::from_path_or_shipped(opts.config.as_deref()) {
        Ok(e) => DriverConfig::new(e),
        Err(e) => {
            let c = Check::new("config", Status::Fail, format!("{source}: {e}")).hint(
                "Pass the RSI Ethernet config the robot uses, or leave it out to check the built-in \
                 kuka_rsi_ethernet.xml (`python -m kuka_rsi.controller DIR` writes it).",
            );
            return (None, c);
        }
    };
    let e = &cfg.ethernet;
    let mut detail = format!("{source}: IP_NUMBER {}, PORT {}, SENTYPE {}", e.ip_number, e.port, e.sentype);
    if e.onlysend {
        detail.push_str(", ONLYSEND (monitor only, no replies)");
        return (Some(cfg), Check::new("config", Status::Pass, detail));
    }
    if has_channel(&cfg, &opts.stop_channel) {
        detail.push_str(&format!(", stop channel {}", opts.stop_channel));
        (Some(cfg), Check::new("config", Status::Pass, detail))
    } else {
        detail.push_str(&format!(", no {} channel", opts.stop_channel));
        let c = Check::new("config", Status::Warn, detail).hint(
            "Without a stop channel the PC cannot end RSI_MOVECORR(): the robot stops on its own after \
             the ETHERNET timeout with KSS00489 and the program needs a reset.",
        );
        (Some(cfg), c)
    }
}

fn has_channel(cfg: &DriverConfig, tag: &str) -> bool {
    cfg.ethernet.receive.iter().any(|e| e.tag == tag)
}

fn address(ip: IpAddr) -> Check {
    if ip.is_unspecified() {
        return Check::new("address", Status::Pass, format!("binding {ip} (every interface)"));
    }
    match UdpSocket::bind((ip, 0)) {
        Ok(_) => Check::new("address", Status::Pass, format!("{ip} is assigned to this PC")),
        Err(e) if e.kind() == ErrorKind::AddrNotAvailable => {
            Check::new("address", Status::Fail, format!("{ip} is not assigned to any network interface on this PC"))
                .hint(format!(
                    "Give the network port cabled to the robot the address {ip} (IP_NUMBER in the config) with \
                     the robot's subnet mask. macOS: System Settings > Network > the adapter > Details > TCP/IP > \
                     Manually (add a second service on the same adapter to keep another address). Linux: \
                     `sudo ip addr add {ip}/24 dev <interface>`. If it is set, the link may be down: check the \
                     cable and the port's link light."
                ))
        }
        Err(e) => Check::new("address", Status::Fail, format!("cannot use {ip}: {e}")),
    }
}

fn port(bind: SocketAddr) -> Check {
    match UdpSocket::bind(bind) {
        Ok(_) => Check::new("port", Status::Pass, format!("UDP {bind} is free")),
        Err(e) if e.kind() == ErrorKind::AddrInUse => {
            Check::new("port", Status::Fail, format!("UDP {bind} is already in use")).hint(format!(
                "Another program is bound to it (rsi-monitor, a driver in another process). \
                 `lsof -nP -iUDP:{}` shows which.",
                bind.port()
            ))
        }
        Err(e) => Check::new("port", Status::Fail, format!("cannot bind UDP {bind}: {e}")),
    }
}

/// The source address the OS picks for traffic to the robot must be the bind address, or the
/// robot is not on the bind address's subnet and the replies leave through another interface.
fn route(bind_ip: IpAddr, robot: Option<IpAddr>) -> Check {
    let Some(robot) = robot else {
        return Check::new("route", Status::Skip, "pass the robot's RSI address (--robot) to check the route");
    };
    let unspecified: IpAddr = if robot.is_ipv4() { Ipv4Addr::UNSPECIFIED.into() } else { bind_ip };
    let source = UdpSocket::bind((unspecified, 0)).and_then(|s| {
        s.connect((robot, 9))?; // UDP connect sends nothing; it only selects the route
        s.local_addr()
    });
    match source {
        Ok(src) if bind_ip.is_unspecified() || src.ip() == bind_ip => {
            Check::new("route", Status::Pass, format!("traffic to {robot} leaves from {}", src.ip()))
        }
        Ok(src) => Check::new(
            "route",
            Status::Fail,
            format!("traffic to {robot} leaves from {}, not from {bind_ip}", src.ip()),
        )
        .hint(format!(
            "The robot's RSI address and IP_NUMBER must share a subnet (for example 192.168.10.147 and \
             192.168.10.10 with mask 255.255.255.0). Check the robot address and the mask set for {bind_ip}."
        )),
        Err(e) => Check::new("route", Status::Fail, format!("no route to {robot}: {e}"))
            .hint(format!("Set {bind_ip} on the adapter cabled to the robot, with the robot's subnet mask.")),
    }
}

/// Result of a `ping` run.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ping {
    pub sent: u32,
    pub received: u32,
    pub avg_ms: Option<f64>,
}

/// Parse the summary of macOS or Linux `ping` output.
pub fn parse_ping(output: &str) -> Option<Ping> {
    let line = output.lines().find(|l| l.contains("packets transmitted"))?;
    let mut fields = line.split(',').map(str::trim);
    let sent = fields.next()?.split_whitespace().next()?.parse().ok()?;
    let received = fields.next()?.split_whitespace().next()?.parse().ok()?;
    let avg_ms = output
        .lines()
        .find(|l| l.contains("min/avg/max"))
        .and_then(|l| l.split('=').nth(1))
        .and_then(|v| v.trim().split('/').nth(1))
        .and_then(|v| v.parse().ok());
    Some(Ping { sent, received, avg_ms })
}

fn ping(robot: Option<IpAddr>) -> Check {
    let Some(robot) = robot else {
        return Check::new("ping", Status::Skip, "pass the robot's RSI address (--robot) to ping it");
    };
    let deadline = if cfg!(target_os = "macos") { "-t" } else { "-w" };
    let out = match Command::new("ping").args(["-c", "3", deadline, "4"]).arg(robot.to_string()).output() {
        Ok(o) => String::from_utf8_lossy(&o.stdout).into_owned(),
        Err(e) => return Check::new("ping", Status::Skip, format!("cannot run ping: {e}")),
    };
    // A warning, not a failure: the controller need not answer pings. When it does, a ping right
    // before a session also refreshes the PC's ARP entry for it.
    let unreachable_hint = "The controller may not answer pings at all; if the live check fails too, check the \
         cable and the link light, that the controller is on, and the robot's RSI address (the KLI's Mixed IP \
         sensor interface).";
    match parse_ping(&out) {
        Some(p) if p.received == 0 => {
            Check::new("ping", Status::Warn, format!("no reply from {robot} ({} sent)", p.sent)).hint(unreachable_hint)
        }
        Some(p) => {
            let avg = p.avg_ms.map(|a| format!(", average {a:.2} ms")).unwrap_or_default();
            let detail = format!("{}/{} replies from {robot}{avg}", p.received, p.sent);
            if p.received == p.sent {
                Check::new("ping", Status::Pass, detail)
            } else {
                Check::new("ping", Status::Warn, detail).hint("Lost pings point at a bad cable or link.")
            }
        }
        None => Check::new("ping", Status::Warn, format!("no reply from {robot}")).hint(unreachable_hint),
    }
}

fn realtime() -> Check {
    let ok = std::thread::spawn(crate::driver::raise_priority).join().unwrap_or(false);
    match (ok, std::env::consts::OS) {
        (true, "linux") => Check::new("realtime", Status::Pass, "SCHED_FIFO priority 80 granted"),
        (true, _) => Check::new("realtime", Status::Pass, "highest thread priority granted (no real-time class)"),
        (false, "linux") => {
            Check::new("realtime", Status::Warn, "SCHED_FIFO refused: the driver runs at normal priority").hint(
                "Grant an rtprio limit (/etc/security/limits.conf: `<user> - rtprio 90`, then log in again) or \
                 CAP_SYS_NICE (`sudo setcap cap_sys_nice+ep <binary>`). Without it, load on the PC can delay replies.",
            )
        }
        (false, os) => Check::new("realtime", Status::Warn, format!("cannot raise the thread priority on {os}")),
    }
}

/// macOS's application firewall can drop the robot's telegrams to an unsigned binary silently.
fn firewall() -> Option<Check> {
    if !cfg!(target_os = "macos") {
        return None;
    }
    let out = Command::new("/usr/libexec/ApplicationFirewall/socketfilterfw").arg("--getglobalstate").output().ok()?;
    let text = String::from_utf8_lossy(&out.stdout).to_lowercase();
    Some(if text.contains("enabled") {
        Check::new("firewall", Status::Warn, "the macOS firewall is on").hint(
            "If the robot link check gets no telegrams, allow this program under System Settings > Network > \
             Firewall > Options, or turn the firewall off on the robot network.",
        )
    } else {
        Check::new("firewall", Status::Pass, "the macOS firewall is off")
    })
}

/// Sleep in short steps until `d` has passed or the run is cancelled; returns false if cancelled.
fn wait_or_cancel(opts: &Options, d: Duration, mut done: impl FnMut() -> bool) -> bool {
    let t0 = Instant::now();
    while t0.elapsed() < d && !done() {
        if opts.cancelled() {
            return false;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    !opts.cancelled()
}

fn live(opts: &Options, mut cfg: DriverConfig, report: &mut dyn FnMut(Progress)) -> Vec<Check> {
    if opts.wait.is_zero() {
        return vec![Check::new(
            "robot link",
            Status::Skip,
            "start kuka_rsi (or kuka_rsi_monitor) on the robot and pass a wait time (--wait 30) to check the link",
        )];
    }
    let onlysend = cfg.ethernet.onlysend;
    if !onlysend && has_channel(&cfg, &opts.stop_channel) {
        cfg = cfg.stop_channel(&opts.stop_channel);
    }
    let drv = match Driver::spawn(cfg) {
        Ok(d) => d,
        Err(e) => return vec![Check::new("robot link", Status::Fail, format!("cannot start the driver: {e}"))],
    };
    report(Progress::Listening { addr: drv.local_addr(), wait: opts.wait });
    let connected = wait_or_cancel(opts, opts.wait, || drv.session() == SessionState::Active);
    let checks = if !connected {
        vec![Check::new("robot link", Status::Skip, "cancelled while waiting for the robot")]
    } else if drv.session() != SessionState::Active {
        vec![no_link(opts, &drv.stats())]
    } else {
        let mut checks = vec![link(opts, &drv)];
        checks.extend(timing(opts, &drv, onlysend));
        checks
    };
    let _ = drv.close();
    checks
}

fn no_link(opts: &Options, s: &Stats) -> Check {
    let secs = opts.wait.as_secs_f64();
    if s.sentype_mismatches > 0 {
        Check::new(
            "robot link",
            Status::Fail,
            format!("{} telegrams with a root other than <Rob Type=\"KUKA\">", s.sentype_mismatches),
        )
        .hint("Something other than an RSI 5.0 robot sends to this port.")
    } else if s.parse_errors > 0 {
        Check::new("robot link", Status::Fail, format!("{} telegrams arrived but do not match the config", s.parse_errors))
            .hint("The robot's copy of the Ethernet config (or its .rsix) differs from this one: copy both to the robot again.")
    } else {
        Check::new("robot link", Status::Fail, format!("no telegram in {secs:.0} s")).hint(
            "Is kuka_rsi (or kuka_rsi_monitor) running on the pendant, past RSI_ON? After a stop, Edit > Reset \
             program before starting it again. The robot sends to IP_NUMBER:PORT from its copy of the config, \
             which must match this PC. A firewall can drop the telegrams.",
        )
    }
}

fn link(opts: &Options, drv: &Driver) -> Check {
    let from = drv.robot_addr().map(|a| a.to_string()).unwrap_or_else(|| "an unknown address".into());
    let detail = format!("telegrams from {from}");
    match (opts.robot, drv.robot_addr()) {
        (Some(want), Some(got)) if got.ip() != want => {
            Check::new("robot link", Status::Warn, format!("{detail}, not from {want}"))
                .hint(format!("The robot's RSI address is {}; use that for --robot.", got.ip()))
        }
        _ => Check::new("robot link", Status::Pass, detail),
    }
}

fn timing(opts: &Options, drv: &Driver, onlysend: bool) -> Option<Check> {
    let s0 = drv.stats();
    let t0 = Instant::now();
    let link_lost = || drv.session() != SessionState::Active;
    if !wait_or_cancel(opts, opts.hold, link_lost) {
        return Some(Check::new("timing", Status::Skip, "cancelled"));
    }
    let elapsed = t0.elapsed().as_secs_f64();
    let s1 = drv.stats();
    let state = drv.state();
    if drv.session() != SessionState::Active {
        return Some(
            Check::new("timing", Status::Fail, format!("the link was lost after {elapsed:.1} s of holding"))
                .hint("The robot stopped answering or gave up on late replies; see the pendant's messages."),
        );
    }
    let cycles = s1.cycles - s0.cycles;
    let rate = cycles as f64 / elapsed;
    let mut detail = format!("{rate:.0} telegrams/s, longest gap {:.1} ms", s1.interarrival_max_us as f64 / 1000.0);
    let mut problems = Vec::new();
    if !(240.0..=260.0).contains(&rate) {
        problems.push(format!("expected 250 telegrams/s (4 ms cycle), got {rate:.0}"));
    }
    if !onlysend {
        let replies = s1.replies - s0.replies;
        let late = s1.late_replies - s0.late_replies;
        detail.push_str(&format!(
            ", {late} of {replies} replies late, reply latency mean {:.2} ms, max {:.2} ms, robot Delay {}",
            s1.reply_latency_mean_us / 1000.0,
            s1.reply_latency_max_us as f64 / 1000.0,
            s1.robot_delay
        ));
        if late > 0 {
            problems.push(format!("{late} late replies"));
        }
    }
    if !state.program_running {
        problems.push("the program is not running (stopped on the pendant?)".into());
    }
    detail.push_str(&format!(", override {:.0} %", state.speed_override * 100.0));
    Some(if problems.is_empty() {
        Check::new("timing", Status::Pass, detail)
    } else {
        Check::new("timing", Status::Warn, detail).hint(format!(
            "{}. Late replies come from load on the PC, missing real-time priority, or a stale ARP entry \
             (ping the robot right before a session).",
            problems.join("; ")
        ))
    })
}
