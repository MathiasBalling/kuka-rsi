use std::net::{SocketAddr, UdpSocket};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use kuka_rsi::config::EthernetConfig;
use kuka_rsi::doctor::{self, Check, Options, Progress, Status};
use kuka_rsi::sim::{ControllerSim, SimConfig, SimExit};

const CONFIG: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/controller/kuka_rsi_ethernet.xml");

fn free_loopback_port() -> SocketAddr {
    UdpSocket::bind("127.0.0.1:0").unwrap().local_addr().unwrap()
}

fn find<'a>(checks: &'a [Check], name: &str) -> &'a Check {
    checks.iter().find(|c| c.name == name).unwrap_or_else(|| panic!("no {name} check in {checks:#?}"))
}

fn run(opts: &Options) -> Vec<Check> {
    let mut reported = Vec::new();
    let checks = doctor::run(opts, |p| {
        if let Progress::Check(c) = p {
            reported.push(c.clone());
        }
    });
    assert_eq!(reported, checks, "every check is reported as it completes");
    checks
}

#[test]
fn a_missing_config_stops_after_the_config_check() {
    let checks = run(&Options::new("/nonexistent/kuka_rsi_ethernet.xml"));
    assert_eq!(checks.len(), 1, "{checks:#?}");
    assert_eq!(checks[0].name, "config");
    assert_eq!(checks[0].status, Status::Fail);
}

#[test]
fn offline_checks_pass_on_loopback() {
    let mut opts = Options::new(CONFIG);
    opts.bind = Some(free_loopback_port());
    opts.robot = Some("127.0.0.1".parse().unwrap());
    let checks = run(&opts);
    let config = find(&checks, "config");
    assert_eq!(config.status, Status::Pass, "{config:?}");
    assert!(config.detail.contains("KukaRsi") && config.detail.contains("MoveStop"), "{config:?}");
    for name in ["address", "port", "route"] {
        assert_eq!(find(&checks, name).status, Status::Pass, "{:?}", find(&checks, name));
    }
    assert_ne!(find(&checks, "realtime").status, Status::Fail);
    assert_eq!(find(&checks, "robot link").status, Status::Skip);
    assert!(checks.iter().all(|c| c.name != "timing"), "no timing check without a robot link");
}

#[test]
fn route_and_ping_are_skipped_without_a_robot_address() {
    let mut opts = Options::new(CONFIG);
    opts.bind = Some(free_loopback_port());
    let checks = run(&opts);
    assert_eq!(find(&checks, "route").status, Status::Skip);
    assert_eq!(find(&checks, "ping").status, Status::Skip);
}

#[test]
fn a_port_in_use_fails() {
    let taken = UdpSocket::bind("127.0.0.1:0").unwrap();
    let mut opts = Options::new(CONFIG);
    opts.bind = Some(taken.local_addr().unwrap());
    let checks = run(&opts);
    assert_eq!(find(&checks, "address").status, Status::Pass);
    let port = find(&checks, "port");
    assert_eq!(port.status, Status::Fail, "{port:?}");
    assert!(port.hint.is_some());
}

#[test]
fn an_address_not_on_this_pc_fails_and_skips_the_port() {
    let mut opts = Options::new(CONFIG);
    opts.bind = Some("192.0.2.1:49152".parse().unwrap()); // TEST-NET-1, never assigned
    let checks = run(&opts);
    let address = find(&checks, "address");
    assert_eq!(address.status, Status::Fail, "{address:?}");
    assert!(address.hint.as_deref().unwrap().contains("192.0.2.1"), "{address:?}");
    assert_eq!(find(&checks, "port").status, Status::Skip);
}

#[test]
fn live_check_holds_the_simulated_robot_and_ends_its_program() {
    let bind = free_loopback_port();
    let mut opts = Options::new(CONFIG);
    opts.bind = Some(bind);
    opts.robot = Some("127.0.0.1".parse().unwrap());
    opts.wait = Duration::from_secs(5);
    opts.hold = Duration::from_secs(1);
    let (listening, rx) = std::sync::mpsc::channel();
    let doctor = std::thread::spawn(move || {
        doctor::run(&opts, |p| {
            if let Progress::Listening { addr, .. } = p {
                listening.send(addr).unwrap();
            }
        })
    });
    // Like the operator at the pendant: start the robot program once the doctor listens.
    assert_eq!(rx.recv_timeout(Duration::from_secs(15)).unwrap(), bind);

    let mut sim_cfg = SimConfig::new(EthernetConfig::from_path(CONFIG).unwrap(), bind);
    sim_cfg.bind = "127.0.0.1:0".parse().unwrap();
    let sim = ControllerSim::spawn(sim_cfg).unwrap();
    let checks = doctor.join().unwrap();

    let link = find(&checks, "robot link");
    assert_eq!(link.status, Status::Pass, "{link:?}");
    assert!(link.detail.contains(&sim.local_addr().to_string()), "{link:?}");
    let timing = find(&checks, "timing");
    assert_ne!(timing.status, Status::Fail, "{timing:?}");
    assert!(timing.detail.contains("telegrams/s"), "{timing:?}");
    assert_eq!(sim.wait_exit(Duration::from_secs(3)), Some(SimExit::Stopped), "the doctor ends the program");
}

#[test]
fn live_check_without_a_robot_fails_with_a_hint() {
    let mut opts = Options::new(CONFIG);
    opts.bind = Some(free_loopback_port());
    opts.wait = Duration::from_millis(300);
    let checks = run(&opts);
    let link = find(&checks, "robot link");
    assert_eq!(link.status, Status::Fail, "{link:?}");
    assert!(link.hint.as_deref().unwrap().contains("Reset program"), "{link:?}");
}

#[test]
fn cancel_ends_the_live_wait() {
    let cancel = Arc::new(AtomicBool::new(false));
    let mut opts = Options::new(CONFIG);
    opts.bind = Some(free_loopback_port());
    opts.wait = Duration::from_secs(30);
    opts.cancel = Some(cancel.clone());
    let t0 = Instant::now();
    let doctor = std::thread::spawn(move || run(&opts));
    std::thread::sleep(Duration::from_millis(200));
    cancel.store(true, Ordering::SeqCst);
    let checks = doctor.join().unwrap();
    assert!(t0.elapsed() < Duration::from_secs(3), "{:?}", t0.elapsed());
    let link = find(&checks, "robot link");
    assert_eq!(link.status, Status::Skip, "{link:?}");
    assert!(link.detail.contains("cancelled"), "{link:?}");
}

#[test]
fn ping_output_parses_on_macos_and_linux() {
    let macos = "PING 192.168.10.147 (192.168.10.147): 56 data bytes\n\
        64 bytes from 192.168.10.147: icmp_seq=0 ttl=64 time=0.512 ms\n\n\
        --- 192.168.10.147 ping statistics ---\n\
        3 packets transmitted, 3 packets received, 0.0% packet loss\n\
        round-trip min/avg/max/stddev = 0.402/0.455/0.512/0.045 ms\n";
    let p = doctor::parse_ping(macos).unwrap();
    assert_eq!((p.sent, p.received), (3, 3));
    assert!((p.avg_ms.unwrap() - 0.455).abs() < 1e-9);

    let linux = "--- 192.168.10.147 ping statistics ---\n\
        3 packets transmitted, 2 received, 33.3333% packet loss, time 2003ms\n\
        rtt min/avg/max/mdev = 0.301/0.350/0.399/0.049 ms\n";
    let p = doctor::parse_ping(linux).unwrap();
    assert_eq!((p.sent, p.received), (3, 2));
    assert!((p.avg_ms.unwrap() - 0.350).abs() < 1e-9);

    let lost =
        "--- 192.168.10.147 ping statistics ---\n3 packets transmitted, 0 packets received, 100.0% packet loss\n";
    let p = doctor::parse_ping(lost).unwrap();
    assert_eq!((p.sent, p.received, p.avg_ms), (3, 0, None));

    assert_eq!(doctor::parse_ping("ping: cannot resolve robot: Unknown host"), None);
}

#[test]
fn cancel_during_the_hold_still_ends_the_program() {
    let bind = free_loopback_port();
    let cancel = Arc::new(AtomicBool::new(false));
    let mut opts = Options::new(CONFIG);
    opts.bind = Some(bind);
    opts.wait = Duration::from_secs(5);
    opts.hold = Duration::from_secs(30);
    opts.cancel = Some(cancel.clone());
    let (listening, rx) = std::sync::mpsc::channel();
    let doctor = std::thread::spawn(move || {
        doctor::run(&opts, |p| {
            if let Progress::Listening { .. } = p {
                listening.send(()).unwrap();
            }
        })
    });
    rx.recv_timeout(Duration::from_secs(15)).unwrap();
    let mut sim_cfg = SimConfig::new(EthernetConfig::from_path(CONFIG).unwrap(), bind);
    sim_cfg.bind = "127.0.0.1:0".parse().unwrap();
    let sim = ControllerSim::spawn(sim_cfg).unwrap();
    std::thread::sleep(Duration::from_millis(500));
    let t0 = Instant::now();
    cancel.store(true, Ordering::SeqCst);
    let checks = doctor.join().unwrap();
    assert!(t0.elapsed() < Duration::from_secs(3), "{:?}", t0.elapsed());
    assert_eq!(find(&checks, "robot link").status, Status::Pass);
    assert_eq!(find(&checks, "timing").status, Status::Skip);
    assert_eq!(sim.wait_exit(Duration::from_secs(3)), Some(SimExit::Stopped));
}
