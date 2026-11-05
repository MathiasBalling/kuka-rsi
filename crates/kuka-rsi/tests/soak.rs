use std::time::{Duration, Instant};

use kuka_rsi::config::{DriverConfig, EthernetConfig, Limits};
use kuka_rsi::driver::Driver;
use kuka_rsi::sim::{ControllerSim, SimConfig};

const SHIPPED: &str = include_str!("../controller/kuka_rsi_ethernet.xml");

fn run(seconds: u64, rt: bool) -> (kuka_rsi::stats::Stats, kuka_rsi::sim::SimStatus) {
    let cfg = DriverConfig::new(EthernetConfig::from_str(SHIPPED).unwrap())
        .bind("127.0.0.1:0".parse().unwrap())
        .limits(Limits::kr10_defaults().with_max_velocity_all(2.0))
        .stop_channel("MoveStop")
        .rt_priority(rt);
    let drv = Driver::spawn(cfg).unwrap();
    let mut c = SimConfig::new(EthernetConfig::from_str(SHIPPED).unwrap(), drv.local_addr());
    c.bind = "127.0.0.1:0".parse().unwrap();
    let sim = ControllerSim::spawn(c).unwrap();
    drv.wait_for_robot(Duration::from_secs(3)).unwrap();
    let q0 = drv.state().q;
    let t0 = Instant::now();
    let mut k = 0u32;
    while t0.elapsed() < Duration::from_secs(seconds) {
        let mut q = q0;
        q[0] += 0.02 * (k as f64 * 0.3).sin();
        drv.stream(&[q], Duration::from_millis(100), false).unwrap();
        k += 1;
        std::thread::sleep(Duration::from_millis(100));
    }
    let stats = drv.stats();
    let status = sim.status();
    drv.close().unwrap();
    (stats, status)
}

/// Short version, always on: no more than 2 % late replies on a shared machine.
#[test]
fn two_seconds_mostly_on_time() {
    let (stats, status) = run(2, false);
    assert!(stats.replies > 400, "{stats:?}");
    assert!(stats.late_replies * 50 <= stats.replies, "driver late: {stats:?}");
    assert!((status.late as u64) * 50 <= status.cycles, "robot-side late: {status:?}");
    assert_eq!(stats.parse_errors, 0);
    assert_eq!(stats.io_errors, 0);
}

/// Long version for the cell PC: `cargo test --release --test soak -- --ignored --nocapture`.
#[test]
#[ignore]
fn sixty_seconds_zero_late() {
    let (stats, status) = run(60, true);
    println!("driver: {stats:#?}");
    println!("robot:  {status:#?}");
    assert_eq!(stats.late_replies, 0, "{stats:?}");
    assert_eq!(status.late, 0, "{status:?}");
    assert!(stats.reply_latency_max_us < 1000, "{stats:?}");
}
