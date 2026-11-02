use std::time::{Duration, Instant};

use kuka_rsi::config::{DriverConfig, EthernetConfig, Limits};
use kuka_rsi::driver::Driver;
use kuka_rsi::sim::{ControllerSim, SimConfig, SimExit, SimHandle, SimStatus};
use kuka_rsi::state::SessionState;
use kuka_rsi::stats::Event;
use kuka_rsi::Error;

const SHIPPED: &str = include_str!("../../../controller/kuka_rsi_ethernet.xml");

fn driver(link_timeout_ms: u64) -> Driver {
    let cfg = DriverConfig::new(EthernetConfig::from_str(SHIPPED).unwrap())
        .bind("127.0.0.1:0".parse().unwrap())
        .limits(Limits::kr10_defaults().with_max_velocity_all(2.0).with_max_acceleration_all(None))
        .stop_channel("MoveStop")
        .link_timeout(Duration::from_millis(link_timeout_ms))
        .rt_priority(false);
    Driver::spawn(cfg).unwrap()
}

fn sim_config(drv: &Driver) -> SimConfig {
    let mut c = SimConfig::new(EthernetConfig::from_str(SHIPPED).unwrap(), drv.local_addr());
    c.bind = "127.0.0.1:0".parse().unwrap();
    c
}

fn connected() -> (Driver, SimHandle) {
    let drv = driver(500);
    let sim = ControllerSim::spawn(sim_config(&drv)).unwrap();
    drv.wait_for_robot(Duration::from_secs(3)).unwrap();
    // The simulator publishes its status only at the end of its first cycle, after our reply
    // arrives; until then `status()` is all zeros.
    assert!(wait_until(Duration::from_secs(1), || sim.status().cycles > 0), "simulator published no status");
    (drv, sim)
}

fn wait_until(timeout: Duration, mut f: impl FnMut() -> bool) -> bool {
    let t0 = Instant::now();
    while t0.elapsed() < timeout {
        if f() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    f()
}

/// The simulator's status once it has applied the driver's current command. It publishes a cycle
/// only after it has received our reply, so right after `wait_idle` it can be one reply behind.
fn settled(drv: &Driver, sim: &SimHandle) -> SimStatus {
    let ak = drv.state().ak;
    let applied = || sim.status().last_ak_deg.iter().zip(&ak).all(|(s, d)| (s - d).abs() < 1e-6);
    assert!(wait_until(Duration::from_secs(1), applied), "simulator never applied {ak:?}: {:?}", sim.status());
    sim.status()
}

#[test]
fn handshake_captures_start_pose_and_close_stops_the_program() {
    let (drv, sim) = connected();
    let st = drv.state();
    assert_eq!(st.session, SessionState::Active);
    assert_eq!(drv.session(), SessionState::Active);
    let start = sim.status().q_setpoint_deg;
    for i in 0..6 {
        assert!((st.q[i] - start[i].to_radians()).abs() < 1e-6, "{st:?}");
        assert!(st.ak[i].abs() < 1e-9);
    }
    assert_eq!(drv.stats().sessions, 1);
    std::thread::sleep(Duration::from_millis(120)); // events reach the history via the monitor thread (Task 15)
    let ev = drv.events();
    assert!(ev.iter().any(|e| matches!(e, Event::RobotConnected { .. })), "{ev:?}");
    assert!(ev.iter().any(|e| matches!(e, Event::RtPriority { .. })), "{ev:?}");
    drv.close().unwrap();
    assert_eq!(sim.wait_exit(Duration::from_secs(2)), Some(SimExit::Stopped));
    assert!(matches!(drv.hold(), Err(Error::Closed)));
}

#[test]
fn wait_for_robot_times_out_without_a_robot() {
    let drv = driver(500);
    let t0 = Instant::now();
    assert!(matches!(drv.wait_for_robot(Duration::from_millis(200)), Err(Error::RobotTimeout)));
    assert!(t0.elapsed() >= Duration::from_millis(200));
    assert!(matches!(drv.stream(&[[0.0; 6]], Duration::from_millis(100), false), Err(Error::Protocol(_))));
}

#[test]
fn stream_tracks_a_sine_and_stays_on_time() {
    let (drv, sim) = connected();
    let q0 = drv.state().q;
    let mut last = q0;
    let dt = Duration::from_millis(100);
    for chunk in 0..15u32 {
        let points: Vec<[f64; 6]> = (0..5)
            .map(|k| {
                let t = (chunk * 5 + k + 1) as f64 * 0.1;
                let mut q = q0;
                q[0] += 0.05 * (t * 2.0).sin();
                q
            })
            .collect();
        last = points[4];
        drv.stream(&points, dt, false).unwrap();
        std::thread::sleep(Duration::from_millis(500));
    }
    drv.wait_idle(Duration::from_secs(5)).unwrap();
    let st = settled(&drv, &sim);
    assert!((st.q_setpoint_deg[0] - last[0].to_degrees()).abs() < 0.01, "{st:?} vs {}", last[0].to_degrees());
    let s = drv.stats();
    assert!(s.replies > 200, "{s:?}");
    assert!(st.late as u64 * 20 < st.cycles, "late replies: {st:?}");
    assert_eq!(drv.session(), SessionState::Active);
}

#[test]
fn empty_queue_holds_and_counts_starvation() {
    let (drv, sim) = connected();
    let mut q = drv.state().q;
    q[0] += 0.01;
    drv.stream(&[q], Duration::from_millis(40), false).unwrap();
    drv.wait_idle(Duration::from_secs(2)).unwrap();
    std::thread::sleep(Duration::from_millis(200));
    let st = drv.state();
    assert!((st.q_command[0] - q[0]).abs() < 1e-9, "{st:?}");
    assert!(drv.stats().starved_cycles > 20);
    assert!((sim.status().q_setpoint_deg[0] - q[0].to_degrees()).abs() < 1e-6);
}

#[test]
fn link_lost_then_reconnect_recaptures_start() {
    let drv = driver(300);
    let mut sim = ControllerSim::spawn(sim_config(&drv)).unwrap();
    drv.wait_for_robot(Duration::from_secs(3)).unwrap();
    sim.stop().unwrap();
    assert!(wait_until(Duration::from_secs(2), || drv.session() == SessionState::Lost));
    std::thread::sleep(Duration::from_millis(120));
    assert!(drv.events().contains(&Event::LinkLost));
    assert!(matches!(drv.stream(&[[0.0; 6]], Duration::from_millis(100), false), Err(Error::LinkLost(_))));
    let mut c = sim_config(&drv);
    c.q_start_deg[1] = -80.0;
    let _sim2 = ControllerSim::spawn(c).unwrap();
    drv.wait_for_robot(Duration::from_secs(3)).unwrap();
    let st = drv.state();
    assert!((st.q_setpoint[1] + 80f64.to_radians()).abs() < 1e-6, "{st:?}");
    assert_eq!(drv.stats().sessions, 2);
}

#[test]
fn stream_rejects_bad_targets_and_durations() {
    let (drv, _sim) = connected();
    let mut q = drv.state().q;
    q[0] = 4.0;
    assert!(matches!(drv.stream(&[q], Duration::from_millis(100), false), Err(Error::Limit(_))));
    q[0] = 0.0;
    assert!(matches!(drv.stream(&[q], Duration::ZERO, false), Err(Error::Limit(_))));
    assert!(matches!(drv.move_to(q, Duration::ZERO), Err(Error::Limit(_))));
    assert!(matches!(drv.set_speed_scale(2.0), Err(Error::Limit(_))));
    let mut bad = Limits::kr10_defaults();
    bad.max_velocity[0] = -1.0;
    assert!(matches!(drv.set_limits(bad), Err(Error::Config(_))));
}

#[test]
fn fault_stops_the_robot() {
    let (drv, sim) = connected();
    drv.fault("policy diverged").unwrap();
    assert_eq!(sim.wait_exit(Duration::from_secs(2)), Some(SimExit::EStrError));
    std::thread::sleep(Duration::from_millis(120));
    assert!(drv.events().contains(&Event::Fault));
}

#[test]
fn disturbed_link_stays_active() {
    let drv = driver(500);
    let mut c = sim_config(&drv);
    c.jitter = Duration::from_micros(500);
    c.drop_probability = 0.05;
    let sim = ControllerSim::spawn(c).unwrap();
    drv.wait_for_robot(Duration::from_secs(3)).unwrap();
    let q0 = drv.state().q;
    for k in 0..10 {
        let mut q = q0;
        q[5] += 0.02 * (k as f64 * 0.5).sin();
        drv.stream(&[q], Duration::from_millis(100), false).unwrap();
        std::thread::sleep(Duration::from_millis(100));
    }
    assert_eq!(drv.session(), SessionState::Active);
    let s = drv.stats();
    assert!(s.ipoc_gaps > 0, "drops must show up as IPOC gaps: {s:?}");
    assert!(sim.status().late < 100);
}

#[test]
fn log_yields_every_cycle() {
    let (drv, _sim) = connected();
    let mut v = Vec::new();
    drv.drain_log(&mut v);
    std::thread::sleep(Duration::from_millis(400));
    let n = drv.drain_log(&mut v);
    assert!((60..=140).contains(&n), "got {n} entries in 400 ms");
    assert!(v.windows(2).all(|w| w[1].cycle == w[0].cycle + 1), "contiguous cycles");
}

#[test]
fn move_to_reaches_the_target_smoothly() {
    let (drv, sim) = connected();
    let mut q = drv.state().q;
    q[5] += 0.03;
    drv.move_to(q, Duration::from_millis(400)).unwrap();
    std::thread::sleep(Duration::from_millis(40));
    let early = drv.state().q_command[5] - (q[5] - 0.03);
    assert!(early < 0.003, "quintic start is slow: {early}");
    drv.wait_idle(Duration::from_secs(2)).unwrap();
    assert!((settled(&drv, &sim).q_setpoint_deg[5] - q[5].to_degrees()).abs() < 1e-6);
}
