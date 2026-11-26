use std::time::{Duration, Instant};

use kuka_rsi::Error;
use kuka_rsi::config::{DriverConfig, EthernetConfig, Limits};
use kuka_rsi::driver::Driver;
use kuka_rsi::sim::{ControllerSim, SimConfig, SimExit, SimHandle, SimStatus};
use kuka_rsi::state::{RobotState, SessionEnd, SessionState};
use kuka_rsi::stats::Event;

const SHIPPED: &str = include_str!("../controller/kuka_rsi_ethernet.xml");

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

/// The events recorded since the last call, without their times.
fn events(drv: &Driver) -> Vec<Event> {
    drv.events().into_iter().map(|e| e.event).collect()
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
    for ((q, s), ak) in st.q.iter().zip(start).zip(&st.ak) {
        assert!((q - s.to_radians()).abs() < 1e-6, "{st:?}");
        assert!(ak.abs() < 1e-9);
    }
    assert_eq!(drv.stats().sessions, 1);
    std::thread::sleep(Duration::from_millis(120)); // events reach the history via the monitor thread (Task 15)
    let ev = events(&drv);
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
        // Push the next chunk when the previous one is nearly played, like a policy would.
        // Playback runs on the simulator's cycles, which a CI VM can stretch to 15 ms.
        assert!(wait_until(Duration::from_secs(10), || drv.stats().queue_len <= 5), "{:?}", drv.stats());
        drv.stream(&points, dt, false).unwrap();
    }
    // 7.5 s of path is 1875 cycles; stretched to 15 ms each they take 28 s.
    drv.wait_idle(Duration::from_secs(120)).unwrap();
    let st = settled(&drv, &sim);
    assert!((st.q_setpoint_deg[0] - last[0].to_degrees()).abs() < 0.01, "{st:?} vs {}", last[0].to_degrees());
    let s = drv.stats();
    assert!(s.replies > 200, "{s:?}");
    assert!(s.ipoc_gaps <= s.skipped_datagrams, "IPOC steps of one cycle are not gaps: {s:?}");
    // The simulator's late count is about the driver only while the simulator keeps its own
    // schedule: a late wake-up of its loop shortens the next reply window.
    if st.overruns * 10 <= st.cycles {
        assert!(st.late as u64 * 20 < st.cycles, "late replies: {st:?}");
    }
    assert_eq!(drv.session(), SessionState::Active);
}

#[test]
fn empty_queue_holds_and_counts_starvation() {
    let (drv, sim) = connected();
    let mut q = drv.state().q;
    q[0] += 0.01;
    drv.stream(&[q], Duration::from_millis(40), false).unwrap();
    drv.wait_idle(Duration::from_secs(10)).unwrap();
    assert!(wait_until(Duration::from_secs(5), || drv.stats().starved_cycles > 20), "{:?}", drv.stats());
    let st = drv.state();
    assert!((st.q_command[0] - q[0]).abs() < 1e-9, "{st:?}");
    assert!((sim.status().q_setpoint_deg[0] - q[0].to_degrees()).abs() < 1e-6);
}

#[test]
fn link_lost_then_reconnect_recaptures_start() {
    let drv = driver(300);
    let mut sim = ControllerSim::spawn(sim_config(&drv)).unwrap();
    drv.wait_for_robot(Duration::from_secs(3)).unwrap();
    sim.stop().unwrap();
    assert_eq!(drv.last_session_end(), None, "no session has ended yet");
    assert!(wait_until(Duration::from_secs(2), || drv.session() == SessionState::Lost));
    assert_eq!(drv.last_session_end(), Some(SessionEnd::LinkTimeout), "the robot went silent unasked");
    std::thread::sleep(Duration::from_millis(120));
    assert!(events(&drv).contains(&Event::SessionEnded(SessionEnd::LinkTimeout)));
    assert!(matches!(drv.stream(&[[0.0; 6]], Duration::from_millis(100), false), Err(Error::LinkLost(_))));
    let mut c = sim_config(&drv);
    c.q_start_deg[1] = -80.0;
    let _sim2 = ControllerSim::spawn(c).unwrap();
    drv.wait_for_robot(Duration::from_secs(3)).unwrap();
    assert_eq!(drv.last_session_end(), Some(SessionEnd::LinkTimeout), "kept while the next session runs");
    let st = drv.state();
    assert!((st.q_setpoint[1] + 80f64.to_radians()).abs() < 1e-6, "{st:?}");
    let s = drv.stats();
    assert_eq!(s.sessions, 2);
    assert!(s.interarrival_max_us < 200_000, "the gap between sessions is not an inter-arrival time: {s:?}");
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
    assert!(events(&drv).contains(&Event::Fault));
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
    // Every cycle, none twice: 100 at the 4 ms cycle. No tight lower bound: on CI VMs the
    // simulator's millisecond sleeps can stretch its cycle to 15 ms.
    assert!((20..=140).contains(&n), "got {n} entries in 400 ms");
    assert_eq!(drv.stats().log_drops, 0);
    assert!(v.windows(2).all(|w| w[1].cycle == w[0].cycle + 1), "contiguous cycles");
}

#[test]
fn move_to_reaches_the_target_smoothly() {
    let (drv, sim) = connected();
    let mut q = drv.state().q;
    q[5] += 0.03;
    drv.move_to(q, Duration::from_millis(400)).unwrap();
    std::thread::sleep(Duration::from_millis(12));
    let early = drv.state().q_command[5] - (q[5] - 0.03);
    assert!(early < 0.005, "quintic start is slow: {early}");
    drv.wait_idle(Duration::from_secs(10)).unwrap();
    assert!((settled(&drv, &sim).q_setpoint_deg[5] - q[5].to_degrees()).abs() < 1e-6);
}

#[test]
fn stream_with_non_integer_cycle_spacing_keeps_rate() {
    let (drv, sim) = connected();
    let start = drv.state().q_command;
    // 60 points, so rounding 1.5 cycles to 2 (120 cycles) or 1 (60) is far outside the tolerance.
    let points: Vec<[f64; 6]> = (0..60)
        .map(|k| {
            let mut q = start;
            q[0] += 0.0005 * (k + 1) as f64;
            q
        })
        .collect();
    let last = points[59];
    drv.stream(&points, Duration::from_millis(6), false).unwrap(); // 1.5 cycles per point
    drv.wait_idle(Duration::from_secs(10)).unwrap();
    // Played cycles from the log: from the first cycle off the start pose to the first at the last target.
    let mut log = Vec::new();
    drv.drain_log(&mut log);
    let moved = log.iter().find(|s| (s.q_command[0] - start[0]).abs() > 1e-9).expect("the command never moved");
    let reached = log
        .iter()
        .find(|s| (s.q_command[0] - last[0]).abs() < 1e-9)
        .expect("the command never reached the last target");
    let played = reached.cycle - moved.cycle + 1;
    assert!((88..=92).contains(&played), "60 points x 1.5 cycles should play in 90 cycles, took {played}");
    assert!((settled(&drv, &sim).q_setpoint_deg[0] - last[0].to_degrees()).abs() < 1e-6);
}

#[test]
fn stream_rejects_calls_that_cannot_fit() {
    let (drv, _sim) = connected();
    let q = drv.state().q;
    let too_many = vec![q; kuka_rsi::control::MAX_QUEUE + 1];
    assert!(matches!(drv.stream(&too_many, Duration::from_millis(100), false), Err(Error::Limit(_))));
    drv.stream(&vec![q; 1000], Duration::from_millis(100), false).unwrap();
    assert!(wait_until(Duration::from_secs(1), || drv.stats().queue_len >= 990));
    assert!(matches!(drv.stream(&vec![q; 100], Duration::from_millis(100), false), Err(Error::QueueFull)));
    drv.stream(&vec![q; 100], Duration::from_millis(100), true).unwrap();
    assert!(wait_until(Duration::from_secs(1), || drv.stats().queue_len <= 100));
}

#[test]
fn wait_idle_waits_for_the_limiter() {
    let (drv, _sim) = connected();
    let mut q = drv.state().q;
    q[0] += 0.2; // one cycle of playback, then 25 cycles at the 2 rad/s velocity limit
    drv.stream(&[q], Duration::from_millis(4), false).unwrap();
    drv.wait_idle(Duration::from_secs(10)).unwrap();
    let st = drv.state();
    assert!((st.q_command[0] - q[0]).abs() < 1e-9, "wait_idle returned while the command was still moving: {st:?}");
}

/// A driver without a stop channel: `close` stops the thread without waiting for the session to end.
fn driver_without_stop_channel() -> Driver {
    let cfg = DriverConfig::new(EthernetConfig::from_str(SHIPPED).unwrap())
        .bind("127.0.0.1:0".parse().unwrap())
        .limits(Limits::kr10_defaults().with_max_velocity_all(2.0).with_max_acceleration_all(None))
        .link_timeout(Duration::from_millis(500))
        .rt_priority(false);
    Driver::spawn(cfg).unwrap()
}

#[test]
fn session_is_lost_after_close() {
    let drv = driver_without_stop_channel();
    let _sim = ControllerSim::spawn(sim_config(&drv)).unwrap();
    drv.wait_for_robot(Duration::from_secs(3)).unwrap();
    drv.close().unwrap();
    assert_eq!(drv.session(), SessionState::Lost, "nobody answers the robot after close");
    assert_eq!(drv.last_session_end(), Some(SessionEnd::Closed));
    assert!(events(&drv).contains(&Event::SessionEnded(SessionEnd::Closed)));
}

#[test]
fn concurrent_close_waits_for_the_join() {
    let drv = driver_without_stop_channel();
    let mut sim = ControllerSim::spawn(sim_config(&drv)).unwrap();
    drv.wait_for_robot(Duration::from_secs(3)).unwrap();
    // Without telegrams the real-time thread sits in its 50 ms receive wait, so the first close's
    // join takes a while, and the session stays `Active` (500 ms link timeout) until the thread exits.
    sim.stop().unwrap();
    std::thread::scope(|s| {
        let first = s.spawn(|| drv.close());
        std::thread::sleep(Duration::from_millis(10));
        drv.close().unwrap();
        assert_eq!(drv.session(), SessionState::Lost, "second close returned before the real-time thread stopped");
        first.join().unwrap().unwrap();
    });
}

#[test]
fn ipoc_gaps_count_missing_telegrams_only() {
    let drv = driver(500);
    let mut c = sim_config(&drv);
    // This seed drops the session's second telegram (a step learned from the first two telegrams
    // would be two cycles, and every later telegram would count as a gap), then about 3 % of the rest.
    c.drop_probability = 0.05;
    c.seed = 0x8d11_6ece_1738_f7d9;
    let _sim = ControllerSim::spawn(c).unwrap();
    drv.wait_for_robot(Duration::from_secs(3)).unwrap();
    std::thread::sleep(Duration::from_millis(400));
    let s = drv.stats();
    assert!(s.ipoc_gaps >= 1, "the dropped second telegram is a gap: {s:?}");
    assert!(s.ipoc_gaps * 5 < s.cycles, "only missing telegrams are gaps: {s:?}");
    assert!(!drv.is_faulted(), "missing telegrams are not a cycle mismatch");
}

/// A driver with a `cycle_ms` cycle against a simulated robot that sends every `robot_ms`.
fn with_cycles(cycle_ms: u64, robot_ms: u64) -> (Driver, SimHandle) {
    let cfg = DriverConfig::new(EthernetConfig::from_str(SHIPPED).unwrap())
        .bind("127.0.0.1:0".parse().unwrap())
        .stop_channel("MoveStop")
        .cycle(Duration::from_millis(cycle_ms))
        .rt_priority(false);
    let drv = Driver::spawn(cfg).unwrap();
    let mut c = sim_config(&drv);
    c.cycle = Duration::from_millis(robot_ms);
    let sim = ControllerSim::spawn(c).unwrap();
    drv.wait_for_robot(Duration::from_secs(3)).unwrap();
    (drv, sim)
}

#[test]
fn a_12_ms_cycle_follows_a_12_ms_robot() {
    let (drv, sim) = with_cycles(12, 12);
    let mut q = drv.state().q_setpoint;
    q[5] += 0.05;
    // Counted in the robot's cycles, not in wall time: a CI VM can stretch the simulator's cycle.
    let c0 = drv.state().cycle;
    drv.move_to(q, Duration::from_millis(600)).unwrap();
    drv.wait_idle(Duration::from_secs(30)).unwrap();
    let cycles = drv.state().cycle - c0;
    assert!((50..=60).contains(&cycles), "0.6 s is 50 cycles of 12 ms, the move took {cycles}");
    let st = settled(&drv, &sim);
    assert!((st.q_setpoint_deg[5].to_radians() - q[5]).abs() < 1e-6, "{st:?}");
    let s = drv.stats();
    assert!(!drv.is_faulted());
    assert!(s.ipoc_gaps <= s.skipped_datagrams, "IPOC steps of 12 ms are not gaps: {s:?}");
}

#[test]
fn a_cycle_other_than_the_robots_faults_the_session() {
    // The dangerous direction first: the PC plans for 12 ms steps, the robot applies one every
    // 4 ms, so every move would run three times too fast.
    for (cycle_ms, robot_ms) in [(12, 4), (4, 12)] {
        let (drv, sim) = with_cycles(cycle_ms, robot_ms);
        assert!(wait_until(Duration::from_secs(1), || drv.is_faulted()), "{cycle_ms} ms against {robot_ms} ms");
        let q = drv.state().q_setpoint;
        match drv.move_to(q, Duration::from_secs(1)) {
            Err(Error::CycleMismatch { robot_ms: r, configured_ms: c }) => assert_eq!((r, c), (robot_ms, cycle_ms)),
            other => panic!("{other:?}"),
        }
        assert_eq!(drv.cycle_mismatch(), Some(Duration::from_millis(robot_ms)));
        // The robot is told why: the fault message stops it.
        assert_eq!(sim.wait_exit(Duration::from_secs(2)), Some(SimExit::EStrError), "{cycle_ms}/{robot_ms}");
        std::thread::sleep(Duration::from_millis(120)); // events reach the history via the monitor thread
        let ev = events(&drv);
        let want = Event::CycleMismatch { robot_ms, configured_ms: cycle_ms };
        assert!(ev.contains(&want), "{ev:?}");
    }
}

#[test]
fn event_history_survives_until_read() {
    let (drv, mut sim) = connected();
    std::thread::sleep(Duration::from_millis(200));
    sim.stop().unwrap();
    assert!(wait_until(Duration::from_secs(2), || drv.session() == SessionState::Lost));
    std::thread::sleep(Duration::from_millis(120));
    let ev = events(&drv);
    assert!(ev.iter().any(|e| matches!(e, Event::RtPriority { .. })), "{ev:?}");
    assert!(ev.iter().any(|e| matches!(e, Event::RobotConnected { .. })), "{ev:?}");
    assert!(ev.contains(&Event::SessionEnded(SessionEnd::LinkTimeout)), "{ev:?}");
    assert!(events(&drv).is_empty(), "drained");
}

/// The first log index at which joint `axis` of the command is off `from`.
fn first_move(log: &[RobotState], axis: usize, from: f64) -> Option<usize> {
    log.iter().position(|s| (s.q_command[axis] - from).abs() > 1e-9)
}

/// Two ramps, `(axis, final value)`, each on its own axis from `q0`, were played one after the
/// other: the axis that moved first reached its final value before the other axis moved.
fn assert_ramps_in_sequence(log: &[RobotState], q0: [f64; 6], ramps: [(usize, f64); 2]) {
    let starts =
        ramps.map(|(axis, _)| first_move(log, axis, q0[axis]).unwrap_or_else(|| panic!("A{} never moved", axis + 1)));
    let (first, second) = if starts[0] < starts[1] { (0, 1) } else { (1, 0) };
    let (axis, end) = ramps[first];
    let done = log[..starts[second]].iter().any(|s| (s.q_command[axis] - end).abs() < 1e-9);
    assert!(
        done,
        "A{} started moving at log index {} before the A{} ramp (from index {}) reached its end: the calls' chunks interleaved",
        ramps[second].0 + 1,
        starts[second],
        axis + 1,
        starts[first]
    );
}

/// 100 points ramping joint `axis` from `q0` in steps of 1e-4 rad.
fn ramp(q0: [f64; 6], axis: usize) -> Vec<[f64; 6]> {
    (1..=100)
        .map(|k| {
            let mut q = q0;
            q[axis] = q0[axis] + k as f64 * 1e-4;
            q
        })
        .collect()
}

#[test]
fn concurrent_streams_do_not_interleave() {
    let (drv, _sim) = connected();
    let q0 = drv.state().q_command;
    let (a, b) = (ramp(q0, 5), ramp(q0, 1));
    let ramps = [(5, a[99][5]), (1, b[99][1])];
    let dt = Duration::from_millis(8);
    let barrier = std::sync::Barrier::new(2);
    std::thread::scope(|s| {
        for points in [&a, &b] {
            let (drv, barrier) = (&drv, &barrier);
            s.spawn(move || {
                barrier.wait();
                drv.stream(points, dt, false).unwrap();
            });
        }
    });
    // 200 points 8 ms apart are 400 cycles: 1.6 s, or 6 s when a CI VM stretches them to 15 ms.
    drv.wait_idle(Duration::from_secs(30)).unwrap();
    assert_eq!(drv.stats().queue_len, 0);
    std::thread::sleep(Duration::from_millis(120));
    let ev = events(&drv);
    assert!(!ev.contains(&Event::CommandRejected), "{ev:?}");
    let mut log = Vec::new();
    drv.drain_log(&mut log);
    assert_ramps_in_sequence(&log, q0, ramps);
}

#[test]
fn back_to_back_streams_that_cannot_both_fit_are_rejected() {
    let (drv, _sim) = connected();
    let q = drv.state().q_command;
    let dt = Duration::from_millis(100);
    drv.stream(&vec![q; 600], dt, false).unwrap();
    // The first call may still be in the command queue, not yet counted in `queue_len`.
    assert!(matches!(drv.stream(&vec![q; 600], dt, false), Err(Error::QueueFull)));
    // Replacing drops whatever is ahead of the call, in flight or not.
    drv.stream(&vec![q; 600], dt, true).unwrap();
    std::thread::sleep(Duration::from_millis(120));
    let ev = events(&drv);
    assert!(!ev.contains(&Event::CommandRejected), "{ev:?}");
}

#[test]
fn first_telegram_without_axis_positions_does_not_start_a_session() {
    let drv = driver(500);
    let robot = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
    robot.set_read_timeout(Some(Duration::from_millis(150))).unwrap();
    let mut buf = [0u8; 2048];
    // No ASPos: `q_start` would be stale, so the telegram is counted and not answered.
    let no_aspos = br#"<Rob Type="KUKA"><AIPos A1="1.0" A2="-90.0" A3="90.0" A4="0.0" A5="90.0" A6="0.0" /><Delay D="0" /><IPOC>4000</IPOC></Rob>"#;
    robot.send_to(no_aspos, drv.local_addr()).unwrap();
    assert!(robot.recv_from(&mut buf).is_err(), "a telegram without ASPos was answered");
    assert_eq!(drv.session(), SessionState::Listening);
    assert_eq!(drv.stats().parse_errors, 1);
    assert_eq!(drv.stats().sessions, 0);

    let full = br#"<Rob Type="KUKA"><AIPos A1="1.0" A2="-90.0" A3="90.0" A4="0.0" A5="90.0" A6="0.0" /><ASPos A1="1.0" A2="-90.0" A3="90.0" A4="0.0" A5="90.0" A6="0.0" /><Delay D="0" /><IPOC>4004</IPOC></Rob>"#;
    robot.send_to(full, drv.local_addr()).unwrap();
    let (n, _) = robot.recv_from(&mut buf).expect("a telegram with both positions starts the session");
    let reply = std::str::from_utf8(&buf[..n]).unwrap();
    assert!(reply.contains("<IPOC>4004</IPOC>"), "{reply}");
    // The reply leaves before the cycle's state is published.
    drv.wait_for_robot(Duration::from_secs(1)).unwrap();
    assert_eq!(drv.stats().sessions, 1);
    drv.close().unwrap();
}

#[test]
fn fault_is_visible_and_refuses_motion_until_the_next_session() {
    // No stop channel, and a link timeout long enough to inspect the faulted session after the
    // simulator stops on `EStr "Error: ..."`.
    let cfg = DriverConfig::new(EthernetConfig::from_str(SHIPPED).unwrap())
        .bind("127.0.0.1:0".parse().unwrap())
        .limits(Limits::kr10_defaults().with_max_velocity_all(2.0).with_max_acceleration_all(None))
        .link_timeout(Duration::from_secs(1))
        .rt_priority(false);
    let drv = Driver::spawn(cfg).unwrap();
    let sim = ControllerSim::spawn(sim_config(&drv)).unwrap();
    drv.wait_for_robot(Duration::from_secs(3)).unwrap();
    assert!(!drv.is_faulted());
    let q = drv.state().q_command;

    drv.fault("policy diverged").unwrap();
    assert!(wait_until(Duration::from_secs(1), || drv.is_faulted()), "the fault was never published");
    assert_eq!(drv.session(), SessionState::Active, "a fault is a state within Active");
    assert!(matches!(drv.stream(&[q], Duration::from_millis(100), false), Err(Error::Faulted)));
    assert!(matches!(drv.move_to(q, Duration::from_millis(100)), Err(Error::Faulted)));
    assert!(matches!(drv.notify("hello"), Err(Error::Faulted)));
    assert!(matches!(drv.wait_idle(Duration::from_millis(100)), Err(Error::Faulted)));
    assert_eq!(sim.wait_exit(Duration::from_secs(2)), Some(SimExit::EStrError));

    // Restarting the program is a new session: the fault is gone.
    assert!(wait_until(Duration::from_secs(3), || drv.session() == SessionState::Lost));
    assert_eq!(drv.last_session_end(), Some(SessionEnd::Fault));
    let _sim = ControllerSim::spawn(sim_config(&drv)).unwrap();
    drv.wait_for_robot(Duration::from_secs(3)).unwrap();
    assert!(!drv.is_faulted());
    drv.stream(&[drv.state().q_command], Duration::from_millis(100), false).unwrap();
    drv.wait_idle(Duration::from_secs(10)).unwrap();
    drv.close().unwrap();
}

#[test]
fn limits_are_the_live_value_and_config_keeps_the_constructor_value() {
    let drv = driver_without_stop_channel();
    let initial = drv.config().limits;
    assert_eq!(drv.limits(), initial);
    let slower = initial.with_max_velocity_all(0.5);
    drv.set_limits(slower).unwrap();
    assert_eq!(drv.limits(), slower, "set_limits changes the live limits");
    assert_eq!(drv.config().limits, initial, "config() keeps the constructor value");
    drv.close().unwrap();
}

#[test]
fn robot_addr_is_the_telegram_source_during_a_session() {
    let drv = driver(150);
    assert_eq!(drv.robot_addr(), None);
    let mut sim = ControllerSim::spawn(sim_config(&drv)).unwrap();
    drv.wait_for_robot(Duration::from_secs(3)).unwrap();
    assert_eq!(drv.robot_addr(), Some(sim.local_addr()));
    sim.stop().unwrap();
    assert!(wait_until(Duration::from_secs(2), || drv.session() == SessionState::Lost), "{:?}", drv.session());
    assert_eq!(drv.robot_addr(), None);
}

/// Tell the simulator to report the program as running or stopped, and wait until the driver sees it.
fn set_program_running(drv: &Driver, sim: &SimHandle, running: bool) {
    sim.set_program_running(running);
    assert!(wait_until(Duration::from_secs(1), || drv.state().program_running == running), "{:?}", drv.state());
}

#[test]
fn wait_idle_outlasts_a_stopped_program() {
    let (drv, sim) = connected();
    let mut q = drv.state().q;
    q[5] += 0.03;
    set_program_running(&drv, &sim, false);
    drv.move_to(q, Duration::from_millis(200)).unwrap();
    std::thread::scope(|s| {
        // 600 ms are plenty for a 200 ms move, and the program stays stopped for longer than that.
        let wait = s.spawn(|| drv.wait_idle(Duration::from_millis(600)));
        std::thread::sleep(Duration::from_millis(900));
        let ended_early = wait.is_finished();
        sim.set_program_running(true);
        let result = wait.join().unwrap();
        assert!(!ended_early, "the wait ended while the program was stopped: {result:?}");
        result.unwrap();
    });
    assert!((drv.state().q_command[5] - q[5]).abs() < 1e-9, "{:?}", drv.state());
}

/// A driver with the default limits and a simulator that stops like the KR 10 did (2026-11-26):
/// it stands 9 cycles before `ProgStatus` says so, its drives need 30 cycles after a resume, and
/// a setpoint step of more than 0.3 deg in a cycle ends RSI ("command velocity"). The robot
/// stopped at 0.29 deg; the test's move steps by 0.02 deg a cycle, so a simulator that a busy
/// host makes miss a dozen replies in a row still stays below it.
fn connected_to_a_robot_that_stands() -> (Driver, SimHandle) {
    let cfg = DriverConfig::new(EthernetConfig::from_str(SHIPPED).unwrap())
        .bind("127.0.0.1:0".parse().unwrap())
        .stop_channel("MoveStop")
        .rt_priority(false);
    let drv = Driver::spawn(cfg).unwrap();
    let mut c = sim_config(&drv);
    c.stop_report_cycles = 9;
    c.drives_on_cycles = 30;
    c.max_step_deg = Some(0.3);
    let sim = ControllerSim::spawn(c).unwrap();
    drv.wait_for_robot(Duration::from_secs(3)).unwrap();
    (drv, sim)
}

#[test]
fn a_move_survives_a_stop_and_a_resume_of_the_program() {
    let (drv, sim) = connected_to_a_robot_that_stands();
    let mut q = drv.state().q;
    q[5] += 12f64.to_radians();
    // Peak 5.6 deg/s: the command runs 0.2 deg ahead of the robot before the stop is reported.
    let motion = drv.move_to(q, Duration::from_secs(4)).unwrap();
    std::thread::sleep(Duration::from_millis(2000));
    sim.set_program_running(false);
    assert!(wait_until(Duration::from_secs(1), || drv.is_paused()), "the stop was never reported");
    std::thread::sleep(Duration::from_millis(300));
    let st = drv.state();
    assert!((st.q_command[5] - st.q[5]).abs() < 1e-5, "the command stands where the robot does: {st:?}");
    sim.set_program_running(true);
    drv.wait(Duration::from_secs(6)).motion(motion).run().unwrap();
    assert_eq!(sim.status().exit, None, "the robot stopped RSI");
    // The move went on from where the robot stood: nowhere faster than the move itself. The
    // command is what counts: the simulated robot takes two steps in one cycle whenever a busy
    // host makes it miss a reply. The step back to the robot comes with the reported stop.
    let mut log = Vec::new();
    drv.drain_log(&mut log);
    let fastest = log
        .windows(2)
        .filter(|w| w[0].program_running && w[1].program_running)
        .map(|w| (w[1].q_command[5] - w[0].q_command[5]).abs())
        .fold(0.0, f64::max);
    let peak = 12f64.to_radians() * 1.875 / 1000.0;
    assert!(fastest <= peak * 1.02, "a step of {fastest} rad in a cycle, the move peaks at {peak}");
    assert!((drv.state().q_command[5] - q[5]).abs() < 1e-9, "{:?}", drv.state());
    assert!((settled(&drv, &sim).q_actual_deg[5] - q[5].to_degrees()).abs() < 1e-3);
}

#[test]
fn wait_idle_counts_playback_time_under_a_low_override() {
    let (drv, sim) = connected();
    sim.set_override_percent(10.0);
    assert!(wait_until(Duration::from_secs(1), || drv.state().speed_override == 0.1), "{:?}", drv.state());
    let mut q = drv.state().q;
    q[5] += 0.03;
    // 50 cycles of path at 10 % take 500 cycles: 2 s, well past the 800 ms the wait allows.
    let c0 = drv.state().cycle;
    drv.move_to(q, Duration::from_millis(200)).unwrap();
    drv.wait_idle(Duration::from_millis(800)).unwrap();
    let cycles = drv.state().cycle - c0;
    assert!(cycles >= 480, "the override did not slow the move: {cycles} cycles");
    assert!((drv.state().q_command[5] - q[5]).abs() < 1e-9, "{:?}", drv.state());
}

#[test]
fn a_pause_longer_than_the_pause_timeout_is_reported_as_paused() {
    let (drv, sim) = connected();
    let mut q = drv.state().q;
    q[5] += 0.03;
    assert!(!drv.is_paused());
    set_program_running(&drv, &sim, false);
    assert!(wait_until(Duration::from_secs(1), || drv.is_paused()));
    drv.move_to(q, Duration::from_millis(200)).unwrap();
    let t0 = Instant::now();
    let r = drv.wait(Duration::from_secs(5)).pause_timeout(Duration::from_millis(300)).run();
    assert!(matches!(r, Err(Error::Paused(_))), "{r:?}");
    assert!(t0.elapsed() >= Duration::from_millis(300), "{:?}", t0.elapsed());
    // The move is still queued and finishes once the program runs again.
    set_program_running(&drv, &sim, true);
    drv.wait(Duration::from_secs(5)).pause_timeout(Duration::from_secs(5)).run().unwrap();
    assert!(!drv.is_paused());
    assert!((drv.state().q_command[5] - q[5]).abs() < 1e-9, "{:?}", drv.state());
}

#[test]
fn a_wait_shorter_than_the_move_times_out_while_playback_runs() {
    let (drv, _sim) = connected();
    let mut q = drv.state().q;
    q[5] += 0.03;
    drv.move_to(q, Duration::from_secs(2)).unwrap();
    let r = drv.wait(Duration::from_millis(100)).pause_timeout(Duration::from_secs(5)).run();
    assert!(matches!(r, Err(Error::IdleTimeout)), "{r:?}");
}

#[test]
fn is_idle_follows_the_motion() {
    let (drv, _sim) = connected();
    assert!(drv.is_idle().unwrap());
    let mut q = drv.state().q;
    q[5] += 0.03;
    let motion = drv.move_to(q, Duration::from_millis(300)).unwrap();
    assert!(!drv.is_idle().unwrap(), "a queued move is not idle, applied or not");
    drv.wait(Duration::from_secs(10)).motion(motion).run().unwrap();
    assert!(drv.is_idle().unwrap());
}

#[test]
fn is_idle_without_a_session_is_an_error() {
    let drv = driver(500);
    assert!(matches!(drv.is_idle(), Err(Error::Protocol(_))));
    assert!(!drv.is_paused());
}

#[test]
fn a_hold_interrupts_the_wait_for_a_motion() {
    let (drv, _sim) = connected();
    let mut q = drv.state().q;
    q[5] += 0.05;
    let motion = drv.move_to(q, Duration::from_secs(2)).unwrap();
    std::thread::scope(|s| {
        let wait = s.spawn(|| drv.wait(Duration::from_secs(10)).motion(motion).run());
        std::thread::sleep(Duration::from_millis(100));
        drv.hold().unwrap();
        let r = wait.join().unwrap();
        assert!(matches!(r, Err(Error::Interrupted)), "{r:?}");
    });
    // The hold itself settles, and a wait that names no motion has nothing to report.
    drv.wait_idle(Duration::from_secs(5)).unwrap();
    assert!(drv.state().q_command[5] < q[5] - 0.01, "the move was cut short: {:?}", drv.state());
    // Still interrupted when asked again later.
    assert!(matches!(drv.wait(Duration::from_secs(1)).motion(motion).run(), Err(Error::Interrupted)));
}

#[test]
fn a_new_motion_interrupts_the_wait_for_the_one_it_replaces() {
    let (drv, _sim) = connected();
    let q0 = drv.state().q;
    let mut q1 = q0;
    q1[5] += 0.05;
    let first = drv.move_to(q1, Duration::from_secs(2)).unwrap();
    let second = drv.move_to(q0, Duration::from_millis(200)).unwrap();
    assert!(matches!(drv.wait(Duration::from_secs(10)).motion(first).run(), Err(Error::Interrupted)));
    drv.wait(Duration::from_secs(10)).motion(second).run().unwrap();

    let third = drv.stream(&[q1], Duration::from_secs(2), false).unwrap();
    let fourth = drv.stream(&[q0], Duration::from_millis(200), true).unwrap();
    assert!(matches!(drv.wait(Duration::from_secs(10)).motion(third).run(), Err(Error::Interrupted)));
    drv.wait(Duration::from_secs(10)).motion(fourth).run().unwrap();
}

#[test]
fn appended_points_extend_the_motion() {
    let (drv, _sim) = connected();
    let q0 = drv.state().q;
    let (mut a, mut b) = (q0, q0);
    a[5] += 0.01;
    b[5] += 0.02;
    let first = drv.stream(&[a], Duration::from_millis(100), false).unwrap();
    let second = drv.stream(&[b], Duration::from_millis(100), false).unwrap();
    assert_eq!(first, second, "points appended to the queue are part of the same motion");
    drv.wait(Duration::from_secs(10)).motion(first).run().unwrap();
    assert!((drv.state().q_command[5] - b[5]).abs() < 1e-9, "{:?}", drv.state());
}

#[test]
fn a_motion_of_an_ended_session_is_reported_as_link_lost() {
    let drv = driver(300);
    let mut sim = ControllerSim::spawn(sim_config(&drv)).unwrap();
    drv.wait_for_robot(Duration::from_secs(3)).unwrap();
    let mut q = drv.state().q;
    q[5] += 0.05;
    let motion = drv.move_to(q, Duration::from_secs(2)).unwrap();
    sim.stop().unwrap();
    assert!(wait_until(Duration::from_secs(2), || drv.session() == SessionState::Lost));
    let _sim2 = ControllerSim::spawn(sim_config(&drv)).unwrap();
    drv.wait_for_robot(Duration::from_secs(3)).unwrap();
    // The new session is idle, but the move never finished.
    assert!(drv.is_idle().unwrap());
    assert!(matches!(drv.wait(Duration::from_secs(1)).motion(motion).run(), Err(Error::LinkLost(_))));
}

#[test]
fn a_requested_stop_ends_the_session_as_stop_requested() {
    let (drv, sim) = connected();
    drv.stop_program().unwrap();
    assert_eq!(sim.wait_exit(Duration::from_secs(2)), Some(SimExit::Stopped));
    assert!(wait_until(Duration::from_secs(2), || drv.session() == SessionState::Lost));
    assert_eq!(drv.last_session_end(), Some(SessionEnd::StopRequested));
    std::thread::sleep(Duration::from_millis(120));
    let ev = events(&drv);
    assert!(ev.contains(&Event::SessionEnded(SessionEnd::StopRequested)), "{ev:?}");
    assert!(!ev.contains(&Event::SessionEnded(SessionEnd::LinkTimeout)), "{ev:?}");
}

#[test]
fn events_carry_the_time_of_their_cycle() {
    let (drv, _sim) = connected();
    std::thread::sleep(Duration::from_millis(120));
    let ev = drv.events();
    let connected = ev.iter().find(|e| matches!(e.event, Event::RobotConnected { .. })).expect("no RobotConnected");
    let mut log = Vec::new();
    drv.drain_log(&mut log);
    assert_eq!(log[0].cycle, 1);
    assert_eq!(connected.t, log[0].t, "the event has the time of the session's first log row");
    assert!(ev.windows(2).all(|w| w[0].t <= w[1].t), "times run forward: {ev:?}");
}
