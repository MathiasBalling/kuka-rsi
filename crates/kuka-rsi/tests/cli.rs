#![cfg(feature = "cli")]

use std::process::Command;

#[test]
fn binaries_print_help() {
    for bin in [env!("CARGO_BIN_EXE_rsi-sim"), env!("CARGO_BIN_EXE_rsi-monitor")] {
        let out = Command::new(bin).arg("--help").output().unwrap();
        assert!(out.status.success(), "{bin}");
        let text = String::from_utf8_lossy(&out.stdout);
        assert!(text.contains("--config"), "{text}");
    }
}

/// `rsi-files DIR` is `kuka_rsi::controller::write_files` on the command line: the same tree,
/// and one line per file saying where it goes on the robot.
#[test]
fn rsi_files_writes_the_controller_files() {
    use kuka_rsi::controller::FILES;

    let dir = std::env::temp_dir().join(format!("kuka-rsi-files-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    let out = Command::new(env!("CARGO_BIN_EXE_rsi-files")).arg(dir.join("usb")).output().unwrap();
    assert!(out.status.success(), "{}", String::from_utf8_lossy(&out.stderr));
    let text = String::from_utf8_lossy(&out.stdout);
    assert_eq!(text.lines().count(), FILES.len(), "{text}");
    for f in FILES {
        let path = dir.join("usb").join(f.local_path());
        assert_eq!(std::fs::read(&path).unwrap(), f.robot_bytes(), "{}", f.name);
        let line = format!("{}  ->  {}", path.display(), f.robot_dir.unwrap_or("(keep on the PC)"));
        assert!(text.lines().any(|l| l == line), "no line {line:?} in:\n{text}");
    }
    std::fs::remove_dir_all(&dir).unwrap();

    let no_dir = Command::new(env!("CARGO_BIN_EXE_rsi-files")).output().unwrap();
    assert!(!no_dir.status.success(), "a folder is required");
}

#[test]
fn sim_runs_for_a_fixed_number_of_cycles() {
    let cfg = concat!(env!("CARGO_MANIFEST_DIR"), "/controller/kuka_rsi_monitor.xml");
    let out = Command::new(env!("CARGO_BIN_EXE_rsi-sim"))
        .args(["--config", cfg, "--target", "127.0.0.1:49999", "--cycle-ms", "1", "--max-cycles", "50"])
        .output()
        .unwrap();
    assert!(out.status.success(), "{}", String::from_utf8_lossy(&out.stderr));
    assert!(String::from_utf8_lossy(&out.stdout).contains("MaxCycles"));
}

/// Ctrl-C in `rsi-monitor` must end the robot program cleanly (MoveStop pulse), not leave the
/// robot to time out. The simulator plays the robot and reports how its session ended. The
/// monitor runs without `--config`, on the built-in copy of the file the simulator reads.
#[cfg(unix)]
#[test]
fn ctrl_c_in_monitor_pulses_move_stop() {
    use std::io::{BufRead, BufReader};
    use std::process::Stdio;
    use std::sync::mpsc;
    use std::time::Duration;

    let cfg = concat!(env!("CARGO_MANIFEST_DIR"), "/controller/kuka_rsi_ethernet.xml");
    let addr = "127.0.0.1:49981";
    let mut monitor = Command::new(env!("CARGO_BIN_EXE_rsi-monitor"))
        .args(["--bind", addr, "--wait", "10"])
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .unwrap();
    let (tx, rx) = mpsc::channel();
    let out = monitor.stdout.take().unwrap();
    std::thread::spawn(move || {
        for line in BufReader::new(out).lines().map_while(Result::ok) {
            let _ = tx.send(line);
        }
    });
    let wait_for = |what: &str| loop {
        let line = rx.recv_timeout(Duration::from_secs(10)).expect(what);
        if line.contains(what) {
            break;
        }
    };
    wait_for("config: built-in kuka_rsi_ethernet.xml");
    wait_for("listening on");
    let sim = Command::new(env!("CARGO_BIN_EXE_rsi-sim"))
        .args(["--config", cfg, "--target", addr, "--max-cycles", "5000"])
        .stdout(Stdio::piped())
        .spawn()
        .unwrap();
    wait_for("robot connected");
    std::thread::sleep(Duration::from_millis(300));
    let kill = Command::new("kill").args(["-INT", &monitor.id().to_string()]).status().unwrap();
    assert!(kill.success());

    let sim_out = sim.wait_with_output().unwrap();
    let text = String::from_utf8_lossy(&sim_out.stdout);
    assert!(text.contains("exit Stopped"), "simulator did not see a MoveStop: {text}");
    let status = monitor.wait().unwrap();
    assert_eq!(status.code(), Some(130), "{status:?}");
}
