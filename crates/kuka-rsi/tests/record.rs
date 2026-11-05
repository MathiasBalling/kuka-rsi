use kuka_rsi::record::{CSV_HEADER, CsvLog, csv_row};
use kuka_rsi::state::{RobotState, SessionState};

fn state() -> RobotState {
    RobotState {
        t: 1.5,
        cycle: 42,
        ipoc: 1000,
        session: SessionState::Active,
        q: [10f64, -90.0, 90.0, 0.0, 45.0, -180.0].map(f64::to_radians),
        q_command: [10.5f64, -90.0, 90.0, 0.0, 45.0, -180.0].map(f64::to_radians),
        ak: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
        tcp: [0.5, -0.25, 1.0, 0.0, std::f64::consts::FRAC_PI_2, 0.0],
        torque: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        delay: 2,
        speed_override: 0.1,
        program_running: true,
        ..RobotState::default()
    }
}

#[test]
fn header_and_row_line_up_in_degrees_and_millimetres() {
    let names: Vec<&str> = CSV_HEADER.trim_end().split(',').collect();
    let row = csv_row(&state());
    let values: Vec<&str> = row.trim_end().split(',').collect();
    assert_eq!(names.len(), values.len(), "{CSV_HEADER}{row}");
    let get = |name: &str| values[names.iter().position(|n| *n == name).unwrap_or_else(|| panic!("no {name}"))];
    assert_eq!(get("t_s"), "1.500000");
    assert_eq!(get("cycle"), "42");
    assert_eq!(get("session"), "Active");
    assert_eq!(get("program_running"), "1");
    assert_eq!(get("override"), "0.100");
    assert_eq!(get("delay"), "2");
    assert_eq!(get("q_a1_deg"), "10.0000");
    assert_eq!(get("cmd_a1_deg"), "10.5000");
    assert_eq!(get("lag_a1_deg"), "0.5000", "lag is command minus actual");
    assert_eq!(get("ak_a1_deg"), "0.5000");
    assert_eq!(get("tcp_x_mm"), "500.000");
    assert_eq!(get("tcp_b_deg"), "90.0000");
    assert_eq!(get("torque_a6"), "6.000");
    assert!(row.ends_with('\n') && CSV_HEADER.ends_with('\n'));
}

#[test]
fn csv_log_writes_the_header_once_and_flushes_every_batch() {
    let dir = std::env::temp_dir().join(format!("kuka-rsi-record-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    let path = dir.join("runs").join("a.csv");
    let mut log = CsvLog::create(&path).unwrap();
    log.write(&[state(), state()]).unwrap();
    // Flushed without closing: a Ctrl-C exit keeps everything written so far.
    let text = std::fs::read_to_string(&path).unwrap();
    assert_eq!(text.lines().count(), 3, "{text}");
    log.write(&[state()]).unwrap();
    assert_eq!(log.rows(), 3);
    drop(log);
    let text = std::fs::read_to_string(&path).unwrap();
    assert_eq!(text.lines().count(), 4);
    assert_eq!(text.matches("t_s,").count(), 1);
    std::fs::remove_dir_all(&dir).unwrap();
}
