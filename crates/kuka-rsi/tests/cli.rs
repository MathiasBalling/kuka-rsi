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

#[test]
fn sim_runs_for_a_fixed_number_of_cycles() {
    let cfg = concat!(env!("CARGO_MANIFEST_DIR"), "/../../controller/kuka_rsi_monitor.xml");
    let out = Command::new(env!("CARGO_BIN_EXE_rsi-sim"))
        .args(["--config", cfg, "--target", "127.0.0.1:49999", "--cycle-ms", "1", "--max-cycles", "50"])
        .output()
        .unwrap();
    assert!(out.status.success(), "{}", String::from_utf8_lossy(&out.stderr));
    assert!(String::from_utf8_lossy(&out.stdout).contains("MaxCycles"));
}
