# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/). Both packages (the `kuka-rsi` crate and the
`kuka-rsi` Python package) share one version number.

## [Unreleased]

First release, planned as 0.1.0 after the streaming and fault paths are confirmed on a
real robot.

### Added

- RSI 5.0 client: one real-time thread answers every 4 ms telegram from the robot with
  absolute axis corrections (`AK`), from a telegram schema compiled out of the
  controller's Ethernet config; allocation-free on the hot path.
- Joint streaming (`stream`), point-to-point moves (`move_to`), hold, speed scaling,
  live limits, faults sent to the robot (`fault`), and a clean program end through the
  `MoveStop` channel (`stop_program`, `close`).
- Per-cycle limiter for position, velocity, acceleration (with braking look-ahead) and
  correction range; tracking-error warning.
- Robot state in SI units, per-cycle log, statistics (late replies, reply latency) and
  events.
- Protocol-level controller simulator (`ControllerSim`, `rsi-sim`) with lag, jitter,
  packet loss and the controller's timeout behaviour.
- `rsi-monitor` and `rsi-files` binaries and `jog` / `stream_trajectory` examples (feature `cli`); Ctrl-C
  ends the robot program cleanly.
- Setup checks (`kuka_rsi::doctor`, `rsi-doctor`, `kuka_rsi.check_setup`,
  `python -m kuka_rsi.doctor`): config, address, port, route, ping, real-time priority,
  firewall, and the live link with its timing. `Driver::robot_addr` reports the robot's
  address during a session.
- The shipped `kuka_rsi_ethernet.xml` is built in and is the default config everywhere:
  `rsi-monitor`, `rsi-doctor`, `rsi-sim` without `--config`, `jog` and `stream_trajectory`
  without a file, `Driver()` and `check_setup()` without one, and
  `EthernetConfig::shipped`, `DriverConfig::shipped` and `doctor::Options::default` in Rust.
- The cycle is a setting for robot programs that run the 12 ms sensor mode
  (`RSI_ON(#ABSOLUTE, #IPO, #BASE)`): `--cycle-ms` on `rsi-monitor`, `rsi-doctor`, `jog`,
  `stream_trajectory` and the Python doctor and example, `cycle_ms` in `Driver` and
  `check_setup`, `DriverConfig::cycle` (which also raises `link_timeout` to 125 cycles). A
  session whose telegrams come at another cycle than configured is faulted within a few
  telegrams (`Event::CycleMismatch`, `Error::CycleMismatch`, `Driver::cycle_mismatch`;
  `Faulted` in Python), and the doctor's timing check names the robot's cycle. The controller
  files include `kuka_rsi_12ms.src`, the same program in that mode. The 12 ms mode is not
  yet confirmed on a robot.
- Python bindings (`kuka_rsi`, one abi3 wheel per platform for CPython 3.10+, numpy
  1.24+) with type stubs.
- Controller files for KR C5 / KSS 8.7: RSI contexts, Ethernet configs and KRL programs,
  bundled in both packages (`kuka_rsi::controller`, `python -m kuka_rsi.controller DIR`,
  the `rsi-files` binary), plus a step-by-step bring-up guide. `write_files` writes them into `SensorInterface/` and `Program/` folders, one per target on the
  robot, with the licenses beside them, and removes macOS AppleDouble files on USB sticks.
  `kuka_rsi.src` starts and parks with `PTP $AXIS_ACT`, so it makes no move before RSI.
- CSV logging (`kuka_rsi::record`, `RobotState.csv_row`, `CSV_HEADER`) and `--log` on
  `rsi-monitor`, `jog` and `stream_trajectory`; `rsi-monitor --keep` waits for the next
  session. `jog` takes multi-axis moves (`--move A1=30,A3=-15`), `--speed` and `--repeat`;
  `stream_trajectory` takes `--axes` and `--amp`; a Python streaming example
  (`examples/stream_path.py`).

[Unreleased]: https://github.com/MathiasBalling/kuka-rsi/commits/main
