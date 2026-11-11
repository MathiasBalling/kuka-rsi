# kuka-rsi

KUKA.RobotSensorInterface (RSI) 5.0 client in Rust with Python bindings. One real-time
thread answers the robot's 4 ms telegrams; callers stream absolute joint targets and read
joint state. A protocol-level controller simulator makes it testable without a robot.

Tested on a KR10 R1100-2 with a KR C5 micro (KSS 8.7, RSI 5.0); see [Platforms](#platforms).

- Design: [docs/design.md](https://github.com/MathiasBalling/kuka-rsi/blob/main/docs/design.md)
- Controller files and setup: [controller/README.md](https://github.com/MathiasBalling/kuka-rsi/blob/main/crates/kuka-rsi/controller/README.md)
- First time at the robot, step by step: [controller/BRINGUP.md](https://github.com/MathiasBalling/kuka-rsi/blob/main/crates/kuka-rsi/controller/BRINGUP.md)
- Test sessions on the robot, notes and results: [docs/sessions/](https://github.com/MathiasBalling/kuka-rsi/blob/main/docs/sessions/README.md)
- Changes: [CHANGELOG.md](https://github.com/MathiasBalling/kuka-rsi/blob/main/CHANGELOG.md)

## Install

From the first release (v0.1.0) on:

```bash
pip install kuka-rsi      # Python 3.10+, wheels for Linux and macOS (x86_64, arm64)
cargo add kuka-rsi
```

## Files for the robot

The RSI contexts, Ethernet configs and KRL programs the controller needs ship with both
packages:

```bash
python -m kuka_rsi.controller robot_files/   # writes them, prints where each one goes
```

```text
robot_files/
  SensorInterface/   kuka_rsi_ethernet.xml, kuka_rsi_monitor.xml, kuka_rsi.rsix, kuka_rsi_monitor.rsix
                     -> C:\KRC\ROBOTER\Config\User\Common\SensorInterface\
  Program/           kuka_rsi.src, kuka_rsi_monitor.src (CRLF) -> KRC:\R1\Program\
  LICENSE-FZI, LICENSE-MIT, LICENSE-APACHE (stay on the PC)
```

One folder per target, so on the pendant you can open a folder on the USB stick, Edit > Mark
all, Edit > Copy and paste into the target. macOS AppleDouble files (`._name`) are removed
when writing to a FAT/exFAT stick. Without the package, from a checkout:
`python3 controller/stage_usb.py robot_files/` writes the same tree.

In Rust: `kuka_rsi::controller::write_files(Path::new("robot_files"))`, or
`kuka_rsi::controller::FILES` for the contents. `kuka_rsi_ethernet.xml` is also the
driver's config: the tools and the library have the shipped copy built in (`IP_NUMBER`
192.168.10.10, `PORT` 49152) and use it when no config is given. If the robot's copy differs,
for example another `IP_NUMBER` for the PC, pass that file (`--config`, `Driver(path)`,
`DriverConfig::from_ethernet_xml`). [controller/README.md](https://github.com/MathiasBalling/kuka-rsi/blob/main/crates/kuka-rsi/controller/README.md)
covers installing them on the controller.

## Layout

- `crates/kuka-rsi`: the library, the `rsi-sim`, `rsi-monitor` and `rsi-doctor` binaries (feature
  `cli`), examples and tests.
- `crates/kuka-rsi-py`: the `kuka_rsi` Python package (PyO3 + maturin).
- `crates/kuka-rsi/controller/` (also reachable as `controller/`): files for the robot.

## Build and test

```bash
cargo test -p kuka-rsi                     # Rust
cargo test -p kuka-rsi --features cli --test cli
cargo test -p kuka-rsi --test no_alloc      # allocation proof
cargo test -p kuka-rsi --release --test soak -- --ignored --nocapture
cargo fmt --all --check                    # CI enforces both
cargo clippy --workspace --all-targets --features kuka-rsi/cli -- -D warnings
uv --directory crates/kuka-rsi-py sync                                         # Python (add --reinstall-package kuka-rsi after Rust changes)
uv --directory crates/kuka-rsi-py run pytest tests
```

## Check the setup

`rsi-doctor` (feature `cli`) checks the PC side before a session: the config, that
`IP_NUMBER` is on this PC and the port is free, that traffic to the robot leaves from that
address, a ping to the robot, real-time priority and (macOS) the firewall. With `--wait` it
also checks the live link: start the robot program when it says "listening", and it holds the
robot for `--hold` seconds (default 3), reports the telegram rate, late replies and reply
latency, then ends the program with `MoveStop`.

```bash
cargo run -p kuka-rsi --features cli --bin rsi-doctor -- --robot 192.168.10.147 --wait 60
python -m kuka_rsi.doctor --robot 192.168.10.147 --wait 60
```

It exits non-zero if a check fails. In code: `kuka_rsi::doctor::run` (Rust) or
`kuka_rsi.check_setup` (Python), which return each check's status, detail and hint.

## Try it without the robot

```bash
cargo run -p kuka-rsi --features cli --bin rsi-sim -- --target 127.0.0.1:49152 --lag 3
cargo run -p kuka-rsi --example stream_trajectory -- --bind 127.0.0.1:49152
```

## Python

```python
import numpy as np
from kuka_rsi import Driver, Limits

limits = Limits.kr10_defaults(max_velocity=np.radians(30))
with Driver(limits=limits, stop_channel="MoveStop") as robot:   # built-in kuka_rsi_ethernet.xml
    state = robot.state()                       # .q, .q_setpoint, .tcp, .torque (numpy), .t, .ipoc, .delay
    robot.move_to(state.q + np.radians([0, 0, 0, 0, 0, 2]), duration=2.0, block=True)
    for chunk in policy():                      # (n, 6) absolute joint targets, radians
        robot.stream(chunk, dt=0.1, replace=True)
    for s in robot.drain_log():                 # every 4 ms cycle since the last call
        ...
    print(robot.stats())
```

Units: radians, metres, seconds in the API; degrees and millimetres only on the wire.
`stream` spreads each target over the cycles between points under the velocity and
acceleration limits; an empty queue holds position; playback follows the pendant
override and pauses when the program is stopped. A `stream` call is accepted whole or
rejected before anything is queued (`LimitError`); `move_to(block=True)` and `wait_idle`
raise `TimeoutError` if the robot cannot settle in time (a paused program, a limit
holding the command short). `Limits.kr10_defaults()` keeps an acceleration limit of
300°/s² with braking look-ahead; `max_acceleration=None` disables it.

## Platforms

- **Controller**: KR C5 / KR C5 micro with KSS 8.7 and RSI 5.0, tested with a KR10
  R1100-2. Other KUKA arms on the same controller should work with their own joint limits:
  `Limits::kr10_defaults()` holds the KR10 R1100-2 ranges, and every field of `Limits` can
  be set.
- **macOS**: tested at the robot (monitor, hold and jog). The real-time thread gets the
  highest normal thread priority; macOS has no real-time scheduling class for it.
- **Linux**: the intended deployment and covered by CI, not yet tried at the robot. The
  real-time thread asks for `SCHED_FIFO` priority 80, which needs `CAP_SYS_NICE` or an
  rtprio limit (`ulimit -r`, `/etc/security/limits.conf`). Without it the driver runs at
  normal priority and reports `RtPriority { ok: false }` as a warning.
- **Windows**: untested, no wheels.

## Behavior notes

- Ctrl-C in `rsi-monitor`, `rsi-doctor`, `jog` and `stream_trajectory` ends the robot
  program cleanly: the driver pulses `MoveStop`, waits up to 1 s for `RSI_MOVECORR()` to
  return, and the tool exits with code 130. In Python, leaving the `with Driver(...)` block
  (also on `KeyboardInterrupt`) does the same through `close()`, but a Ctrl-C during a
  blocking wait (`wait_idle`, `move_to(block=True)`, `wait_for_robot`) only takes effect
  when that wait returns. `check_setup` reacts to Ctrl-C at once.
- `fault(msg)` sends `Error: msg` to the robot and freezes the command. The fault
  persists until the KRL program restarts (a new session): meanwhile `is_faulted()` is
  true and `stream`, `move_to`, `notify` and `wait_idle` raise `Faulted` (Rust:
  `Error::Faulted`). `fault` and `stop_program` stay allowed. A fault is a state within
  the `Active` session, so `session()` does not show it.
- `set_speed_scale` survives a session restart: a scale of 0 set before the KRL program
  is restarted still pauses playback in the new session.
- `config().limits` (Rust) keeps the constructor value; `set_limits` changes the live
  limits, which `limits()` returns (Rust `Driver::limits()`, Python `Driver.limits()`).
- `link_timeout` must exceed the robot's own give-up time, the ETHERNET object's
  `Timeout` × cycle (100 × 4 ms = 0.4 s in the shipped context; the default 0.5 s does).
  Shorter, the driver may declare a session lost that the robot still keeps, and the next
  telegram would start a new session with `q_start` captured mid-motion.
- The cycle is 4 ms, the default sensor mode of `RSI_ON` (`#IPO_FAST`). For a robot program
  that runs the 12 ms mode (`RSI_ON(#ABSOLUTE, #IPO, #BASE)`) set `cycle_ms=12` (Rust:
  `DriverConfig::cycle`, tools: `--cycle-ms 12`); `link_timeout` then rises to 1.5 s. The 12 ms
  mode is not yet confirmed on a robot. If the robot's telegrams come at another cycle than
  the driver is set to, the driver faults the session within a few telegrams and motion calls
  raise `Faulted` with "cycle mismatch" (Rust: `Error::CycleMismatch`): durations and speed
  limits are counted in cycles, so motion would otherwise run at the wrong speed.
- The tracking-error warning (`Limits.max_tracking_error`, default 2°) compares the
  actual position with the command just sent, so it must exceed `max_velocity` × the
  robot's lag (measure the lag at bring-up); below that it fires on every move.

## Wire format (from the shipped config)

Robot to PC every 4 ms:

```xml
<Rob Type="KUKA"><RIst X=".." Y=".." Z=".." A=".." B=".." C=".." /><RSol .../>
<AIPos A1=".." ... A6=".." /><ASPos .../><Delay D="0" /><GearTorque A1=".." ... />
<ProgStatus R="3" /><OvPro R="100" /><IPOC>123645634563</IPOC></Rob>
```

The root's `Type` is always `KUKA` (seen on the real controller); `SENTYPE` from the config
names only the replies.

PC to robot, same IPOC, within the cycle:

```xml
<Sen Type="KukaRsi"><AK A1="0.000000" ... A6="0.000000" /><MoveStop>0</MoveStop><EStr/><IPOC>123645634563</IPOC></Sen>
```

`AK` is the absolute correction from the setpoint at `RSI_ON` (`q_start`, captured from
the first telegram's `ASPos`), so `AK = q_target - q_start` in degrees. `MoveStop` ends
`RSI_MOVECORR()`; `<EStr>Error: ...</EStr>` stops the robot.

## License

Licensed under either of [Apache License, Version 2.0](https://github.com/MathiasBalling/kuka-rsi/blob/main/LICENSE-APACHE)
or [MIT license](https://github.com/MathiasBalling/kuka-rsi/blob/main/LICENSE-MIT) at your
option. The RSI contexts and KRL programs in `controller/` are adapted from FZI's
`kuka_rsi_driver` and keep its BSD-3-Clause license
([controller/LICENSE-FZI](https://github.com/MathiasBalling/kuka-rsi/blob/main/crates/kuka-rsi/controller/LICENSE-FZI)).

Unless you explicitly state otherwise, any contribution intentionally submitted for
inclusion in this work, as defined in the Apache-2.0 license, shall be dual licensed as
above, without any additional terms or conditions.
