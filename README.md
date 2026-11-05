# kuka-rsi

KUKA.RobotSensorInterface 5.0 client in Rust with Python bindings, for the KR10
R1100-2 on a KR C5 micro (KSS 8.7). One real-time thread answers the robot's 4 ms
telegrams; callers stream absolute joint targets and read joint state.

Design: `docs/design.md`.
Controller files and bring-up: `controller/README.md`.
First time at the robot, step by step: `controller/BRINGUP.md`.

## Layout

- `crates/kuka-rsi`: the library, the `rsi-sim` and `rsi-monitor` binaries (feature
  `cli`), examples and tests.
- `crates/kuka-rsi-py`: the `kuka_rsi` Python package (PyO3 + maturin).
- `controller/`: files for the robot.

## Build and test

```bash
cargo test -p kuka-rsi                     # Rust
cargo test -p kuka-rsi --features cli --test cli
cargo test -p kuka-rsi --test no_alloc      # allocation proof
cargo test -p kuka-rsi --release --test soak -- --ignored --nocapture
uv --directory crates/kuka-rsi-py sync                                         # Python (add --reinstall-package kuka-rsi after Rust changes)
uv --directory crates/kuka-rsi-py run pytest tests
```

## Try it without the robot

```bash
cargo run -p kuka-rsi --features cli --bin rsi-sim -- \
    --config controller/kuka_rsi_ethernet.xml --target 127.0.0.1:49152 --lag 3
cargo run -p kuka-rsi --example stream_trajectory -- \
    controller/kuka_rsi_ethernet.xml 127.0.0.1:49152
```

## Python

```python
import numpy as np
from kuka_rsi import Driver, Limits

limits = Limits.kr10_defaults(max_velocity=np.radians(30))
with Driver("controller/kuka_rsi_ethernet.xml", limits=limits, stop_channel="MoveStop") as robot:
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

## Behavior notes

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
