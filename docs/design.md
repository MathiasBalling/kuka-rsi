# KUKA RSI driver in Rust, with Python bindings

Design for the `kuka-rsi` crate: a KUKA.RobotSensorInterface 5.0 client for the
KR10 R1100-2 on a KR C5 micro running KSS 8.7. Written 2026-09-24 against the RSI 5.0
manual (KST RSI 5.0 V3, 11.12.2024), the FZI ROS 2 driver
(`kuka_rsi_driver`, tested on a KR C5) and `rsi-pi` (`RSIPI` 0.3.0, tested on a KR C4).

Status: implemented. Section 15 lists the rulings made during implementation that
change what the earlier sections say.

## 1. Purpose and scope

Ethernet KRL issues blocking PTP/LIN moves and cannot stream targets. A learned or
scripted controller needs a 10 to 20 Hz stream of absolute joint targets, executed
smoothly, and joint state read back for recording. RSI gives a 4 ms UDP cycle for
exactly that.

Goals, in priority order:

1. A Rust library that speaks RSI 5.0 over UDP reliably at the 4 ms cycle: replies always
   on time, never blocked by the caller, safe behaviour when targets stop arriving.
2. Joint-space streaming: the caller pushes absolute joint targets at any rate, the driver
   interpolates them onto the 4 ms clock under velocity and acceleration limits.
3. Joint state at every cycle for logging, and a wait-free "latest state" read.
4. Python bindings with numpy arrays: context manager, waits with a timeout, plain
   getters.
5. A protocol-level simulator of the controller so everything above is tested without
   the robot, in Rust and in pytest.

Non-goals for the first version (section 13 lists what is prepared for but not built):
Cartesian correction, external axes, digital I/O, a physics simulator backend behind the
same API, safety-rated behaviour of any kind. The 12 ms `#IPO` mode is not a target either:
the driver takes the cycle as a setting (`DriverConfig::cycle`, `--cycle-ms`) and refuses a
session whose telegrams come at another one, but the shipped program runs `#IPO_FAST` and
12 ms is not confirmed on a robot.

## 2. Facts that drive the design

All from the RSI 5.0 manual unless marked otherwise. Page numbers refer to the manual.

| Fact | Source | Consequence |
| --- | --- | --- |
| The controller initiates: after `RSI_ON` it sends a datagram to `IP_NUMBER:PORT` from the Ethernet config and expects a reply every cycle | p. 14, 60 | The PC is a UDP server bound to its own address. The robot's address is learned from the first datagram |
| A reply must arrive within the sensor cycle. Late packets are rejected and counted. When the count exceeds the ETHERNET object's `Timeout` (100 in both reference drivers, KUKA default 10) the robot stops with `RSIBad` (KSS29002) | p. 15; rsi-pi, FZI contexts | Reply latency budget is well under 4 ms, every cycle, forever. The loop that replies must never wait on anything else |
| Sensor cycle: 4 ms in `#IPO_FAST` (default), 12 ms in `#IPO`. `#IPO` corrects only LIN/CIRC paths | p. 16, 49-50 | Use `#IPO_FAST`, 250 Hz |
| Telegram format is user-defined XML from `RSI_EthernetConfig`-style file: `<Rob TYPE="KUKA">` with one element per configured tag, `<IPOC>` last; reply `<Sen Type="SENTYPE">` with the configured receive elements and the same `<IPOC>` | p. 61-67 | Parser and writer are driven by that config file; the driver reads the same file as the controller |
| `IPOC` must be echoed unchanged; a mismatched timestamp invalidates the packet. On hardware it is a millisecond counter advancing by 4 per packet | p. 63; rsi-pi hardware findings | Echo only. Use it for loss detection and pacing, never modify it |
| The controller checks `SENTYPE` on every received packet | p. 60 | `Type` attribute comes from the config, not a constant |
| `HOLDON="1"` keeps the last valid value of a receive element when a packet is late, `HOLDON="0"` resets it to 0 | p. 63 | In absolute mode use `HOLDON="1"`: a reset to 0 would command a jump back to the start pose |
| Correction mode `#ABSOLUTE`: the correction is an offset from the programmed setpoint. `#RELATIVE`: each cycle's value is added to the accumulated correction (a held value acts as a velocity) | p. 20, 23 | Use `#ABSOLUTE` with `AXISCORR`: `AK = q_target - q_start`. Deterministic, no drift, a repeated packet is harmless |
| Under `RSI_MOVECORR()` the robot has no programmed path; it holds the start pose and moves purely on corrections | p. 22-24, 51 | `q_start` is the setpoint of the first telegram (`ASPos`) and stays constant. `ASPos` never reflects the correction; `AIPos` is the actual position (rsi-pi measured this) |
| `AXISCORR` clamps the cumulative correction to `LowerLim/UpperLim` silently (KSS29001 notification only). Without `AXISCORRMON` an overall limit of 6 degrees applies and exceeding it stops the robot (KSS29000) | p. 17, 113-115; rsi-pi | Ship an `AXISCORRMON`. Keep the robot-side clamps wide and enforce real joint limits in the driver, so a clamp is never silent |
| `DEF_EStr` receive keyword: `<EStr>Error: text</EStr>` raises an acknowledgement message and stops the robot; `<EStr>text</EStr>` is a notification | p. 66 | Software fault stop from the PC |
| A `STOP` object in `ExitMoveCorr` mode ends `RSI_MOVECORR()` on a rising edge of a signal; the KRL program then continues | p. 130; rsi-pi verified on a KR C4 (Mode=4) | Clean shutdown: pulse a `MoveStop` channel, the KRL program runs `RSI_OFF` and parks |
| `ONLYSEND=TRUE`: the robot streams telegrams and expects no reply; verified on hardware for 30 s without `RSIBad` | p. 60; rsi-pi | A read-only monitor mode for the first minutes on hardware |
| `DEF_Delay` reports the robot's own late-packet count; `Precision` (ETHERNET object parameter) sets the decimals of what the robot sends, default 1 | p. 65; rsi-pi, FZI | Publish `Delay`; set `Precision=4` as FZI does on their KR C5 |
| RSI 5.0 needs KSS 8.7.x, WorkVisual 6.0, a real-time capable external system with 100 Mbit full duplex, and a dedicated sensor network in its own subnet ("Mixed IP address" interface on KLI or KONI; `192.168.0.x` and the existing KLI subnet are blocked) | p. 31, 41-42; rsi-pi | The RSI link cannot reuse the EKI addresses `172.31.1.147/77`. It needs a second subnet, e.g. `192.168.10.0/24`, and the PC needs an alias on that NIC |
| On KR C5 the context is a single `.rsix` file and `RSI_CREATE` takes the name without extension | p. 48; FZI KR_C5 files | Ship `.rsix`, not the KR C4 `.rsi/.rsi.xml/.rsi.diagram` triple |
| Reference drivers' pitfalls: edge-triggered receive that falls one packet behind, no PC-side watchdog, offset captured once and never reset after an RSI restart, per-cycle logging from the real-time thread, a Python `Manager` read of 1.8 to 4.3 ms inside the reply path causing 10 % late packets under load | FZI code review; rsi-pi hardware findings | Drain to the newest datagram, watchdog and restart handling, no logging or allocation in the loop, nothing Python-side in the reply path |

## 3. Approaches considered

**A. Config-driven schema, dedicated real-time thread, PyO3 bindings (recommended).**
The driver parses the same `RSI_EthernetConfig` XML the controller uses and compiles it
into a fixed telegram layout at start. The 4 ms loop runs on one Rust thread that never
touches Python, allocates nothing and logs nothing. Callers push commands through a
lock-free queue and read state through a wait-free buffer. Python gets the Rust `Driver`
through PyO3, built with maturin.

Trade-off: two config sources (the XML for the wire, a small struct for limits) and a
schema compile step, in exchange for one source of truth for the telegram layout and a
hot path that is slot lookups only.

**B. Hard-coded telegram schema (FZI style).** Parser and writer know exactly the tags of
one shipped config. Simplest and fastest, but the controller files and the Rust code are
two copies of the same layout that drift by hand, and every extra channel (a digital
output, a second sensor value) is a code change plus a rebuild.

**C. Generic dictionary telegrams (rsi-pi style).** Everything is a map from tag name to
value; maximal flexibility. The hot path hashes strings and allocates, and the API is
stringly typed (`robot["AKorr"]["A1"]`).

Rejected variants of the threading and binding model:

- An async runtime (tokio) for the loop: a hard 4 ms deadline wants one blocking thread
  with a raised priority, not a scheduler.
- Python-owned loop with Rust only for XML: rsi-pi shows the reply path cannot contain
  Python (GIL, GC pauses, IPC).
- A separate driver process with IPC to Python: robust but doubles the surface. The
  PyO3 thread model gives the same isolation inside one process.

Approach A is the recommendation and the rest of this document describes it.

## 4. Architecture

```
                 Python (policy, recorder, scripts)            Rust callers
                        │  kuka_rsi.Driver (PyO3)                 │
                        └───────────────┬─────────────────────────┘
                                        │ Driver handle (Send + Sync)
        ┌───────────────────────────────┼──────────────────────────────────┐
        │ commands: ArrayQueue<Command> │ state: triple buffer (wait-free) │
        │ (targets, hold, fault, ...)   │ log: rtrb ring of every cycle    │
        │                               │ events: ring of Event (for logs) │
        └───────────────┬───────────────┴───────────────┬──────────────────┘
                        ▼                               ▲
        ┌──────────────────────────────── kuka-rsi-rt thread ─────────────────┐
        │  UdpLink.recv_newest ─▶ parse <Rob> ─▶ update RobotState            │
        │  ─▶ drain commands ─▶ Interpolator.step ─▶ Limiter ─▶ AK = q - q0   │
        │  ─▶ write <Sen> ─▶ UdpLink.send ─▶ publish state, stats, log        │
        └──────────────────────────────────────────────────────────────────────┘
                        ▲ UDP <Rob> every 4 ms          │ UDP <Sen>, same IPOC
                        │                               ▼
                    KR C5 micro: kuka_rsi.src ▶ RSI_ON(#ABSOLUTE) ▶ RSI_MOVECORR()
                    context kuka_rsi.rsix: Ethernet ─▶ AxisCorr, AxisCorrMon, Stop
```

Session state machine in the real-time thread:

```
Listening ──first telegram──▶ Active ──no telegram for link_timeout──▶ Lost ──▶ Listening
              (q_start = ASPos,          (robot has already stopped:            (next RSI_ON
               queue cleared, Hold)       100 late packets = 0.4 s)              starts fresh)
```

## 5. Components

Each is a module of the `kuka-rsi` crate with one job and its own tests.

### 5.1 `config`

- `EthernetConfig`: serde model of the controller's XML (`CONFIG` with `IP_NUMBER`,
  `PORT`, `SENTYPE`, `ONLYSEND`; `SEND`/`RECEIVE` element lists with `TAG`, `TYPE`,
  `INDX`, `HOLDON`). Loaded with quick-xml's serde support. Validated: consecutive
  indices, at most 64 per direction, known `DEF_*` keywords, `AK.A1..A6` present unless
  `ONLYSEND`.
- `DriverConfig`: bind address override (default: `IP_NUMBER:PORT` from the XML),
  `cycle` (4 ms), `link_timeout` (500 ms), `robot_wait_timeout`, `rt_priority`
  (try `SCHED_FIFO` on Linux), `stop_channel` (name of the receive element wired to the
  `STOP` object, optional), `limits`.
- `Limits`: per-joint `position_min/max` (rad), `max_velocity` (rad/s), optional
  `max_acceleration` (rad/s²), `max_correction` (rad, mirrors `AXISCORR` limits),
  `max_tracking_error` (rad, warning threshold). Defaults are conservative and must be
  set from the KR10 machine data (`$SOFTN_END/$SOFTP_END`, axis velocities) during
  bring-up; the spec does not invent datasheet numbers.
- Everything derives `Serialize`/`Deserialize` so a run can dump its config next to a
  recorded dataset.

### 5.2 `telegram`

- `Schema`: compiled from `EthernetConfig` once. For the robot side, a list of
  elements: `Keyword(RIst | RSol | AIPos | ASPos | EIPos | ESPos | MACur | MECur | Delay |
  Tech(n))` mapped to typed fields, or `User { tag, attrs: [(name, slot)] }` /
  `UserText { tag, slot }` mapped into a fixed `[f64; 64]` slot array (LONG and BOOL are
  stored as f64; BOOL is parsed by numeric value because the robot may send `1.000`).
  For the PC side, the receive elements in config order: `AK`, `RKorr`, `EK`, `EStr`,
  `Tech(n)`, or user elements by slot.
- `RobotTelegram`: plain struct, degrees and millimetres exactly as on the wire, plus
  `ipoc: u64`, `sentype_ok: bool`, arrival `Instant`.
- `parse(schema, bytes, &mut RobotTelegram) -> Result<(), ParseError>`: quick-xml
  borrowed events over the receive buffer, no allocation. Unknown elements are skipped;
  a missing expected element keeps the previous value and increments a counter; a root
  other than `Rob` or a `TYPE` other than the configured `SENTYPE` is a `ParseError`
  and the packet is not answered.
- `SensorTelegram`: `ak: [f64; 6]` (deg), optional `rkorr`, `estr: ArrayString<64>`,
  user slots, `ipoc`.
- `write(schema, &SensorTelegram, &mut Vec<u8>)`: writes into a reused buffer, doubles
  with six decimals, elements in config order, `<IPOC>` last. `EStr` is XML-escaped.
- Test fixtures: the manual's example documents (p. 61, 63, 66, 67), FZI's byte-exact
  writer expectation, and, after the first hardware session, real captured datagrams.

### 5.3 `link`

- `UdpLink::bind(addr)`: `socket2` socket, no `SO_REUSEADDR` (rsi-pi: a leftover KUKA
  TestServer silently stole packets while the bind "succeeded"), 64 KiB receive buffer,
  read timeout as the loop's tick.
- `recv_newest(&mut buf, timeout) -> Option<Received { len, from, arrived, skipped }>`:
  block for one datagram, then drain non-blocking and keep only the last. Answering a
  stale packet is worse than dropping it.
- Peer lock: after the first datagram in a session only that source address is accepted;
  others are counted and ignored.
- `send(&[u8])` to the locked peer. `ONLYSEND` disables the send path entirely.

### 5.4 `control`

- `Command` enum, pushed by callers: `Stream { points, cycles_per_point, replace }`,
  `MoveTo { q, cycles }`, `Hold`, `Fault(msg)`, `Notify(msg)`, `StopProgram`,
  `SetSpeedScale(f)`, `SetLimits(Limits)`. Point lists are pre-allocated fixed-capacity
  arrays (`ArrayVec<[f64; 6], 64>`, longer chunks are split by the API) so a command is
  `Copy`-sized and the queue is
  `crossbeam::ArrayQueue<Command>` (bounded, multi-producer, lock-free).
- `Interpolator`: a queue of segments on the robot's cycle clock. Time advances by
  `speed_scale × OvPro/100` cycles per received telegram, and by 0 while
  `ProgStatus` is not "running", so the pendant override slows playback instead of
  producing a jump, and a program stop pauses the queue. Segments are linear between
  consecutive stream points (the policy owns smoothness) and quintic for `MoveTo`. When
  the queue runs dry the last point is held and `starved_cycles` counts. `replace=true`
  drops everything after the current cycle and starts the new chunk from the current
  commanded position, which is what real-time chunking needs.
- `Limiter`: per cycle, clamp the command to `position_min/max`, then clamp the delta
  from the previously sent command to `max_velocity × cycle` (and the delta of deltas to
  `max_acceleration × cycle²` if set), then clamp `q - q_start` to `max_correction`.
  Every clamp increments a counter; nothing is silent. This is also what turns a coarse
  20 Hz step into a spread of 4 ms steps (rsi-pi's "hammer blow" lesson).
- `Session`: owns `q_start`, the mode (`Hold` until the first target) and the tracking
  check `|AIPos - (q_start + AK)|` against `max_tracking_error` (warning event only; in
  T1 the controller may limit speed and this is how we will see it).
- Fault: `Fault(msg)` sends `<EStr>Error: msg</EStr>` for 25 cycles and freezes the
  last sent `AK` (never zero it in absolute mode: zero means "return to start").
- Stop: `StopProgram` holds the `stop_channel` element at 1 for 25 cycles, then 0, and
  the session expects the link to end.

### 5.5 `driver`

- `Driver::spawn(DriverConfig) -> Result<Driver>`: binds the socket first (fail fast on
  a wrong address), spawns `kuka-rsi-rt`, tries to raise its priority (a warning event if
  refused), returns immediately in `Listening`.
- `wait_for_robot(timeout)`, `state() -> RobotState` (wait-free copy from a
  `triple_buffer` output; multiple readers are serialized by a mutex on the caller side,
  never on the real-time side), `session() -> SessionState`, `stats() -> Stats`,
  `events() -> Vec<Event>` (drained ring), `log() -> LogReader` (an `rtrb` consumer of
  every-cycle `RobotState`; when Python does not drain it the ring drops newest and
  counts), `stream`, `move_to`, `wait_idle`, `hold`, `fault`, `notify`, `stop_program`,
  `set_speed_scale`, `set_limits`, `close()`.
- `close()`: pulse the stop channel if configured, wait up to 1 s for the link to end,
  then stop the thread. Without a stop channel it holds until the caller stops the KRL
  program, then times out. `Drop` calls `close()`.
- Logging: the real-time thread pushes `Event`s (session change, first clamp, late
  reply, parse error, priority refused) into a ring; a low-priority `kuka-rsi-monitor`
  thread drains it and logs through the `log` facade, rate-limited. The Python package
  routes those into `logging` via `pyo3-log`, which is safe because that thread may take
  the GIL.

### 5.6 `state` and `stats`

- `RobotState` (SI units): `t` (monotonic seconds at arrival), `cycle` (count in this
  session), `ipoc`, `q` and `q_setpoint` (rad, 6), `tcp` and `tcp_setpoint`
  (x, y, z in m; a, b, c in rad, KUKA Z-Y'-X'' Euler), `torque` (N m, as reported),
  `delay` (robot's late count), `override` (0 to 1), `program_running`, `q_command`
  (what we sent, rad), `ak` (deg, as sent). Derives serde and is `Copy`.
- `Stats`: cycles, replies, late replies (reply latency above `cycle`), max and mean
  reply latency, inter-arrival jitter (max, p99 from a small histogram), `ipoc` gaps,
  datagrams skipped by the drain, parse errors, sentype mismatches, clamps per kind,
  starved cycles, log drops, session count.

### 5.7 `sim`: the fake controller

`ControllerSim` speaks the same schema and runs a free-running 4 ms clock:

- Sends `<Rob TYPE="SENTYPE">` each tick with `AIPos`, `ASPos` (constant start pose),
  `RIst`/`RSol` (zeros, or a constant; the simulator has no kinematics), `Delay`, user
  slots, `IPOC` advancing by 4 ms.
- Waits until the next tick for `<Sen>`; checks `Type` and exact `IPOC`; a late or
  missing reply increments `Delay`, applies `HOLDON` per element, and after `timeout`
  late packets (running total, configurable to consecutive) reports `RsiBad` and stops
  sending (`ProgStatus` 4).
- Applies `AK` in absolute mode: `q_setpoint_internal = q_start + AK`, clamped to the
  `AXISCORR` limits (counts KSS29001), stops on the `AXISCORRMON` limit (KSS29000).
  `AIPos` follows through a first-order lag and a velocity cap, so tracking error and
  starvation are observable.
- Honours `EStr` starting with `Error:` (stop) and the stop channel (ends the session as
  the KRL program would).
- Optional disturbances: Gaussian send jitter, a drop probability, a burst stall.
- Exposed as a library type, the `rsi-sim` binary, and `kuka_rsi.sim.ControllerSim` in
  Python.

### 5.8 `kuka-rsi-py`

- PyO3 `cdylib` named `kuka_rsi._core`; a thin pure-Python layer `kuka_rsi/__init__.py`
  adds docstrings, dataclass-like `RobotState` views and type stubs (`_core.pyi`,
  `py.typed`).
- Blocking calls (`wait_for_robot`, `move_to(block=True)`, `log()` iteration) release
  the GIL. Arrays cross as `numpy` float64 via the `numpy` crate; scalars as Python
  floats and ints.
- Exceptions: `kuka_rsi.Error` (base), `kuka_rsi.LinkLost`, `kuka_rsi.ProtocolError`,
  `kuka_rsi.LimitError` (a target outside `position_min/max` or a non-finite value is
  rejected at the API, not clamped), and the built-in `TimeoutError`.
- Built by maturin from `crates/kuka-rsi-py`.

## 6. Controller side (`kuka-rsi/controller/`)

| File | Goes to | Content |
| --- | --- | --- |
| `kuka_rsi_ethernet.xml` | `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\` | `CONFIG`: PC address on the RSI subnet, port 49152, `SENTYPE` `KukaRsi`, `ONLYSEND` FALSE. `SEND`: `DEF_RIst`, `DEF_RSol`, `DEF_AIPos`, `DEF_ASPos`, `DEF_Delay`, `GearTorque.A1..A6` (1-6), `ProgStatus.R` (7), `OvPro.R` (8). `RECEIVE`: `AK.A1..A6` (1-6, `HOLDON="1"`), `MoveStop` BOOL (7, `HOLDON="0"`), `DEF_EStr` |
| `kuka_rsi_monitor.xml` | same | Identical with `ONLYSEND` TRUE, for the read-only first step |
| `kuka_rsi.rsix` | same | RSI 5.0 context adapted from FZI's KR C5 file: `Ethernet` (ConfigFile, `Timeout` 100, `Flag` 1, `Precision` 4), `AxisCorr` fed by Ethernet outputs 1-6 with limits equal to the axis ranges, `AxisCorrMon` with the same, `GearTorque`, `Status` (program state), `OV_PRO` into Ethernet inputs, `Stop` (`ExitMoveCorr`) fed by Ethernet output 7 |
| `kuka_rsi.src` | `KRC:\R1\Program\` | INI fold, `PTP $AXIS_ACT` (block coincidence where the robot stands; a taught start pose, if used, should have the wrist bent, not `HOME`, see rsi-pi's A5 = 0 finding), `RSI_CREATE("kuka_rsi", CONTID, TRUE)`, `RSI_ON(#ABSOLUTE)` (default `#IPO_FAST`), `RSI_MOVECORR()`, `RSI_OFF()`, `RSI_DELETE`, `PTP $AXIS_ACT`. Every return value checked with `HALT` on error |
| `kuka_rsi_12ms.src` | same | `kuka_rsi.src` with `RSI_ON(#ABSOLUTE, #IPO, #BASE)`, the 12 ms mode (added later; not confirmed on a robot) |
| `kuka_rsi_monitor.src` | same | The monitor variant: `RSI_ON`, `WAIT SEC 60`, no `RSI_MOVECORR` |
| `README.md` | | Where each file goes, the pendant network configuration (Start-up > Network configuration > KLI > Advanced > add interface, Mixed IP address, own subnet, cold start), start order (driver first, then the program), and the bring-up checklist from section 10 |

The `.rsix` should be opened and saved once in RSIVisual (WorkVisual 6.0) so all
parameter ordinals are regenerated by KUKA's tool rather than trusted from a hand edit
(rsi-pi lost time to a wrong ordinal that still loaded). Add objects one at a time when
extending it.

## 7. Public API

### 7.1 Rust

```rust
use kuka_rsi::{Driver, DriverConfig, Limits};
use std::time::Duration;

let cfg = DriverConfig::shipped()?                    // or from_ethernet_xml(path) for another copy
    .limits(Limits::kr10_defaults().max_velocity_all(0.5))   // rad/s, bring-up value
    .stop_channel("MoveStop");
let driver = Driver::spawn(cfg)?;                 // bound and listening
driver.wait_for_robot(Duration::from_secs(60))?;  // first telegram: session Active

let s = driver.state();                           // RobotState, SI units
driver.move_to(&target, Duration::from_secs(2))?; // quintic profile, non-blocking
driver.wait_idle(Duration::from_secs(3))?;        // queue drained
driver.stream(&chunk, Duration::from_millis(100), true)?; // 10 Hz chunk, replace queue
driver.hold();                                    // freeze at the current command
driver.fault("policy diverged");                  // EStr "Error: ..." → robot stops
let st = driver.stats();
driver.close()?;                                  // pulse MoveStop, thread joins
```

### 7.2 Python

```python
import numpy as np
from kuka_rsi import Driver, Limits

limits = Limits.kr10_defaults(max_velocity=np.radians(30))
with Driver(limits=limits, stop_channel="MoveStop") as robot:  # or Driver(path, ...)
    # __enter__ spawned the thread and waited for the first telegram (timeout=60 s)
    state = robot.state()            # .q, .q_setpoint, .tcp, .torque: np.ndarray, .t, .ipoc, .delay, .override
    robot.move_to(q_start, duration=2.0, block=True)
    for chunk in policy():           # (10, 6) absolute joint targets, 10 Hz
        robot.stream(chunk, dt=0.1, replace=True)
    robot.hold()
    for s in robot.log():            # every-cycle states, for recording
        ...
    print(robot.stats())
```

The context manager connects and cleans up even if a move raises, waits take a
timeout, getters return plain values. Everything is non-blocking unless asked, and
units are SI.

## 8. Error handling and safety behaviour

- Bind failure, a wrong XML, or no robot within the timeout: errors at the call site,
  nothing running afterwards.
- Late reply on our side: counted; the robot decides what to do with it (`HOLDON`).
- No telegram for `link_timeout`: session `Lost`, `Event::LinkLost`, queue cleared,
  `Driver.state().session` reports it, blocking calls return `LinkLost`. The thread goes
  back to `Listening`, so restarting the KRL program reconnects without restarting the
  process and `q_start` is re-captured. (FZI never resets it; rsi-pi reconnects by
  restarting the whole process.)
- Target stream stops: hold the last point. A fresh chunk resumes from the held pose.
- Target outside joint limits or non-finite: rejected at the API with `LimitError`.
  Velocity and acceleration limits are enforced by clamping, counted in `Stats`, and
  reported once as an `Event` per burst.
- Parse errors and `SENTYPE` mismatches: not answered, counted; after 10 in a row an
  `Event::ProtocolError`.
- `fault(msg)`: the robot stops through `EStr`; our side freezes the last `AK`. Nothing
  in this driver is safety-rated; the cell's E-stop chain and T1 mode remain the safety
  layers.
- The real-time thread cannot panic outward: every cycle body runs under
  `catch_unwind`, a panic becomes an `Event` and the session drops to `Lost`.

## 9. Real-time and deployment

- Linux cell PC: a dedicated NIC on the RSI subnet, `SCHED_FIFO` for `kuka-rsi-rt`
  (needs `CAP_SYS_NICE` or an `rtprio` limit; the driver falls back with a warning),
  optional `mlockall` behind a feature. Expect sub-100 µs reply latency; the manual's
  "real-time capable operating system" requirement is met well enough by a dedicated
  thread on a lightly loaded machine, as both reference drivers show.
- macOS: development and simulation only; no priority elevation, occasional late replies
  are expected and reported.
- Reply path budget (measured by the soak test): receive and drain, parse (< 30 µs on
  a 600-byte telegram), control (< 10 µs), write (< 20 µs), send.
- No allocation, locking, logging or Python in the reply path. The only shared
  structures are the bounded command queue, the triple buffer, and the two rings.
- Python garbage collection and the GIL never touch the reply path; the Python API is
  free to be slow.

## 10. Testing

- **Unit** (`cargo test`): schema compilation from the shipped XML and the manual's
  examples; parse and write round trips, attribute order independence, missing and
  unknown elements, XML escaping of `EStr`; interpolator (segments, replace, override
  scaling, starvation); limiter (position, velocity, acceleration, correction, counters);
  session (start capture, restart, fault freeze, stop pulse). Property tests with
  `proptest` for the number formatting and parsing.
- **Integration** (`tests/`, loopback with `ControllerSim`): handshake and `q_start`
  capture; a 5 s stream tracked within tolerance with the sim's lag model; late replies
  increment `Delay` and `HOLDON` holds; `Timeout` exceeded ends the session and the
  driver returns to `Listening` and reconnects; `MoveStop` ends the session cleanly;
  disturbances (jitter, drops, a 200 ms stall) leave the driver answering the newest
  packet.
- **Soak** (`cargo test --release -- --ignored`): 60 s at 250 Hz, asserts zero late
  replies and reports the latency histogram; run manually and on the cell PC.
- **Python** (`pytest` in `kuka-rsi-py/tests`): the same scenarios through the
  bindings with `kuka_rsi.sim.ControllerSim` in-process; numpy shapes and dtypes;
  exceptions; the context manager's cleanup on error.
- **Hardware bring-up**, in order, each step its own session: (1) monitor config,
  `ONLYSEND`, read state for a minute, capture datagrams into `tests/fixtures/`;
  (2) full config, `RSI_MOVECORR` with `hold()` only, check `Delay` stays 0 and `AK`
  is 0; (3) `move_to` a 2° step on A6 in T1 at low override, verify tracking, clamps and
  the override scaling; (4) `stream` a recorded path at 10 Hz;
  (5) `MoveStop`, `fault`, unplug-the-cable and restart-the-program recovery.

## 11. Repository layout and tooling

```
kuka-rsi/
├── Cargo.toml                  # workspace: crates/*, shared dependency versions, edition 2024
├── README.md                   # what it is, wiring, wire format table, bring-up checklist
├── controller/                 # section 6
├── crates/
│   ├── kuka-rsi/               # the library
│   │   ├── src/{lib,config,link,driver,state,stats,error}.rs
│   │   ├── src/telegram/{schema,parse,write}.rs
│   │   ├── src/control/{command,interpolator,limiter,session}.rs
│   │   ├── src/sim/mod.rs
│   │   ├── src/bin/rsi-sim.rs       # run the fake controller from the CLI
│   │   ├── src/bin/rsi-monitor.rs   # print state and stats (ONLYSEND or full)
│   │   ├── src/bin/rsi-files.rs     # write the controller files into a folder
│   │   ├── examples/{jog,stream_trajectory}.rs
│   │   └── tests/                   # integration and soak
│   └── kuka-rsi-py/
│       ├── Cargo.toml               # cdylib, pyo3 0.29, numpy 0.29, pyo3-log
│       ├── pyproject.toml           # maturin; distribution kuka-rsi, module kuka_rsi
│       ├── src/lib.rs
│       ├── python/kuka_rsi/{__init__.py, _core.pyi, py.typed, sim.py}
│       └── tests/
└── docs/                       # nothing yet; protocol notes live in README.md
```

- Dependencies: `quick-xml` (with `serialize`), `serde`, `thiserror`, `socket2`,
  `crossbeam` (queue), `triple_buffer`, `rtrb`, `arrayvec`, `log`, `thread-priority`;
  dev: `proptest`, `env_logger`. Python side: `pyo3`, `numpy`, `pyo3-log`.
- Toolchain: stable Rust 1.98 (installed), MSRV 1.86 (quick-xml 0.42). Python 3.14 with
  uv 0.12 and maturin 1.15 (available through `uvx`).

## 12. Decisions

1. **Joint-space absolute corrections** (`RSI_ON(#ABSOLUTE)` + `AXISCORR`), not
   Cartesian relative corrections as in KUKA's own example. The policy emits joint
   targets and absolute mode cannot drift or double-apply a repeated packet.
2. **SI units in the API** (radians, metres, seconds), degrees and millimetres only on
   the wire. Robot-learning tools generally use radians.
3. **The driver reads the controller's Ethernet XML** as its telegram definition, rather
   than a hard-coded schema. One file to keep in sync, and extra channels need no Rust change.
4. **Playback follows the pendant override and pauses on a program stop.** At 50 %
   override a chunk plays at half speed. The alternative, ignoring the override, makes
   T1 testing at low override meaningless.
5. **A `STOP` object and `MoveStop` channel** for clean shutdown. It is one more object
   in the context (rsi-pi verified it on a KR C4, not yet on a KR C5); without it,
   ending a session always means letting the robot time out with an error message.
6. **The RSI link gets its own subnet** rather than reusing the EKI addresses. The
   manual and both reference projects require a separate "sensor network" interface;
   the addresses in the shipped config are placeholders until you pick the subnet.
7. **Layout**: a Cargo workspace with two crates, the Python package living in the
   bindings crate. Alternative: a single crate
   with a `python` feature, which mixes a `cdylib` into the library and complicates
   `cargo test`.

Assumptions: RSI 5.0 is (or will be) installed on the KR C5 micro; the cell PC that
runs the driver is Linux; the KR10 machine data will be read from the pendant during
bring-up to fill in the limits.

## 13. Prepared for, not built

- **Cartesian mode** (`POSCORR`, `RKorr`): the schema and writer already handle `RKorr`;
  a `Cartesian` session mode and a `PosCorr` object in a second context are the missing
  parts.
- **Digital I/O**: a `DiO` LONG receive element mapped through `Map2DigOut` to a tool
  output, and `DigIn` for its feedback.
- **External axes** (`EK`, `AXISCORREXT`): schema keywords exist; not wired.
- **Physics simulator backend**: a Python object with the same `Driver` surface, so
  client code is identical in simulation and on the robot.
- **`RSI_SETPUBLICPAR` in KRL** to set `AxisCorr` limits from the actual start pose so
  the robot-side clamps equal the true joint limits instead of the full axis range.

## 14. Sources

- KST RSI 5.0 V3 manual: sections 2.2.3, 2.4, 4.1, 6.2-6.4, 7.1, 7.5, 9.3, 10.2, 11.2, 12.2.
- FZI `kuka_rsi_driver` at `c47d86f` (2025-11-19): `krl/KR_C5/*`, `krl/Common/ros_rsi_ethernet.xml`,
  `src/{control_thread,control_buffer,phase_estimator,rsi_parser,rsi_writer,udp_server,hardware_interface}.cpp`,
  `kuka_rsi_driver/simulator.py`.
- `rsi-pi` (RSIPI 0.3.0 at `d31dc1d`): `docs/hardware-findings.md`, `docs/rsi-objects.md`,
  `docs/controller-setup.md`, `src/RSIPI/{network_handler,xml_handler,rsi_echo_server,motion_api}.py`,
  `controller/Program/RSIPI_Minimal.src`, `src/RSIPI/contexts/*`.

## 15. Amendments from execution (2026-09-24)

Rulings made while executing the plan that change what the sections above say. The
approved text is left as written; where the two disagree, this section wins.

- **§6, `AxisCorr` limits.** `AK` is a correction relative to the start pose, so limits
  "equal to the axis ranges" would clamp silently from any start pose off centre. The
  shipped `.rsix` files set `AxisCorr` and `AxisCorrMon` to ± the range width of each
  axis (A1 ±340°, A2 ±235°, A3 ±276°, A4 ±370°, A5 ±240°, A6 ±700°). The driver enforces
  the true joint limits; the robot-side clamp is a backstop that never binds. The
  `RSI_SETPUBLICPAR` item in §13 stands as the way to tighten it later.
- **§5.7, late counting.** The simulator counts late packets as a running total only.
  There is no consecutive-count option.
- **§5.1, config validation.** `EthernetConfig::validate` also requires `DEF_AIPos` and
  `DEF_ASPos` in `SEND` and `HOLDON="1"` on `AK.A1..A6`, unless `ONLYSEND` is true. The
  real-time loop refuses to start a session from a first telegram that lacks `AIPos` or
  `ASPos` (counted as a parse error, no reply).
- **§5.4 and §5.5, faults.** After `fault()` or an `Error:` message the session keeps
  sending the last correction and rejects `stream`, `move_to`, `hold` and `notify` with
  `Error::Faulted` until the KRL program restarts the session. `Driver::is_faulted` and
  `wait_idle` expose it. `speed_scale` survives a session restart.
- **§5.8, Python.** A `Faulted(Error)` exception maps `Error::Faulted`. Negative or
  non-finite durations raise `ValueError`. After `close()` every `Driver` method raises
  `Error("driver is closed")`; keeping the read accessors usable after close, as in
  Rust, is an open follow-up. Dropping a `Driver` without `close()` closes it with the
  GIL released.
- **§5.6, not built.** The p99 inter-arrival histogram, an aggregated
  `Event::ProtocolError`, `Driver::listen`, `robot_wait_timeout`, a `LogReader` type, the
  stall test and `mlockall` were not implemented. A panic in the real-time loop while
  `Active` restarts the session as `Lost`, not as a fault.
