# Controller side of the kuka-rsi driver (KR C5 micro, KSS 8.7, RSI 5.0)

First bring-up, step by step (network, USB staging, first three tests): `BRINGUP.md`.

## Files and where they go

| File | Robot path | Purpose |
| --- | --- | --- |
| `kuka_rsi_ethernet.xml` | `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\` | Telegram layout and the PC address. The driver reads the same file |
| `kuka_rsi_monitor.xml` | same | Same layout with `ONLYSEND` TRUE, for the read-only first step |
| `kuka_rsi.rsix` | same | RSI context: Ethernet, AxisCorr, AxisCorrMon, Stop, GearTorque, Status, OV_PRO |
| `kuka_rsi_monitor.rsix` | same | Same context pointing at the monitor config |
| `kuka_rsi.src` | `KRC:\R1\Program\` | `PTP $AXIS_ACT` (no move), `RSI_ON(#ABSOLUTE)`, `RSI_MOVECORR()` |
| `kuka_rsi_12ms.src` | `KRC:\R1\Program\` | `kuka_rsi` in the 12 ms sensor mode, `RSI_ON(#ABSOLUTE, #IPO, #BASE)`, same context; run the PC tools with `--cycle-ms 12`. Not yet confirmed on a robot |
| `kuka_rsi_monitor.src` | `KRC:\R1\Program\` | `RSI_ON`, `WAIT SEC 60`, no motion |

Copy them with WorkVisual (drag into `SensorInterface` and `Program` in the project
structure, deploy) or with a USB stick: `python -m kuka_rsi.controller DIR` or
`python3 controller/stage_usb.py DIR` writes them into `DIR/SensorInterface/` and
`DIR/Program/`, one folder per target. `RSI_CREATE` takes the context name without the
`.rsix` extension.

Open both `.rsix` files once in RSIVisual (WorkVisual 6.0 + RSI 5.0 option) and save
them, so the tool regenerates the layout and validates every parameter. They were
derived by hand from FZI's `kuka_rsi_driver` KR C5 context (BSD-3-Clause,
https://github.com/fzi-forschungszentrum-informatik/kuka_rsi_driver).

## Network

RSI needs its own sensor network, separate from the EKI addresses (`172.31.1.x`):

1. On the pendant: Start-up > Network configuration > KLI tab > Advanced... > Add
   interface. Name it `RSI`, address type **Mixed IP address**, controller IP on a free
   subnet, e.g. `192.168.10.147/24`. `192.168.0.x` and the existing KLI subnet are
   blocked by the controller.
2. Save, then reboot with Cold start and Reload files.
3. Give the PC's NIC an address on the same subnet (`192.168.10.10/24`, or an alias on
   the NIC already used for EKI) and put that address in `IP_NUMBER` of both XML files.
4. Check with the monitor step below before any motion.

## Start order

1. PC: start the driver (`rsi-monitor`, an example, or your Python script). It binds
   `IP_NUMBER:PORT` and waits.
2. Pendant: select and start the program. After the first `PTP`, `RSI_ON` sends the first
   telegram; the driver answers within 4 ms from then on.
3. To end: `Driver.close()` pulses `MoveStop`, the program leaves `RSI_MOVECORR()`,
   runs `RSI_OFF` and parks. Stopping the driver without that makes the robot stop with
   an RSI communication error after 0.4 s, which is safe but leaves a message to
   acknowledge.

## Bring-up checklist

1. `kuka_rsi_monitor.src` with `rsi-monitor --config controller/kuka_rsi_monitor.xml`:
   telegrams at 250 Hz, `Delay` 0, joint angles match the pendant. Save a few raw
   telegrams into `crates/kuka-rsi/tests/fixtures/` for the parser tests.
2. `kuka_rsi.src` in T1 with `rsi-monitor` (built-in `kuka_rsi_ethernet.xml`):
   the robot holds, `AK` stays 0, `Delay` stays 0 for a minute, `close()` ends the
   program cleanly.
3. `cargo run --example jog` in T1 at 10 % override:
   A6 moves 2° and back; tracking error and clamp counters stay quiet; playback slows
   with the override.
4. `cargo run --example stream_trajectory -- ...`, then a recorded path
   through the Python `Driver.stream`.
5. Fault paths: `Driver.fault()`, cable unplugged, program restarted while the driver
   keeps running (it must reconnect and re-capture the start pose).
6. Confirm what the driver and the simulator assume about the robot (with
   `rsi-monitor` and the pendant's RSI monitor):
   - `ProgStatus` reads 3 while the program runs and 4 when it is stopped (the driver
     pauses playback on anything but 3);
   - `IPOC` advances by 4 per telegram (the driver counts any other step as a gap);
   - whether the ETHERNET `Timeout` counts late packets as a running total or only
     consecutive ones (the simulator assumes a running total, like the `Delay` value).

The shipped `.rsix` files are LF-terminated UTF-8 without a BOM. If WorkVisual refuses
them, convert them to CRLF with a BOM (e.g. `unix2dos -m`) before importing.

Read the machine data (`$SOFTN_END`, `$SOFTP_END`, axis speeds) from the pendant and
put them into `Limits` before step 3. The driver enforces the true joint limits
(`position_min/max`) on every target and every cycle. `AxisCorr` and `AxisCorrMon` bound
something else: the correction relative to the start pose (`AK = q - q_start`), not the
joint angle. They are therefore set to the axis range widths (`AxisCorr` ±340°, ±235°,
±276°, ±370°, ±240°, ±700° for A1 to A6; `AxisCorrMon` the same widths), so they never
bind before the driver's own joint limits, whatever the start pose. If the machine data
differs from the data sheet, set them to the new widths (`max - min`) and keep
`Limits.max_correction` equal to them.
