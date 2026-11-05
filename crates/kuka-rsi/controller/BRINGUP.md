# First bring-up on the real robot

Step by step, from a controller with nothing installed to a 2° move driven from the PC.
Background and the later checklist steps: `README.md` in this folder.

Setup assumed here: KR C5 micro, KSS 8.7, RSI 5.0, a Mac on the KLI port that already talks
Ethernet KRL (EKI) to the controller on `172.31.1.x`.

**Safety.** The driver has run only against the simulator so far, and the `.rsix` files have
not been opened in RSIVisual. Do every step in T1, hand on the enabling switch, someone at the
E-stop.

## 0. Before leaving the desk

```bash
cargo build -p kuka-rsi --features cli --release --bins --examples
python3 controller/stage_usb.py /Volumes/<STICK>/kuka_rsi_usb
```

The build puts `rsi-doctor` and `rsi-monitor` in `target/release/` and `jog` and `stream_trajectory` in
`target/release/examples/`. `stage_usb.py` writes the same tree as `python -m kuka_rsi.controller`
and removes the macOS `._*` files from the stick. The USB folder holds:

| Folder | Goes to | Contents |
| --- | --- | --- |
| `SensorInterface/` | `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\` | 2 × `.xml`, 2 × `.rsix`, as in the repo |
| `Program/` | `KRC:\R1\Program\` | `kuka_rsi.src`, `kuka_rsi_monitor.src`, CRLF line endings |
| (top) | stays on the stick | `LICENSE-FZI`, `LICENSE-MIT`, `LICENSE-APACHE` |

`kuka_rsi.src` starts and parks with `PTP $AXIS_ACT`: block coincidence where the robot
stands, no PTP move. Replace both lines with a taught pose if the program should move there first.

## 1. Pendant checks (T1, user group Expert)

- RSI installed: Main menu > Help > Info > Options lists "KUKA.RobotSensorInterface 5.0". If
  not, it must be installed first (KOP file from KUKA, Start-up > Additional software).
- Leave the existing KLI interface (`172.31.1.147`, Mixed IP, used by EKI) as it is.

## 2. Controller: a separate RSI interface

RSI gets its own subnet on the same KLI port and cable as EKI.

1. Start-up > Network configuration > KLI tab > Advanced... > Add interface.
2. Name `RSI`, address type **Mixed IP address**, IP `192.168.10.147`, mask `255.255.255.0`.
   The receiving task and the real-time receiving task (UDP) are created automatically.
3. Save, reboot with **Cold start** and **Reload files**. Copying the files (step 4) can wait for
   the reboot.

If `192.168.10.x` is taken, use another free subnet and change `IP_NUMBER` in both XML files to
the PC's address on it before staging.

## 3. Mac: two static addresses on the robot adapter

The robot sends its telegrams to `IP_NUMBER` in the XML (`192.168.10.10`) through the RSI
interface, and the driver binds that address, so the Mac needs it. EKI needs `172.31.1.77`
on the same cable. Give the USB LAN adapter one network service per address:

1. System Settings > Network > the USB LAN adapter (`networksetup -listnetworkserviceorder`
   shows which service is the `en` device on the robot cable) > Details > TCP/IP: Configure
   IPv4 **Manually**, `172.31.1.77`, mask `255.255.255.0`, router empty.
2. Back in the Network list: "..." > Duplicate Service on that adapter, name it `Robot RSI`,
   TCP/IP Manually `192.168.10.10`, mask `255.255.255.0`, router empty.
3. Check: `ifconfig en7 | grep 'inet '` shows both addresses. A `169.254.x.x` address alone
   means the adapter is still on DHCP and got no static address.

Quick alternative for one session: `sudo ifconfig en7 alias 192.168.10.10 255.255.255.0`
(gone after unplugging or a restart, and possibly after the link drops while the controller
reboots).

The macOS firewall must allow incoming UDP for the binaries (or be off).

Check the PC side: the config, the address and port, the route and a ping to the robot,
thread priority and the firewall:

```bash
target/release/rsi-doctor --config controller/kuka_rsi_ethernet.xml --robot 192.168.10.147
```

Each line reads PASS, WARN (works, but may bite later) or FAIL, with what to do. With the files
on the controller (step 4), `--wait 60` adds the live checks: start `kuka_rsi` when the doctor
says "listening"; it holds the robot where it stands for 3 s, reports the telegram rate, late
replies and the robot's `Delay`, then ends the program with `MoveStop`. Without a Rust build:
`python -m kuka_rsi.doctor` with the same options.

## 4. Copy the files to the controller

- `SensorInterface/*`: this folder is outside the Navigator's normal view. Either WorkVisual
  (Windows) or, as Administrator, Start-up > Service > Minimize HMI and Windows Explorer from the
  USB drive (`E:\` on the cabinet, `K:\` on the smartPAD).
- `Program/*.src`: Navigator as Expert, open `Program` on the USB drive, Edit > Mark all,
  Edit > Copy, paste into `KRC:\R1\Program\`.

The licenses stay on the stick.

## 5. Test 1: connect and read joints (no motion)

PC first, from the repository root:

```bash
target/release/rsi-monitor --config controller/kuka_rsi_monitor.xml --wait 300 | tee ~/rsi_test1.log
```

Then select and start `kuka_rsi_monitor` on the pendant (it runs 60 s, the robot only sends).

Pass: "robot connected", `ipoc` rising steadily, `q(deg)` equal to the pendant's actual axis
values.

## 6. Test 2: hold position (closed loop, zero correction)

T1, low override. PC first:

```bash
target/release/rsi-monitor --config controller/kuka_rsi_ethernet.xml --wait 300 --seconds 60 | tee ~/rsi_test2.log
```

Then start `kuka_rsi` (in T1, hold the enabling switch and the Start key).

Pass: the robot does not move, `delay` stays 0, after 60 s the program ends without a message.
A rising `delay` means the Mac answers late; the robot stops with an RSI communication error
(safe), and a Linux PC with a real-time setup would be the next step.

## 7. Test 3: move A6

T1, the tool cable free to follow A6. PC first:

```bash
target/release/examples/jog controller/kuka_rsi_ethernet.xml --deg 45 --secs 10
```

Then start `kuka_rsi`. A6 turns `--deg` (default 2, at most 45) over `--secs` and back;
playback follows the pendant override, so 10 % takes ten times as long. Pass: the printed
measured A6 angle reaches start + `--deg` and returns, the stats show no clamps and no late
replies, the program ends cleanly. 2° is hard to see on the robot; 45° over 10 s is clear.

Next: the second session below (bigger moves, streaming, fault paths).

## Notes from the first bring-up (2026-11-05)

- Ping the controller (`ping 192.168.10.147`, or run `rsi-doctor --robot 192.168.10.147`)
  right before a closed-loop run. The first test 2
  failed with every reply counted late (robot `Delay` +1 per cycle, stop after 100 cycles,
  pendant `KSS00489 Stop by $CORRECTION function`, originator A2), while the same replies were
  accepted right after a ping. Likely cause: the Mac's ARP entry for the controller had expired
  (20 min), since receiving the robot's telegrams does not refresh it. Not proven.
- `KSS00489 Stop by $CORRECTION function` is how the controller reports RSI stopping the
  correction, e.g. after the ETHERNET timeout; it does not by itself mean a bad correction value.
  Confirm, then Edit > Reset program (block selection), or the next start resumes at
  `RSI_MOVECORR()` without RSI and stops at once.
- After `MoveStop` the program runs `RSI_OFF`, `RSI_DELETE` and waits at the final
  `PTP $AXIS_ACT` for Start; that is the normal end.
- The robot's `Delay` went 0, 1, 0: it counts current late packets, not a running total.
- The USB Ethernet link dropped once mid-session (`ifconfig en7` showed `status: inactive`, then
  ARP `incomplete`); reseat the adapter and cable if the robot stops answering pings.

Results (Mac, macOS, USB 1 Gbit adapter, Rust driver):

| Test | Result |
| --- | --- |
| 1 monitor, 60 s | 14,993 telegrams at 250 Hz, IPOC +4 per telegram, joints equal to the pendant, 7 late reads by the Mac (max 10 ms) |
| 2 hold, 60 s | 14,996 replies, 1 late (6.8 ms), mean latency 0.13 ms, robot `Delay` 0, clean `MoveStop` end |
| 3 A6 +45° / 10 s and back | measured A6 −286.847 → −241.846 → −286.849°, 5,152 replies, 0 late, max latency 0.96 ms, no clamps |

## Second session: bigger moves, streaming, fault paths

Goal: confirm the driver on the moves and failures a real application will see before the
first release. Every step in T1, hand on the enabling switch, someone at the E-stop. Offsets
stay within 30° per axis (A6 45°) around the pose the robot starts in; check the cell around
that pose first. After any stop on the pendant: acknowledge, then Edit > Reset program before
the next start.

At the desk:

```bash
cargo build -p kuka-rsi --features cli --release --bins --examples
(cd crates/kuka-rsi-py && uv sync)          # Python steps run with: uv run --project crates/kuka-rsi-py python ...
python3 controller/stage_usb.py /Volumes/<STICK>/kuka_rsi_usb
```

`--log runs/NAME.csv` on `rsi-monitor`, `jog` and `stream_trajectory` writes every cycle
(commanded, actual, lag, `AK`, TCP, torques; header in `kuka_rsi::record`). Keep the CSV files of
every step; they are the evidence for the results table.

### S1. New files, then the known-good tests

1. Copy the new files (step 4). On the controller, delete the program and config files from the
   first session if their names differ from the ones on the stick, so only one set is there.
2. Test 1 (`rsi-monitor` with `kuka_rsi_monitor`), as in step 5.
3. `target/release/rsi-doctor --robot 192.168.10.147 --wait 60`, start `kuka_rsi` when it says
   "listening".

Pass: test 1 as before; every doctor check PASS (ping may WARN), about 250 telegrams/s, no late
replies, the program ends cleanly.

### S2. One axis at a time

For each axis N from 1 to 6, at 10 % override, then at 50 %:

```bash
target/release/examples/jog controller/kuka_rsi_ethernet.xml --axis N --deg 10 --log runs/s2_aN_10.csv
target/release/examples/jog controller/kuka_rsi_ethernet.xml --axis N --deg 30 --log runs/s2_aN_30.csv
```

Start `kuka_rsi` after each command. Try +10° first; if the cell needs the other direction, use a
negative `--deg`. Pass for each leg: measured equals target within 0.1°, 0 late replies, no
clamps, robot `Delay` max 0 or 1, clean end. Note the max lag per axis.

### S3. Several axes at once

```bash
target/release/examples/jog controller/kuka_rsi_ethernet.xml \
    --move A1=30,A2=-10,A3=15,A5=20,A6=45 --repeat 3 --log runs/s3_multi.csv
```

Pass: all axes start and stop together (the move is synchronised to the slowest axis), each leg
of all three repeats ends on its target within 0.1°, the last leg ends where the robot started,
0 late replies, no clamps.

### S4. Speed steps

A6 first (short lever), then A1 with the arm folded:

```bash
for v in 10 20 30; do
  target/release/examples/jog controller/kuka_rsi_ethernet.xml --axis 6 --deg 30 --speed $v --log runs/s4_a6_v$v.csv
done
```

Start `kuka_rsi` for each run, at 100 % override, still in T1. T1 limits the TCP speed to
250 mm/s; A1 at 30°/s exceeds that about 0.5 m from the axis. If the controller stops with a T1
velocity message, note the speed and stay below it; that is the controller's limit, not the
driver's. Pass: targets reached as in S2, no velocity or acceleration clamps. Note the max lag
at each speed: `max_tracking_error` (default 2°) should sit clearly above the lag at 30°/s.

### S5. Streaming

Rust, then Python, each once to the end and once stopped early:

```bash
target/release/examples/stream_trajectory controller/kuka_rsi_ethernet.xml --log runs/s5_rust.csv
target/release/examples/stream_trajectory controller/kuka_rsi_ethernet.xml --stop-after 4 --log runs/s5_rust_stop.csv
uv run --project crates/kuka-rsi-py python crates/kuka-rsi-py/examples/stream_path.py \
    controller/kuka_rsi_ethernet.xml --log runs/s5_py.csv
uv run --project crates/kuka-rsi-py python crates/kuka-rsi-py/examples/stream_path.py \
    controller/kuka_rsi_ethernet.xml --stop-after 4 --log runs/s5_py_stop.csv
```

Defaults: all six axes ±3°, 4 s period, 8 s, new points every 100 ms. Pass: smooth motion, no
jerk at the 100 ms chunk boundaries, 0 late replies, the stopped runs hold within 0.1° of the last
target sent, then the return to the start pose ends within 0.1°. A few acceleration clamps are
expected (the limiter rounds the corners between 100 ms points); velocity clamps should be 0.
Then `--amp 10 --period 6 --secs 12` once for a larger path.

### S6. Fault paths

1. **`fault()`.** Start this, then `kuka_rsi`:

   ```bash
   uv run --project crates/kuka-rsi-py python -c "
   import time, kuka_rsi
   with kuka_rsi.Driver('controller/kuka_rsi_ethernet.xml', stop_channel='MoveStop') as d:
       d.wait_for_robot(120); time.sleep(3); d.fault('bring-up fault test'); time.sleep(0.1)
       print('faulted:', d.is_faulted()); time.sleep(2); print(d.events())"
   ```

   Pass: `faulted: True`; the robot stops with the message text on the pendant; the events end
   with `Fault`, `LinkLost` (the program stopped sending).
2. **Cable pulled during a hold.**
   `target/release/rsi-monitor --config controller/kuka_rsi_ethernet.xml --wait 300 --keep --log runs/s6_cable.csv`,
   start `kuka_rsi`, pull the cable at the Mac after about 10 s. Pass: the robot stops within
   0.4 s with an RSI communication error, the monitor reports the link lost and waits. Plug the
   cable back in, acknowledge, reset and start the program: the monitor prints "session 2
   connected" with the start pose.
3. **Program restarted while the driver runs.** Same monitor command. Cancel the program on the
   pendant, jog A6 a few degrees by hand, reset and start again. Pass: the new session's start
   pose equals the pendant's axis values, and the robot holds there with no jump.
4. **Ctrl-C mid-move.** `jog --axis 6 --deg 30 --secs 10`, Ctrl-C halfway. Pass: the robot stops
   where it is, the program ends at the final `PTP $AXIS_ACT` without an error message, `jog`
   exits with 130.
5. **Override and enabling switch.** `jog --axis 6 --deg 30 --secs 10 --repeat 2`; change the
   override 10 % → 50 % during the first leg; release the enabling switch during the second, then
   press it and Start again. Expected: playback speeds up with the override without a jerk; on
   release the robot stops, the CSV shows `program_running` 0 and the command holding, and after
   Start the move continues from there. Record what happens; this is new ground.

### S7. Long hold (ARP theory)

Tests the notes from the first session: macOS drops ARP entries after 20 minutes.

```bash
arp -an | grep 192.168.10.147
caffeinate -i target/release/rsi-monitor --config controller/kuka_rsi_ethernet.xml \
    --wait 300 --seconds 1500 --log runs/s7_hold25.csv
arp -an | grep 192.168.10.147
```

Start `kuka_rsi`; do not ping. Pass: 25 minutes with 0 late replies and `Delay` 0, in particular
around minute 20. Then leave the link idle for 25 minutes (no driver, no ping) and run test 2
without pinging first. If replies are late again and a ping fixes it, the theory holds.

### Checklist item 6

From the CSV logs and the pendant's RSI monitor: `program_running` is 1 while the program runs
and 0 when it is stopped (S6.5); the `ipoc` column rises by 4 per row; whether the robot's
`Delay` counts consecutive late packets or a running total (S7, or any run with late replies).

### Results (second session)

| Step | Result |
| --- | --- |
| S1 files, test 1, doctor | |
| S2 single axes, 10° / 30° (max lag per axis) | |
| S3 five axes, 3 repeats | |
| S4 speed 10 / 20 / 30 °/s (max lag, T1 limit) | |
| S5 stream Rust / Python, stop-after | |
| S6.1 `fault()` | |
| S6.2 cable pulled, reconnect | |
| S6.3 program restart | |
| S6.4 Ctrl-C mid-move | |
| S6.5 override, enabling switch | |
| S7 25 min hold, idle start | |
| Item 6: `ProgStatus`, `IPOC`, `Delay` | |

## Troubleshooting

| Symptom | Check |
| --- | --- |
| `rsi-monitor` fails to start (bind error) | The Mac has no `192.168.10.10` (step 3); `rsi-doctor` says which part is missing |
| No telegrams, robot stops with an RSI communication error | Cable on a KLI port; the RSI interface exists after the cold start; same subnet both ends; macOS firewall. A failed ping to `.147` proves nothing |
| Program halts at `RSI_CREATE` | Read the message window. A file read or parse error: check the file names and that no `._*` files were copied with them. Otherwise a parameter in the `.rsix`: open it in RSIVisual |
| Program halts at `RSI_ON` | Message window; often the network or the config XML name |
| `delay` climbs | The PC answers later than 4 ms; close other load, then consider a Linux PC |
