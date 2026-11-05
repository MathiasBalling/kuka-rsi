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

The build puts `rsi-monitor` in `target/release/` and `jog` and `stream_trajectory` in
`target/release/examples/`. The USB folder holds:

| Folder | Goes to | Contents |
| --- | --- | --- |
| `SensorInterface/` | `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\` | 2 × `.xml`, 2 × `.rsix`, as in the repo |
| `SensorInterface_crlf_fallback/` | same, only if the first set is rejected | same files, CRLF line endings and a UTF-8 BOM |
| `Program/` | `KRC:\R1\Program\` | `kuka_rsi.src`, `kuka_rsi_monitor.src` |

In the staged `kuka_rsi.src` the fixed start pose is replaced with `PTP $AXIS_ACT`: the
program starts and parks where the robot stands, with no PTP move. Pass `--keep-start-pose` to
keep the pose from the repo.

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

## 4. Copy the files to the controller

- `SensorInterface/*`: this folder is outside the Navigator's normal view. Either WorkVisual
  (Windows) or, as Administrator, Start-up > Service > Minimize HMI and Windows Explorer from the
  USB drive (`E:\` on the cabinet, `K:\` on the smartPAD).
- `Program/*.src`: Navigator as Expert, copy from the USB drive, paste into `KRC:\R1\Program\`.

Never put both `SensorInterface` sets in the target folder; the names are the same.

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

Next: `stream_trajectory` (±3° sine on A6), then checklist steps 4 to 6 in `README.md`.

## Notes from the first bring-up (2026-11-05)

- Ping the controller (`ping 192.168.10.147`) right before a closed-loop run. The first test 2
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

## Troubleshooting

| Symptom | Check |
| --- | --- |
| `rsi-monitor` fails to start (bind error) | The Mac has no `192.168.10.10` (step 3) |
| No telegrams, robot stops with an RSI communication error | Cable on a KLI port; the RSI interface exists after the cold start; same subnet both ends; macOS firewall. A failed ping to `.147` proves nothing |
| Program halts at `RSI_CREATE` | Read the message window. A file read or parse error: swap in `SensorInterface_crlf_fallback/`. Otherwise a parameter in the `.rsix`: open it in RSIVisual |
| Program halts at `RSI_ON` | Message window; often the network or the config XML name |
| `delay` climbs | The PC answers later than 4 ms; close other load, then consider a Linux PC |
