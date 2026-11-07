# First bring-up on the real robot

Step by step, from a controller with nothing installed to a 2° move driven from the PC.
Background and the later checklist steps: `README.md` in this folder.

Setup assumed here: KR C5 micro, KSS 8.7, RSI 5.0, a Mac on the KLI port that already talks
Ethernet KRL (EKI) to the controller on `172.31.1.x`.

**Safety.** The driver has been tested on one KR10 R1100-2 with a KR C5 micro (notes and results:
`docs/sessions/` in the repository), and the `.rsix` files have not been opened in RSIVisual. Do
every step in T1, hand on the enabling switch, someone at the E-stop.

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
the PC's address on it before staging. The tools have `kuka_rsi_ethernet.xml` built in, so
rebuild them after the edit (or pass `--config controller/kuka_rsi_ethernet.xml`).

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
target/release/rsi-doctor --robot 192.168.10.147
```

Without `--config` the tools use their built-in copy of `kuka_rsi_ethernet.xml`, the file
staged for the robot; the doctor's config line says which one it checked.

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
target/release/rsi-monitor --wait 300 --seconds 60 | tee ~/rsi_test2.log
```

Then start `kuka_rsi` (in T1, hold the enabling switch and the Start key). At `RSI_ON` the
pendant shows "Caution – sensor correction is activated!!!": acknowledge it and press Start
again. The safety controller raises it at every start in T1 and T2 (RSI 5.0 manual, section 3.1);
it cannot be switched off and does not appear in AUT or AUT EXT.

Pass: the robot does not move, `delay` stays 0, after 60 s the program ends without a message.
A rising `delay` means the Mac answers late; the robot stops with an RSI communication error
(safe), and a Linux PC with a real-time setup would be the next step.

## 7. Test 3: move A6

T1, the tool cable free to follow A6. PC first:

```bash
target/release/examples/jog --deg 45 --secs 10
```

Then start `kuka_rsi`. A6 turns `--deg` (default 2, at most 45) over `--secs` and back;
playback follows the pendant override, so 10 % takes ten times as long. Pass: the printed
measured A6 angle reaches start + `--deg` and returns, the stats show no clamps and no late
replies, the program ends cleanly. 2° is hard to see on the robot; 45° over 10 s is clear.

After `MoveStop` the program runs `RSI_OFF` and `RSI_DELETE` and waits at the final
`PTP $AXIS_ACT` for Start; that is the normal end. After any stop with a message: acknowledge,
then Edit > Reset program before the next start.

Next: bigger moves, streaming and the fault paths. The test plans, notes and results of each
session on the robot are in `docs/sessions/` in the repository.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| `rsi-monitor` fails to start (bind error) | The Mac has no `192.168.10.10` (step 3); `rsi-doctor` says which part is missing |
| No telegrams, robot stops with an RSI communication error | Cable on a KLI port; the RSI interface exists after the cold start; same subnet both ends; macOS firewall. A failed ping to `.147` proves nothing |
| Program halts at `RSI_CREATE` | Read the message window. A file read or parse error: check the file names and that no `._*` files were copied with them. Otherwise a parameter in the `.rsix`: open it in RSIVisual |
| Program halts at `RSI_ON` | Message window; often the network or the config XML name |
| `delay` climbs | The PC answers later than 4 ms; close other load, then consider a Linux PC |
| Every reply late from the first telegram, `delay` +1 per cycle | Ping the controller, then start again. Seen once after the link sat idle for about 20 minutes, accepted right after a ping; likely the Mac's ARP entry had expired (not proven yet) |
| `KSS00489 Stop by $CORRECTION function` | RSI stopped the correction, e.g. after the ETHERNET timeout; not by itself a bad correction. Acknowledge, then Edit > Reset program, or the next start resumes at `RSI_MOVECORR()` without RSI and stops at once |
| The robot stops answering pings mid-session | The USB Ethernet adapter's link dropped (`ifconfig` shows `status: inactive`); reseat the adapter and cable |
| "Caution – sensor correction is activated!!!" at every start | Normal in T1 and T2; acknowledge and press Start again |
