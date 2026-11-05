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
cargo build --manifest-path kuka-rsi/Cargo.toml -p kuka-rsi --features cli --release --bins --examples
python3 kuka-rsi/controller/stage_usb.py /Volumes/<STICK>/kuka_rsi_usb
```

The build puts `rsi-monitor` in `kuka-rsi/target/release/` and `jog` and `stream_trajectory` in
`kuka-rsi/target/release/examples/`. The USB folder holds:

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

## 3. Mac: a second address on the EKI adapter

The robot sends its telegrams to `IP_NUMBER` in the XML (`192.168.10.10`) through the RSI
interface, and the driver binds that address, so the Mac needs it. Add it next to the EKI
address instead of replacing it, so EKI keeps working:

```bash
ifconfig | awk '/^[a-z]/{i=$1} /inet /{print i, $2}'    # the adapter with 172.31.1.x, e.g. en7
sudo ifconfig en7 alias 192.168.10.10 255.255.255.0
```

The alias lasts until the adapter is unplugged or the Mac restarts; remove it with
`sudo ifconfig en7 -alias 192.168.10.10`. For a permanent setup: System Settings > Network > the
adapter > "..." > Duplicate Service, manual IP `192.168.10.10` / `255.255.255.0`, no router.

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
kuka-rsi/target/release/rsi-monitor --config kuka-rsi/controller/kuka_rsi_monitor.xml --wait 300 | tee ~/rsi_test1.log
```

Then select and start `kuka_rsi_monitor` on the pendant (it runs 60 s, the robot only sends).

Pass: "robot connected", `ipoc` rising steadily, `q(deg)` equal to the pendant's actual axis
values.

## 6. Test 2: hold position (closed loop, zero correction)

T1, low override. PC first:

```bash
kuka-rsi/target/release/rsi-monitor --config kuka-rsi/controller/kuka_rsi_ethernet.xml --wait 300 --seconds 60 | tee ~/rsi_test2.log
```

Then start `kuka_rsi` (in T1, hold the enabling switch and the Start key).

Pass: the robot does not move, `delay` stays 0, after 60 s the program ends without a message.
A rising `delay` means the Mac answers late; the robot stops with an RSI communication error
(safe), and a Linux PC with a real-time setup would be the next step.

## 7. Test 3: 2° on A6

T1, 10 % override, the tool cable free to follow A6 by 2°. PC first:

```bash
kuka-rsi/target/release/examples/jog kuka-rsi/controller/kuka_rsi_ethernet.xml
```

Then start `kuka_rsi`. Pass: A6 turns 2° over 2 s and back, the stats print, the program ends
cleanly.

Next: `stream_trajectory` (±3° sine on A6), then checklist steps 4 to 6 in `README.md`.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| `rsi-monitor` fails to start (bind error) | The Mac has no `192.168.10.10` (step 3) |
| No telegrams, robot stops with an RSI communication error | Cable on a KLI port; the RSI interface exists after the cold start; same subnet both ends; macOS firewall. A failed ping to `.147` proves nothing |
| Program halts at `RSI_CREATE` | Read the message window. A file read or parse error: swap in `SensorInterface_crlf_fallback/`. Otherwise a parameter in the `.rsix`: open it in RSIVisual |
| Program halts at `RSI_ON` | Message window; often the network or the config XML name |
| `delay` climbs | The PC answers later than 4 ms; close other load, then consider a Linux PC |
