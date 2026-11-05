"""Stage the controller files into a folder to copy onto a USB stick.

Usage (from the repository root):
    python3 controller/stage_usb.py OUT_DIR [--keep-start-pose]

OUT_DIR gets three folders:
  SensorInterface/                as shipped, for C:\\KRC\\ROBOTER\\Config\\User\\Common\\SensorInterface\\
  SensorInterface_crlf_fallback/  the same files with CRLF line endings and a UTF-8 BOM, for when
                                  the controller rejects the first set
  Program/                        the two KRL programs with CRLF line endings, for KRC:\\R1\\Program\\

By default the fixed start/park pose in kuka_rsi.src is replaced with PTP $AXIS_ACT, so the
program reaches block coincidence where the robot already stands and makes no PTP move.
"""

import argparse
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
SENSOR_FILES = ["kuka_rsi_ethernet.xml", "kuka_rsi_monitor.xml", "kuka_rsi.rsix", "kuka_rsi_monitor.rsix"]
PROGRAMS = ["kuka_rsi.src", "kuka_rsi_monitor.src"]
START_POSE = "PTP {A1 0, A2 -90, A3 90, A4 0, A5 90, A6 0}"


def crlf(text: str) -> str:
    return text.replace("\r\n", "\n").replace("\n", "\r\n")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_dir", type=pathlib.Path)
    ap.add_argument("--keep-start-pose", action="store_true", help="keep the fixed PTP pose of kuka_rsi.src")
    a = ap.parse_args()

    # Folders this run creates, the output folder included: their AppleDouble files land in the
    # folder above, outside out_dir.
    created = [d for d in [a.out_dir, *a.out_dir.parents] if not d.exists()]

    sensor = a.out_dir / "SensorInterface"
    fallback = a.out_dir / "SensorInterface_crlf_fallback"
    program = a.out_dir / "Program"
    for d in (sensor, fallback, program):
        d.mkdir(parents=True, exist_ok=True)

    for name in SENSOR_FILES:
        text = (HERE / name).read_text(encoding="utf-8")
        (sensor / name).write_text(text, encoding="utf-8", newline="")
        (fallback / name).write_bytes(b"\xef\xbb\xbf" + crlf(text).encode("utf-8"))

    for name in PROGRAMS:
        text = (HERE / name).read_text(encoding="utf-8")
        if name == "kuka_rsi.src" and not a.keep_start_pose:
            if text.count(START_POSE) != 2:
                raise SystemExit(f"{name}: expected the start pose twice, found {text.count(START_POSE)}")
            text = text.replace(START_POSE, "PTP $AXIS_ACT   ; bring-up: stay where the robot is")
        (program / name).write_bytes(crlf(text).encode("utf-8"))

    # macOS writes AppleDouble "._name" files next to each file and folder on FAT/exFAT sticks;
    # on the controller they would show up as broken programs and configs.
    for p in a.out_dir.rglob("._*"):
        p.unlink()
    for d in [a.out_dir, *created]:
        d.parent.joinpath("._" + d.name).unlink(missing_ok=True)

    for p in sorted(a.out_dir.rglob("*")):
        if p.is_file():
            print(p)


if __name__ == "__main__":
    main()
