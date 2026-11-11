"""Stage the controller files into a folder to copy onto a USB stick.

Usage (from the repository root, no install needed):
    python3 controller/stage_usb.py OUT_DIR

OUT_DIR gets the same tree as `python -m kuka_rsi.controller OUT_DIR` (kuka_rsi::controller::
write_files), one folder per target on the robot, so the pendant can mark all files in a folder
and copy them in one go:

  SensorInterface/  the Ethernet configs and RSI contexts, for
                    C:\\KRC\\ROBOTER\\Config\\User\\Common\\SensorInterface\\
  Program/          the KRL programs with CRLF line endings, for KRC:\\R1\\Program\\
  LICENSE-*         stay on the stick

macOS AppleDouble files ("._name") that the copy leaves on FAT/exFAT sticks are removed.
"""

import argparse
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
CRATE = HERE.parent
SENSOR_FILES = ["kuka_rsi_ethernet.xml", "kuka_rsi_monitor.xml", "kuka_rsi.rsix", "kuka_rsi_monitor.rsix"]
PROGRAMS = ["kuka_rsi.src", "kuka_rsi_12ms.src", "kuka_rsi_monitor.src"]
LICENSES = [HERE / "LICENSE-FZI", CRATE / "LICENSE-MIT", CRATE / "LICENSE-APACHE"]
SENSOR_INTERFACE_DIR = "C:\\KRC\\ROBOTER\\Config\\User\\Common\\SensorInterface\\"
PROGRAM_DIR = "KRC:\\R1\\Program\\"


def crlf(text: str) -> str:
    return text.replace("\r\n", "\n").replace("\n", "\r\n")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_dir", type=pathlib.Path)
    a = ap.parse_args()

    # Folders this run creates, the output folder included: their AppleDouble files land in the
    # folder above, outside out_dir.
    created = [d for d in [a.out_dir, *a.out_dir.parents] if not d.exists()]
    sensor = a.out_dir / "SensorInterface"
    program = a.out_dir / "Program"
    for d in (sensor, program):
        d.mkdir(parents=True, exist_ok=True)

    written = []  # (path, where it goes on the robot)
    for name in SENSOR_FILES:
        (sensor / name).write_bytes((HERE / name).read_bytes())
        written.append((sensor / name, SENSOR_INTERFACE_DIR))
    for name in PROGRAMS:
        (program / name).write_bytes(crlf((HERE / name).read_text(encoding="utf-8")).encode("utf-8"))
        written.append((program / name, PROGRAM_DIR))
    for lic in LICENSES:
        (a.out_dir / lic.name).write_bytes(lic.read_bytes())
        written.append((a.out_dir / lic.name, "(keep on the PC)"))

    # macOS writes AppleDouble "._name" files next to each file and folder on FAT/exFAT sticks;
    # on the controller they would show up as broken programs and configs.
    for p in a.out_dir.rglob("._*"):
        p.unlink()
    for d in [a.out_dir, *created]:
        d.parent.joinpath("._" + d.name).unlink(missing_ok=True)

    for path, target in written:
        print(f"{path}  ->  {target}")


if __name__ == "__main__":
    main()
