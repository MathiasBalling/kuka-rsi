"""The robot-side files (RSI contexts, Ethernet configs, KRL programs) bundled with the package.

`write_files(dir)`, or `python -m kuka_rsi.controller DIR`, puts them in a folder, for example to
copy onto a USB stick for the controller, one subfolder per target on the robot so the pendant can
copy a whole folder at once:

  DIR/SensorInterface/  Ethernet configs and RSI contexts -> C:\\KRC\\ROBOTER\\Config\\User\\Common\\SensorInterface\\
  DIR/Program/          KRL programs (CRLF)               -> KRC:\\R1\\Program\\
  DIR/LICENSE-*         stay on the PC

`controller/README.md` in the repository says how to set them up.
"""

from __future__ import annotations

import argparse
import os
import pathlib
from typing import NamedTuple

from kuka_rsi import _core


class ControllerFile(NamedTuple):
    name: str
    contents: str
    robot_dir: str | None
    """Target folder on the controller; None for files that stay on the PC (the licenses)."""


def files() -> list[ControllerFile]:
    """Every bundled file, Ethernet configs first, then RSI contexts, KRL programs, licenses."""
    return [ControllerFile(*f) for f in _core.controller_files()]


def write_files(directory: str | os.PathLike[str]) -> list[pathlib.Path]:
    """Write every bundled file into `directory` (created if missing), in the subfolders shown
    above, overwriting files of the same name, and return the written paths in the order of
    `files()`. The KRL programs get CRLF line endings, as the controller's own editor writes them;
    the other files are written as shipped. macOS AppleDouble files ("._name") on FAT/exFAT
    sticks are removed for everything written."""
    return _core.write_controller_files(directory)


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(prog="python -m kuka_rsi.controller", description=__doc__)
    ap.add_argument("directory", help="folder to write the files into (created if missing)")
    for path, f in zip(write_files(ap.parse_args(argv).directory), files()):
        print(f"{path}  ->  {f.robot_dir or '(keep on the PC)'}")


__all__ = ["ControllerFile", "files", "write_files"]

if __name__ == "__main__":
    main()
