"""The robot-side files (RSI contexts, Ethernet configs, KRL programs) bundled with the package.

`write_files(dir)`, or `python -m kuka_rsi.controller DIR`, puts them in a folder, for example to
copy them onto a USB stick for the controller. `controller/README.md` in the repository says where each one goes and how to set it up.
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
    """Target folder on the controller; None for files that stay on the PC (the license)."""


def files() -> list[ControllerFile]:
    """Every bundled file, Ethernet configs first, then RSI contexts, KRL programs, license."""
    return [ControllerFile(*f) for f in _core.controller_files()]


def write_files(directory: str | os.PathLike[str]) -> list[pathlib.Path]:
    """Write every bundled file into `directory` (created if missing), overwriting files of the
    same name, and return the written paths in the order of `files()`. The KRL programs get CRLF
    line endings, as the controller's own editor writes them; the other files are written as shipped."""
    return _core.write_controller_files(directory)


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(prog="python -m kuka_rsi.controller", description=__doc__)
    ap.add_argument("directory", help="folder to write the files into (created if missing)")
    for path, f in zip(write_files(ap.parse_args(argv).directory), files()):
        print(f"{path}  ->  {f.robot_dir or '(keep on the PC)'}")


__all__ = ["ControllerFile", "files", "write_files"]

if __name__ == "__main__":
    main()
