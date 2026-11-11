import pathlib
import subprocess
import sys

import kuka_rsi
from kuka_rsi import controller

CRATE = pathlib.Path(__file__).resolve().parents[2] / "kuka-rsi"
FOLDER = CRATE / "controller"
NAMES = [
    "kuka_rsi_ethernet.xml",
    "kuka_rsi_monitor.xml",
    "kuka_rsi.rsix",
    "kuka_rsi_monitor.rsix",
    "kuka_rsi.src",
    "kuka_rsi_12ms.src",
    "kuka_rsi_monitor.src",
    "LICENSE-FZI",
    "LICENSE-MIT",
    "LICENSE-APACHE",
]
SENSOR_INTERFACE = "C:\\KRC\\ROBOTER\\Config\\User\\Common\\SensorInterface\\"
PROGRAM = "KRC:\\R1\\Program\\"


def shipped(name):
    """The repository copy: the crate's own licenses sit at the crate root, the rest in controller/."""
    return (CRATE / name if name in ("LICENSE-MIT", "LICENSE-APACHE") else FOLDER / name).read_text(encoding="utf-8")


def local_path(f):
    sub = {SENSOR_INTERFACE: "SensorInterface", PROGRAM: "Program"}.get(f.robot_dir)
    return pathlib.Path(sub, f.name) if sub else pathlib.Path(f.name)


def tree(root):
    return {p.relative_to(root).as_posix(): p.read_bytes() for p in root.rglob("*") if p.is_file()}


def test_bundled_files_match_the_repository():
    files = controller.files()
    assert [f.name for f in files] == NAMES
    for f in files:
        assert f.contents == shipped(f.name)
    dirs = {f.name: f.robot_dir for f in files}
    assert dirs["kuka_rsi.src"] == PROGRAM
    assert dirs["kuka_rsi.rsix"] == SENSOR_INTERFACE
    assert [n for n, d in dirs.items() if d is None] == ["LICENSE-FZI", "LICENSE-MIT", "LICENSE-APACHE"]


def test_write_files(tmp_path):
    out = tmp_path / "nested"
    paths = controller.write_files(out)
    assert [p.name for p in paths] == NAMES
    for p, f in zip(paths, controller.files()):
        assert p == out / local_path(f)
        want = f.contents.encode("utf-8")
        if f.name.endswith(".src"):
            want = want.replace(b"\n", b"\r\n")
        assert p.read_bytes() == want


def test_written_config_loads_in_the_driver(tmp_path):
    xml = controller.write_files(tmp_path)[0]
    drv = kuka_rsi.Driver(str(xml), bind="127.0.0.1:0", rt_priority=False)
    drv.close()


def test_module_cli_writes_the_files(tmp_path):
    out = subprocess.run(
        [sys.executable, "-m", "kuka_rsi.controller", str(tmp_path)], check=True, capture_output=True, text=True
    ).stdout
    assert sorted(tree(tmp_path)) == sorted(local_path(f).as_posix() for f in controller.files())
    assert PROGRAM in out
