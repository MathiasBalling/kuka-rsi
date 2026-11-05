import pathlib
import subprocess
import sys

import kuka_rsi
from kuka_rsi import controller

FOLDER = pathlib.Path(__file__).resolve().parents[2] / "kuka-rsi" / "controller"
NAMES = [
    "kuka_rsi_ethernet.xml",
    "kuka_rsi_monitor.xml",
    "kuka_rsi.rsix",
    "kuka_rsi_monitor.rsix",
    "kuka_rsi.src",
    "kuka_rsi_monitor.src",
    "LICENSE-FZI",
]


def test_bundled_files_match_the_repository():
    files = controller.files()
    assert [f.name for f in files] == NAMES
    for f in files:
        assert f.contents == (FOLDER / f.name).read_text(encoding="utf-8")
    dirs = {f.name: f.robot_dir for f in files}
    assert dirs["kuka_rsi.src"] == "KRC:\\R1\\Program\\"
    assert dirs["kuka_rsi.rsix"] == "C:\\KRC\\ROBOTER\\Config\\User\\Common\\SensorInterface\\"
    assert dirs["LICENSE-FZI"] is None


def test_write_files(tmp_path):
    out = tmp_path / "nested"
    paths = controller.write_files(out)
    assert [p.name for p in paths] == NAMES
    for p, f in zip(paths, controller.files()):
        assert p.parent == out
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
    assert sorted(p.name for p in tmp_path.iterdir()) == sorted(NAMES)
    assert "KRC:\\R1\\Program\\" in out
