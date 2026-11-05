import pathlib
import socket
import subprocess
import sys

import kuka_rsi

ROOT = pathlib.Path(__file__).resolve().parents[1]
CONFIG = ROOT.parents[1] / "controller" / "kuka_rsi_ethernet.xml"


def test_csv_row_matches_the_header():
    drv = kuka_rsi.Driver(str(CONFIG), bind="127.0.0.1:0", rt_priority=False)
    sim = kuka_rsi.ControllerSim(str(CONFIG), drv.local_addr(), bind="127.0.0.1:0")
    drv.wait_for_robot(3.0)
    row = drv.state().csv_row()
    drv.close()
    sim.stop()
    names = kuka_rsi.CSV_HEADER.rstrip("\n").split(",")
    values = row.rstrip("\n").split(",")
    assert len(names) == len(values)
    assert dict(zip(names, values))["session"] == "Active"
    assert row.endswith("\n")


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_stream_path_example_holds_after_a_stop(tmp_path):
    port = free_port()
    log = tmp_path / "runs" / "stream.csv"
    cmd = [sys.executable, str(ROOT / "examples" / "stream_path.py"), str(CONFIG), "--axes", "1,6", "--amp", "2",
           "--period", "1", "--secs", "3", "--stop-after", "2", "--bind", f"127.0.0.1:{port}", "--log", str(log)]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    out = ""
    while "listening" not in out:
        line = proc.stdout.readline()
        assert line, out
        out += line
    sim = kuka_rsi.ControllerSim(str(CONFIG), f"127.0.0.1:{port}", bind="127.0.0.1:0")
    out += proc.stdout.read()
    assert proc.wait(30) == 0, out
    assert "A1  max lag" in out and "A6  max lag" in out, out
    assert "held" in out and "back at the start pose" in out, out
    assert sim.wait_exit(3.0) == "Stopped"
    lines = log.read_text().splitlines()
    assert lines[0] == kuka_rsi.CSV_HEADER.rstrip("\n")
    assert len(lines) > 300, len(lines)


def test_stream_path_example_refuses_too_fast_a_path():
    cmd = [sys.executable, str(ROOT / "examples" / "stream_path.py"), str(CONFIG), "--amp", "10", "--period", "2", "--secs", "4"]
    run = subprocess.run(cmd, capture_output=True, text=True)
    assert run.returncode != 0
    assert "peaks" in run.stderr
