import socket
import subprocess
import sys
import threading

import kuka_rsi
from kuka_rsi import controller


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def config(tmp_path) -> str:
    return str(controller.write_files(tmp_path)[0])


def by_name(checks):
    return {c["name"]: c for c in checks}


def test_offline_checks_on_loopback(tmp_path):
    seen = []
    checks = kuka_rsi.check_setup(
        config(tmp_path), bind=f"127.0.0.1:{free_port()}", robot="127.0.0.1", report=seen.append
    )
    c = by_name(checks)
    assert [c["status"] for c in checks][:4] == ["pass"] * 4, checks
    assert c["robot link"]["status"] == "skip"
    assert c["port"]["hint"] is None
    assert [e["name"] for e in seen if e["event"] == "check"] == [x["name"] for x in checks]


def test_without_a_config_the_built_in_one_is_checked():
    checks = kuka_rsi.check_setup(bind=f"127.0.0.1:{free_port()}")
    c = by_name(checks)["config"]
    assert c["status"] == "pass", c
    assert c["detail"].startswith("built-in kuka_rsi_ethernet.xml: IP_NUMBER 192.168.10.10"), c


def test_missing_config_fails(tmp_path):
    checks = kuka_rsi.check_setup(str(tmp_path / "missing.xml"))
    assert [(c["name"], c["status"]) for c in checks] == [("config", "fail")]


def test_live_check_against_the_simulator(tmp_path):
    xml = config(tmp_path)
    port = free_port()
    sims = []

    def on_progress(event):
        if event["event"] == "listening":
            assert event["addr"] == f"127.0.0.1:{port}"
            sims.append(kuka_rsi.ControllerSim(xml, f"127.0.0.1:{port}", bind="127.0.0.1:0"))

    checks = kuka_rsi.check_setup(xml, bind=f"127.0.0.1:{port}", wait=5.0, hold=1.0, report=on_progress)
    c = by_name(checks)
    assert c["robot link"]["status"] == "pass", c["robot link"]
    assert c["timing"]["status"] != "fail", c["timing"]
    assert sims[0].wait_exit(3.0) == "Stopped"


def test_module_cli_reports_and_exits_nonzero_on_failure(tmp_path):
    run = subprocess.run(
        [sys.executable, "-m", "kuka_rsi.doctor", "--config", str(tmp_path / "missing.xml")],
        capture_output=True,
        text=True,
    )
    assert run.returncode == 1
    assert "FAIL  config" in run.stdout
