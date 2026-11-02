import pathlib

import numpy as np
import pytest

import kuka_rsi

CONFIG = pathlib.Path(__file__).resolve().parents[3] / "controller" / "kuka_rsi_ethernet.xml"


def test_limits_defaults_and_arrays():
    lim = kuka_rsi.Limits.kr10_defaults(max_velocity=0.5)
    assert isinstance(lim.max_velocity, np.ndarray)
    assert lim.max_velocity.shape == (6,)
    assert np.allclose(lim.max_velocity, 0.5)
    assert np.isclose(lim.position_max[0], np.radians(170))
    lim.max_velocity = [0.25] * 6
    assert np.allclose(lim.max_velocity, 0.25)
    with pytest.raises(ValueError):
        lim.max_velocity = np.zeros(5)
    lim.validate()
    lim.max_velocity = np.zeros(6)
    with pytest.raises(kuka_rsi.Error):
        lim.validate()
    lim = kuka_rsi.Limits.kr10_defaults(max_acceleration=None)
    assert lim.max_acceleration is None
    lim.max_acceleration = np.full(6, 3.0)
    assert np.allclose(lim.max_acceleration, 3.0)
    assert lim.max_tracking_error > 0


def test_exception_hierarchy():
    assert issubclass(kuka_rsi.LinkLost, kuka_rsi.Error)
    assert issubclass(kuka_rsi.ProtocolError, kuka_rsi.Error)
    assert issubclass(kuka_rsi.LimitError, kuka_rsi.Error)
    assert issubclass(kuka_rsi.Error, Exception)


def test_sim_runs_for_max_cycles_and_reports_status():
    sim = kuka_rsi.ControllerSim(str(CONFIG), target="127.0.0.1:49998", bind="127.0.0.1:0", cycle_ms=1, max_cycles=20)
    assert sim.local_addr().startswith("127.0.0.1:")
    assert sim.wait_exit(2.0) == "MaxCycles"
    st = sim.status()
    assert st["cycles"] == 20
    assert st["exit"] == "MaxCycles"
    assert len(st["q_setpoint_deg"]) == 6
    assert sim.stop() == "MaxCycles"


def test_sim_context_manager_stops():
    with kuka_rsi.ControllerSim(str(CONFIG), target="127.0.0.1:49998", bind="127.0.0.1:0") as sim:
        assert sim.status()["exit"] is None
    assert sim.status()["exit"] in ("Shutdown", "RsiBad")


def test_bad_config_path_raises():
    with pytest.raises(kuka_rsi.Error):
        kuka_rsi.ControllerSim("/nonexistent.xml", target="127.0.0.1:1")
