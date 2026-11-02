import pathlib
import time

import numpy as np
import pytest

import kuka_rsi

CONFIG = pathlib.Path(__file__).resolve().parents[3] / "controller" / "kuka_rsi_ethernet.xml"


def make(**sim_kw):
    drv = kuka_rsi.Driver(
        str(CONFIG),
        bind="127.0.0.1:0",
        limits=kuka_rsi.Limits.kr10_defaults(max_velocity=2.0, max_acceleration=None),
        stop_channel="MoveStop",
        rt_priority=False,
    )
    sim = kuka_rsi.ControllerSim(str(CONFIG), target=drv.local_addr(), bind="127.0.0.1:0", **sim_kw)
    drv.wait_for_robot(3.0)
    return drv, sim


def test_handshake_and_state_shapes():
    drv, sim = make()
    st = drv.state()
    assert st.session == "Active"
    assert drv.session() == "Active"
    assert st.q.shape == (6,) and st.q.dtype == np.float64
    assert np.allclose(st.ak, 0.0)
    assert np.isclose(st.q_setpoint[1], np.radians(-90))
    d = st.as_dict()
    assert set(d) >= {"t", "cycle", "ipoc", "session", "q", "q_setpoint", "tcp", "torque", "delay", "speed_override", "program_running", "q_command", "ak"}
    assert st.program_running and st.speed_override == 1.0
    drv.close()
    assert sim.wait_exit(2.0) == "Stopped"


def test_stream_and_wait_idle():
    drv, sim = make()
    q = drv.state().q.copy()
    pts = np.tile(q, (5, 1))
    pts[:, 0] += np.linspace(0.01, 0.05, 5)
    drv.stream(pts, dt=0.1)
    drv.wait_idle(5.0)
    assert abs(sim.status()["q_setpoint_deg"][0] - np.degrees(q[0] + 0.05)) < 1e-3
    drv.stream(q, dt=0.1, replace=True)  # a single (6,) point is accepted
    drv.wait_idle(5.0)
    assert abs(sim.status()["q_setpoint_deg"][0] - np.degrees(q[0])) < 1e-3
    drv.close()


def test_limit_and_value_errors():
    drv, _sim = make()
    q = drv.state().q.copy()
    q[0] = 4.0
    with pytest.raises(kuka_rsi.LimitError):
        drv.stream(q, dt=0.1)
    with pytest.raises(ValueError):
        drv.stream(np.zeros((2, 5)), dt=0.1)
    with pytest.raises(kuka_rsi.LimitError):
        drv.move_to(drv.state().q, duration=0.0)
    with pytest.raises(kuka_rsi.LimitError):
        drv.set_speed_scale(1.5)
    drv.close()


def test_context_manager_waits_and_closes():
    drv = kuka_rsi.Driver(str(CONFIG), bind="127.0.0.1:0", stop_channel="MoveStop", rt_priority=False)
    sim = kuka_rsi.ControllerSim(str(CONFIG), target=drv.local_addr(), bind="127.0.0.1:0")
    with drv as d:
        assert d.session() == "Active"
        d.move_to(d.state().q + np.array([0, 0, 0, 0, 0, 0.01]), duration=0.2, block=True)
    assert sim.wait_exit(2.0) == "Stopped"
    with pytest.raises(kuka_rsi.Error):
        drv.hold()


def test_not_connected_errors():
    drv = kuka_rsi.Driver(str(CONFIG), bind="127.0.0.1:0", rt_priority=False)
    with pytest.raises(TimeoutError):
        drv.wait_for_robot(0.2)
    with pytest.raises(kuka_rsi.ProtocolError):
        drv.stream(np.zeros(6), dt=0.1)
    drv.close()


def test_log_stats_and_events():
    drv, _sim = make()
    drv.drain_log()
    time.sleep(0.4)
    log = drv.drain_log()
    assert 60 <= len(log) <= 140
    assert log[1].cycle == log[0].cycle + 1
    st = drv.stats()
    assert st.cycles > 0 and st.sessions == 1
    assert "replies" in st.as_dict()
    time.sleep(0.15)
    ev = drv.events()
    assert any("RobotConnected" in e for e in ev), ev
    drv.close()


def test_fault_stops_the_sim_and_link_is_lost_after_sim_stops():
    drv, sim = make()
    drv.fault("policy diverged")
    assert sim.wait_exit(2.0) == "EStrError"
    t0 = time.monotonic()
    while drv.session() != "Lost" and time.monotonic() - t0 < 2.0:
        time.sleep(0.01)
    assert drv.session() == "Lost"
    with pytest.raises(kuka_rsi.LinkLost):
        drv.stream(np.zeros(6), dt=0.1)
    drv.close()


def test_array_like_inputs_are_accepted():
    drv, _sim = make()
    q = drv.state().q.copy()
    drv.stream(list(q), dt=0.1)                      # plain list
    drv.stream(np.tile(q, (3, 1)).astype(np.float32), dt=0.1)  # float32 array
    drv.move_to([float(v) for v in q], duration=0.2, block=True)
    with pytest.raises(ValueError):
        drv.stream([[0.0] * 5], dt=0.1)
    drv.close()


def test_fault_is_visible_and_refuses_motion():
    # No stop channel, and a link timeout long enough to inspect the faulted session after the
    # simulator stops on the fault message.
    drv = kuka_rsi.Driver(str(CONFIG), bind="127.0.0.1:0", link_timeout=3.0, rt_priority=False)
    _sim = kuka_rsi.ControllerSim(str(CONFIG), target=drv.local_addr(), bind="127.0.0.1:0")
    drv.wait_for_robot(3.0)
    assert not drv.is_faulted()
    q = drv.state().q_command
    drv.fault("policy diverged")
    t0 = time.monotonic()
    while not drv.is_faulted() and time.monotonic() - t0 < 1.0:
        time.sleep(0.005)
    assert drv.is_faulted()
    assert drv.session() == "Active"
    with pytest.raises(kuka_rsi.Faulted):
        drv.stream(q, dt=0.1)
    with pytest.raises(kuka_rsi.Faulted):
        drv.move_to(q, duration=0.2, block=True)
    with pytest.raises(kuka_rsi.Faulted):
        drv.notify("hello")
    with pytest.raises(kuka_rsi.Faulted):
        drv.wait_idle(0.1)
    assert issubclass(kuka_rsi.Faulted, kuka_rsi.Error)
    assert not issubclass(kuka_rsi.Faulted, kuka_rsi.LimitError)
    drv.close()
