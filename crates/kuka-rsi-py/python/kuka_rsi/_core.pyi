import os
import pathlib
from collections.abc import Callable, Sequence
from typing import Any, Self

import numpy as np
from numpy.typing import ArrayLike, NDArray

CSV_HEADER: str
"""Header of the per-cycle CSV log (see RobotState.csv_row), newline-terminated."""

class Error(Exception): ...
class LinkLost(Error): ...
class ProtocolError(Error): ...
class LimitError(Error): ...

class Faulted(Error):
    """A `fault()` froze the command.

    The fault persists until the KRL program restarts (a new session); meanwhile `stream`,
    `move_to`, `notify` and `wait_idle` raise this. `fault` and `stop_program` stay allowed.
    """

class Limits:
    @staticmethod
    def kr10_defaults(max_velocity: float | None = None, max_acceleration: float | None = ...) -> Limits:
        """KR 10 R1100-2 axis ranges with conservative speeds; scalars apply to all joints.

        `max_velocity` in rad/s (default 30°/s). `max_acceleration` in rad/s²: omitted, it keeps
        the default of 300°/s²; `None` disables the acceleration clamp.
        """
    position_min: NDArray[np.float64]
    position_max: NDArray[np.float64]
    max_velocity: NDArray[np.float64]
    max_correction: NDArray[np.float64]
    max_acceleration: NDArray[np.float64] | None
    max_tracking_error: float
    def validate(self) -> None: ...

class RobotState:
    def csv_row(self) -> str:
        """This state as a CSV row matching CSV_HEADER (degrees, millimetres)."""
    @property
    def t(self) -> float: ...
    @property
    def cycle(self) -> int: ...
    @property
    def ipoc(self) -> int: ...
    @property
    def session(self) -> str: ...
    @property
    def q(self) -> NDArray[np.float64]: ...
    @property
    def q_setpoint(self) -> NDArray[np.float64]: ...
    @property
    def tcp(self) -> NDArray[np.float64]: ...
    @property
    def tcp_setpoint(self) -> NDArray[np.float64]: ...
    @property
    def torque(self) -> NDArray[np.float64]: ...
    @property
    def q_command(self) -> NDArray[np.float64]: ...
    @property
    def ak(self) -> NDArray[np.float64]: ...
    @property
    def delay(self) -> int: ...
    @property
    def speed_override(self) -> float: ...
    @property
    def program_running(self) -> bool: ...
    def as_dict(self) -> dict[str, Any]: ...

class Stats:
    sessions: int
    cycles: int
    replies: int
    late_replies: int
    reply_latency_max_us: int
    reply_latency_mean_us: float
    interarrival_max_us: int
    ipoc_gaps: int
    skipped_datagrams: int
    parse_errors: int
    io_errors: int
    sentype_mismatches: int
    position_clamps: int
    velocity_clamps: int
    acceleration_clamps: int
    correction_clamps: int
    starved_cycles: int
    log_drops: int
    queue_len: int
    robot_delay: int
    def as_dict(self) -> dict[str, Any]: ...

class Driver:
    def __init__(
        self,
        config: str | os.PathLike[str] | None = None,
        *,
        bind: str | None = None,
        limits: Limits | None = None,
        stop_channel: str | None = None,
        link_timeout: float = 0.5,
        rt_priority: bool = True,
        cycle_ms: float = 4.0,
        wait: float = 60.0,
    ) -> None:
        """Start the driver for the RSI Ethernet config XML at `config`, the same file as on the
        robot; None uses the built-in copy of the shipped kuka_rsi_ethernet.xml."""
    def local_addr(self) -> str: ...
    def robot_addr(self) -> str | None:
        """The robot's address ("ip:port") during a session, else None."""
    def session(self) -> str: ...
    def limits(self) -> Limits:
        """The limits in force: the constructor's, or the last accepted `set_limits`."""
    def is_faulted(self) -> bool:
        """True once a `fault()` has been applied (with the next telegram after the call).

        The command stays frozen and `stream`, `move_to`, `notify` and `wait_idle` raise
        `Faulted` for the rest of the session. A fault is a state within an `"Active"`
        session, so `session()` does not show it; when the robot stops on the fault the
        session ends ("Lost"), `is_faulted()` turns false and those calls raise `LinkLost`
        until the KRL program restarts (a new session).
        """
    def wait_for_robot(self, timeout: float = 60.0) -> None: ...
    def wait_idle(self, timeout: float = 10.0) -> None:
        """Wait until the queue has drained and the command has settled.

        Raises `TimeoutError` if that takes longer than `timeout`, and `Faulted` once a
        `fault()` has frozen the command.
        """
    def state(self) -> RobotState: ...
    def stats(self) -> Stats: ...
    def events(self) -> list[str]: ...
    def drain_log(self) -> list[RobotState]: ...
    def stream(self, points: ArrayLike, dt: float, replace: bool = False) -> None:
        """Queue absolute joint targets (rad), `dt` seconds apart.

        Raises `Faulted` after a `fault()`, until the KRL program restarts.
        """
    def move_to(self, q: ArrayLike, duration: float, block: bool = False) -> None:
        """Smooth (quintic) move to `q` over `duration` seconds.

        With `block=True`, waits on `wait_idle` with a `duration + 5.0` second timeout
        (not just `duration`): `wait_idle` also waits for the motion limiter to settle,
        and raises `TimeoutError` if a clamp keeps the command short of the target or the
        program is paused for longer than that timeout. Raises `Faulted` after a `fault()`,
        until the KRL program restarts.
        """
    def hold(self) -> None: ...
    def fault(self, msg: str) -> None:
        """Stop the robot through `EStr` (`Error: msg`) and freeze the command.

        For the rest of the session `stream`, `move_to`, `notify` and `wait_idle` raise
        `Faulted` and `is_faulted()` is true; motion resumes only after the KRL program
        restarts (a new session).
        """
    def notify(self, msg: str) -> None:
        """Show `msg` on the smartHMI. Raises `Faulted` after a `fault()`."""
    def stop_program(self) -> None: ...
    def set_speed_scale(self, s: float) -> None: ...
    def set_limits(self, limits: Limits) -> None: ...
    def close(self) -> None:
        """Pulse the stop channel if configured, then stop the driver's threads. Idempotent.

        Afterwards every method raises `Error("driver is closed")`. A Driver dropped without
        `close()` is closed on deallocation (with the GIL released).
        """
    def __enter__(self) -> Self: ...
    def __exit__(self, *args: object) -> bool: ...

class ControllerSim:
    def __init__(
        self,
        config: str,
        target: str,
        *,
        bind: str = "0.0.0.0:0",
        cycle_ms: float = 4.0,
        timeout: int = 100,
        lag_cycles: float = 0.0,
        jitter_us: int = 0,
        drop_probability: float = 0.0,
        start_deg: Sequence[float] | None = None,
        max_cycles: int | None = None,
        seed: int | None = None,
    ) -> None: ...
    def local_addr(self) -> str: ...
    def status(self) -> dict[str, Any]:
        """Snapshot of the simulator's counters and last commanded/actual pose.

        Includes `overruns`: the number of cycles in which the simulator's own step took
        longer than `cycle_ms`, among the other fields (`cycles`, `replies`, `late`, `ipoc`,
        `q_setpoint_deg`, `q_actual_deg`, `last_ak_deg`, `clamped`, `exit`).
        """

    def wait_exit(self, timeout: float) -> str | None: ...
    def stop(self) -> str: ...
    def __enter__(self) -> Self: ...
    def __exit__(self, *args: object) -> bool: ...

def controller_files() -> list[tuple[str, str, str | None]]:
    """The bundled controller files as (name, contents, robot_dir); see kuka_rsi.controller."""
def write_controller_files(directory: str | os.PathLike[str]) -> list[pathlib.Path]:
    """Write the bundled controller files into `directory`; see kuka_rsi.controller.write_files."""

def check_setup(
    config: str | os.PathLike[str] | None = None,
    *,
    robot: str | None = None,
    bind: str | None = None,
    wait: float = 0.0,
    hold: float = 3.0,
    stop_channel: str = "MoveStop",
    report: Callable[[dict[str, Any]], object] | None = None,
) -> list[dict[str, Any]]:
    """Check the config (None: the built-in kuka_rsi_ethernet.xml), network address and port,
    route and ping to `robot`, real-time priority and, with `wait` > 0, the live link and its
    timing.

    Returns one dict per check: name, status ("pass", "warn", "fail", "skip"), detail, hint.
    `report` gets each check as it completes (with "event": "check") and, when the live check
    starts listening, {"event": "listening", "addr", "wait"}: start the robot program then. The
    live check holds the robot for `hold` seconds and ends the program with `stop_channel`.
    Ctrl-C cancels it and still ends the program cleanly.
    """
