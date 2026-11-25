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

    The driver faults a session itself when the robot's telegrams come at another cycle than
    `cycle_ms` (the message starts with "cycle mismatch"): every duration and speed limit is
    counted in cycles, so motion would run at the wrong speed.
    """

class Paused(TimeoutError):
    """Playback stood still for longer than the `pause_timeout` of a wait.

    The robot program is stopped (the enabling switch or Start was released), or the pendant
    override or the speed scale is 0. The motion is still queued and continues when playback
    runs again. A `TimeoutError`, so code that catches that keeps working; catch `Paused` first
    to tell it from a command that cannot settle.
    """

class Interrupted(Error):
    """The motion a wait was for did not finish: `hold`, `move_to` or `stream(replace=True)`
    dropped it, from this thread or another."""

class Motion:
    """What `Driver.move_to` and `Driver.stream` queued, for `Driver.wait_idle(motion=...)`.

    A motion starts when the queue is dropped (`move_to`, `hold`, `stream(replace=True)`) and
    includes everything appended to it since: `stream` without `replace` returns the motion it
    extended. Motions compare equal when they are the same one.
    """
    def __eq__(self, other: object) -> bool: ...

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
        robot; None uses the built-in copy of the shipped kuka_rsi_ethernet.xml.

        `cycle_ms` is the robot's RSI cycle, set by the sensor mode of `RSI_ON` in the robot
        program: 4 (`#IPO_FAST`, the default) or 12 (`#IPO`). `link_timeout` is raised to 125
        cycles if it is shorter (1.5 s at 12 ms), so the driver does not give up a session
        before the robot does."""
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
    def last_session_end(self) -> str | None:
        """Why the last session ended, None until one has; kept while the next one runs.

        "StopRequested": the driver pulsed the stop channel (`stop_program`, `close`) and the
        telegrams stopped soon after. "Fault": the session was faulted. "LinkTimeout": the
        telegrams stopped unasked (the program ended, was reset or stopped with an error on the
        robot, or the link dropped). "Closed": the driver was closed while the robot was still
        sending. "Panic": the real-time thread panicked. Read it when `session()` turns "Lost"
        or a call raises `LinkLost`.
        """
    def is_paused(self) -> bool:
        """True while playback stands still in an active session: the robot program is stopped
        (the enabling switch or Start was released), or the pendant override or the speed scale
        is 0. Queued motion continues when it runs again."""
    def is_idle(self) -> bool:
        """True when every queued command has been applied, the queue is empty and the command
        has settled: what `wait_idle` waits for, without blocking.

        Raises as `wait_idle` does: `LinkLost` or `ProtocolError` outside a session, `Faulted`
        on a faulted one.
        """
    def wait_for_robot(self, timeout: float = 60.0) -> None: ...
    def wait_idle(
        self, timeout: float = 10.0, *, motion: Motion | None = None, pause_timeout: float | None = None
    ) -> None:
        """Wait until the queue has drained and the command has settled.

        `timeout` counts playback time, not wall time: playback follows the pendant override
        and the speed scale and stands still while the robot program is stopped. A 2 s move is
        given the same room at 10 % override (20 s) and across a released enabling switch.
        `TimeoutError` therefore means playback ran for `timeout` and the command still had not
        settled, e.g. a limit holds it short of the target.

        While playback stands still the wait goes on: for as long as it takes, or with
        `pause_timeout` (seconds) until that has passed, then `Paused`. With `motion` (from
        `move_to` or `stream`) the wait raises `Interrupted` if that motion was held or
        replaced, and `LinkLost` if its session has ended; without it, the wait is for whatever
        is queued and returns normally after a `hold`. Raises `Faulted` once a `fault()` has
        frozen the command. Ctrl-C (KeyboardInterrupt) ends the wait within 50 ms.
        """
    def state(self) -> RobotState: ...
    def stats(self) -> Stats: ...
    def events(self) -> list[dict[str, Any]]:
        """The events since the last call, oldest first, one dict each.

        Every dict has `kind` and `t`, the driver time in seconds on the clock of
        `RobotState.t`: an event raised in a cycle has the `t` of that cycle's `drain_log` row.
        Kinds with fields: "RobotConnected" (`ipoc`), "SessionEnded" (`reason`, as
        `last_session_end`), "LateReply" (`latency_us`), "Clamped" (`limit`: "Position",
        "Velocity", "Acceleration", "Correction"), "TrackingError" (`error_rad`),
        "CycleMismatch" (`robot_ms`, `configured_ms`), "RtPriority" (`ok`). Without fields:
        "ParseError", "IoError", "SentypeMismatch", "Starved", "CommandRejected", "Fault",
        "Notified", "StopPulsed", "Panic", "LogDropped".
        """
    def drain_log(self) -> list[RobotState]: ...
    def stream(self, points: ArrayLike, dt: float, replace: bool = False) -> Motion:
        """Queue absolute joint targets (rad), `dt` seconds apart.

        Returns the motion the points belong to: a new one with `replace`, else the one they
        were appended to. Raises `Faulted` after a `fault()`, until the KRL program restarts.
        """
    def move_to(
        self, q: ArrayLike, duration: float, block: bool = False, *, pause_timeout: float | None = None
    ) -> Motion:
        """Smooth (quintic) move to `q` over `duration` seconds; drops what was queued.

        With `block=True`, waits as `wait_idle(duration + 5.0, motion=..., pause_timeout=...)`
        does: the 5 s are for the motion limiter to settle, and all of it is playback time, so
        a low override or a stopped program does not use it up. `TimeoutError` if a clamp keeps
        the command short of the target for that long, `Paused` after `pause_timeout` of a
        stopped program, `Interrupted` if a `hold` or another motion cut the move short.
        Raises `Faulted` after a `fault()`, until the KRL program restarts.
        """
    def hold(self) -> None:
        """Drop the queue and hold the current command. A wait for the motion that was queued
        raises `Interrupted`."""
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

    def set_program_running(self, running: bool) -> None:
        """Report the program as running or stopped from the next telegram on: what releasing
        the enabling switch or the Start key does. The driver pauses playback while it is
        stopped."""
    def set_override_percent(self, percent: float) -> None:
        """The pendant override (0 to 100) from the next telegram on."""
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
    cycle_ms: float = 4.0,
    report: Callable[[dict[str, Any]], object] | None = None,
) -> list[dict[str, Any]]:
    """Check the config (None: the built-in kuka_rsi_ethernet.xml), network address and port,
    route and ping to `robot`, real-time priority and, with `wait` > 0, the live link and its
    timing.

    Returns one dict per check: name, status ("pass", "warn", "fail", "skip"), detail, hint.
    `report` gets each check as it completes (with "event": "check") and, when the live check
    starts listening, {"event": "listening", "addr", "wait"}: start the robot program then. The
    live check holds the robot for `hold` seconds and ends the program with `stop_channel`.
    `cycle_ms` is the robot's RSI cycle (4 for `#IPO_FAST`, 12 for `#IPO`); the timing check
    fails if the robot sends at another one.
    Ctrl-C cancels it and still ends the program cleanly.
    """
