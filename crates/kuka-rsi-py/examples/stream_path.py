"""Stream a multi-axis sine the way a policy would, from Python.

    python examples/stream_path.py kuka_rsi_ethernet.xml [--axes 6] [--amp 3] [--period 4]
        [--secs 8] [--stop-after S] [--speed 20] [--log run.csv] [--bind ADDR]

The same path and checks as the Rust example `stream_trajectory`: every 100 ms the next 0.5 s
of the path as five points with replace=True. Each axis in --axes swings +/- --amp degrees around
the start pose, phases spread across the axes, fading in over the first period and out over the
last. --stop-after stops sending mid-path; the report shows where the robot holds against the
last target sent. Then it returns to the start pose. Amplitudes are capped at 30 degrees (A6 45)
and the path's peak speed must stay within --speed (at most 30 deg/s).
"""

from __future__ import annotations

import argparse
import logging
import math
import pathlib
import sys
import time

import numpy as np

import kuka_rsi

CAP_DEG = [30.0, 30.0, 30.0, 30.0, 30.0, 45.0]
MAX_SPEED_DEG_S = 30.0
CHUNK_EVERY = 0.1
POINTS = 5


def smoothstep(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


def offset_deg(a: argparse.Namespace, t: float) -> np.ndarray:
    o = np.zeros(6)
    fade = smoothstep(min(t, a.secs - t) / a.period)
    for k, i in enumerate(a.axes):
        phase = 2 * math.pi * k / len(a.axes)
        o[i] = a.amp * fade * math.sin(2 * math.pi * t / a.period + phase)
    return o


def parse(argv: list[str] | None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("config")
    ap.add_argument("--axes", default="6")
    ap.add_argument("--amp", type=float, default=3.0)
    ap.add_argument("--period", type=float, default=4.0)
    ap.add_argument("--secs", type=float, default=8.0)
    ap.add_argument("--stop-after", type=float)
    ap.add_argument("--speed", type=float, default=20.0)
    ap.add_argument("--log")
    ap.add_argument("--bind")
    a = ap.parse_args(argv)
    try:
        a.axes = sorted({int(x.strip().lstrip("Aa")) - 1 for x in a.axes.split(",")})
    except ValueError:
        ap.error("--axes: comma-separated axis numbers")
    if any(not 0 <= i <= 5 for i in a.axes):
        ap.error("axes must be 1 to 6")
    for i in a.axes:
        if not (math.isfinite(a.amp) and 0 < a.amp <= CAP_DEG[i]):
            ap.error(f"--amp must be above 0 and at most {CAP_DEG[i]} deg for A{i + 1}")
    if not 1 <= a.speed <= MAX_SPEED_DEG_S:
        ap.error(f"--speed must be 1 to {MAX_SPEED_DEG_S} deg/s")
    if not (math.isfinite(a.period) and math.isfinite(a.secs) and a.period >= 1 and 2 * a.period <= a.secs <= 600):
        ap.error("need --period of at least 1 s and --secs of at least two periods, at most 600 s")
    peak = a.amp * (2 * math.pi + 1.5) / a.period
    if peak > a.speed:
        ap.error(f"the path peaks at {peak:.1f} deg/s, above --speed {a.speed}: lower --amp or raise --period")
    if a.stop_after is not None and not 0 < a.stop_after < a.secs:
        ap.error("--stop-after must be within --secs")
    return a


def main(argv: list[str] | None = None) -> int:
    a = parse(argv)
    # The driver's events (late replies, starvation, link lost) go to Python logging.
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    limits = kuka_rsi.Limits.kr10_defaults(max_velocity=math.radians(a.speed))
    kw = {"bind": a.bind} if a.bind else {}
    drv = kuka_rsi.Driver(a.config, limits=limits, stop_channel="MoveStop", **kw)
    if a.log:
        pathlib.Path(a.log).parent.mkdir(parents=True, exist_ok=True)
    log = open(a.log, "w") if a.log else None  # noqa: SIM115 - closed in finally
    if log:
        log.write(kuka_rsi.CSV_HEADER)
    states: list[kuka_rsi.RobotState] = []

    def drain() -> None:
        new = drv.drain_log()
        states.extend(new)
        if log:
            log.writelines(s.csv_row() for s in new)
            log.flush()

    try:
        print(f"listening on {drv.local_addr()}: start the program on the pendant", flush=True)
        while True:  # short waits, so Ctrl-C gets through
            try:
                drv.wait_for_robot(0.5)
                break
            except TimeoutError:
                continue
        q0 = drv.state().q_setpoint
        drain()
        states.clear()
        before = drv.stats()
        end = a.stop_after if a.stop_after is not None else a.secs
        print(f"streaming A{[i + 1 for i in a.axes]} +/-{a.amp} deg for {end:.1f} s", flush=True)
        t0 = time.monotonic()
        nxt = t0
        last_target = q0
        while (t := time.monotonic() - t0) < end:
            points = np.array([q0 + np.radians(offset_deg(a, min(t + k * CHUNK_EVERY, a.secs))) for k in range(1, POINTS + 1)])
            drv.stream(points, dt=CHUNK_EVERY, replace=True)
            last_target = points[-1]
            drain()
            nxt += CHUNK_EVERY
            time.sleep(max(0.0, nxt - time.monotonic()))
        streamed = len(states)
        time.sleep(1.0)  # the last chunk plays out (0.5 s of path), then the robot holds
        drain()
        after = drv.stats()
        q_held = drv.state().q
        print(f"stream ({streamed} cycles):")
        for i in a.axes:
            lag = max(abs(s.q_command[i] - s.q[i]) for s in states[:streamed])
            held = math.degrees(q_held[i] - last_target[i])
            print(f"  A{i + 1}  max lag {math.degrees(lag):.3f} deg, held {held:+.3f} deg from the last target sent")
        print(
            f"  {after.late_replies - before.late_replies} late replies, clamps: "
            f"{after.position_clamps - before.position_clamps} position, "
            f"{after.velocity_clamps - before.velocity_clamps} velocity, "
            f"{after.acceleration_clamps - before.acceleration_clamps} acceleration, "
            f"{after.correction_clamps - before.correction_clamps} correction, "
            f"robot Delay max {max((s.delay for s in states), default=0)}"
        )
        back = max(1.875 * a.amp / a.speed, 2.0)
        drv.move_to(q0, duration=back)
        drv.wait_idle(back * 10 + 30)
        drain()
        err = max(abs(math.degrees(drv.state().q[i] - q0[i])) for i in a.axes)
        print(f"back at the start pose, largest error {err:.3f} deg")
        if log:
            print(f"logged {len(states)} cycles to {a.log}")
    except KeyboardInterrupt:
        print("\nCtrl-C: ending the robot program (MoveStop) and closing")
        return 130
    finally:
        drv.close()
        if log:
            log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
