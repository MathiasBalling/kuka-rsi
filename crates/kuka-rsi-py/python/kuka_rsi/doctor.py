"""Check the PC side of an RSI setup and, with --wait, the live link to the robot.

    python -m kuka_rsi.doctor --robot 192.168.10.147 --wait 30

Without --config it checks the built-in kuka_rsi_ethernet.xml, the file the package ships.

With --wait, start kuka_rsi (or kuka_rsi_monitor) on the pendant when asked. The doctor holds
the robot where it stands for --hold seconds, then ends the program (MoveStop).
"""

from __future__ import annotations

import argparse
import sys
import textwrap
from typing import Any

from kuka_rsi._core import check_setup


def _print_event(event: dict[str, Any]) -> None:
    if event["event"] == "listening":
        print(
            f"        listening on {event['addr']}: start the program on the pendant now (waiting {event['wait']:.0f} s)",
            flush=True,
        )
        return
    print(f"  {event['status'].upper()}  {event['name']:<10}  {event['detail']}")
    if event["hint"]:
        print(textwrap.fill(event["hint"], width=100, initial_indent=" " * 8, subsequent_indent=" " * 8))
    sys.stdout.flush()  # piped output (a log, a CI job) should show each check as it completes


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="python -m kuka_rsi.doctor", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--config", help="RSI Ethernet config XML, the same file as on the robot; default: the built-in kuka_rsi_ethernet.xml"
    )
    ap.add_argument("--robot", help="the robot's RSI address, for the route and ping checks")
    ap.add_argument("--bind", help="bind address override; default IP_NUMBER:PORT from the config")
    ap.add_argument("--wait", type=float, default=0.0, help="seconds to wait for the robot; 0 skips the live checks")
    ap.add_argument("--hold", type=float, default=3.0, help="seconds to hold the robot while measuring the timing")
    ap.add_argument("--stop-channel", default="MoveStop")
    ap.add_argument(
        "--cycle-ms", type=float, default=4.0, help="the robot's RSI cycle: 4 (#IPO_FAST) or 12 (#IPO, RSI_ON(#ABSOLUTE, #IPO, #BASE))"
    )
    a = ap.parse_args(argv)
    try:
        checks = check_setup(
            a.config,
            robot=a.robot,
            bind=a.bind,
            wait=a.wait,
            hold=a.hold,
            stop_channel=a.stop_channel,
            cycle_ms=a.cycle_ms,
            report=_print_event,
        )
    except KeyboardInterrupt:
        print("\ncancelled")
        return 130
    failed = sum(c["status"] == "fail" for c in checks)
    warnings = sum(c["status"] == "warn" for c in checks)
    print(f"\n{failed} failed, {warnings} warnings")
    return 1 if failed else 0


__all__ = ["check_setup", "main"]

if __name__ == "__main__":
    sys.exit(main())
