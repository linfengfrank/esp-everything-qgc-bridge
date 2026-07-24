#!/usr/bin/env python3
"""Minimal GCS script: CMD_START (OFFBOARD + arm + takeoff) then CMD_GOTO.

This uses the existing custom ESP32 protocol, not direct MAVLink.
In this firmware, OFFBOARD mode switch and arming happen inside mission_task
after CMD_START is received.

Examples:
  python laptop/simple_arm_set_goal.py --drone-id 2 --goal-x -2.0 --goal-y 5.0
  python laptop/simple_arm_set_goal.py --drone-id 2 --goal-x 1.0 --goal-y 0.5 --confirm
    python laptop/simple_arm_set_goal.py --drone-id 2 --goal-x 0 --goal-y 0 --start-only --start-retries 3
"""

from __future__ import annotations

import argparse
import logging
import signal
import threading
import time
from pathlib import Path

from comms import CommsNode
from protocol import CMD_GOTO, CMD_START, CommandPacket, TelemetryPacket

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("simple_arm_set_goal")


class TelemetryWaiter:
    def __init__(self, drone_id: int) -> None:
        self._drone_id = drone_id
        self._cv = threading.Condition()
        self._seen = False

    def on_telemetry(self, pkt: TelemetryPacket, src_ip: str) -> None:
        if pkt.drone_id != self._drone_id:
            return
        with self._cv:
            self._seen = True
            self._cv.notify_all()

    def wait(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        with self._cv:
            while not self._seen:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        f"No telemetry from drone {self._drone_id} within {timeout_s:.1f}s"
                    )
                self._cv.wait(timeout=min(0.5, remaining))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Arm via CMD_START, then send one CMD_GOTO goal."
    )
    parser.add_argument("--drone-id", type=int, default=2, help="Target drone ID")
    parser.add_argument("--goal-x", type=float, required=True, help="Goal X in map frame (m)")
    parser.add_argument("--goal-y", type=float, required=True, help="Goal Y in map frame (m)")
    parser.add_argument("--telem-port", type=int, default=5005, help="Laptop telemetry UDP port")
    parser.add_argument("--cmd-port", type=int, default=5006, help="ESP32 command UDP port")
    parser.add_argument("--config", default="setup.yaml", help="Path to setup.yaml")
    parser.add_argument(
        "--takeoff-wait",
        type=float,
        default=3.0,
        help="Seconds to wait after CMD_START before sending CMD_GOTO",
    )
    parser.add_argument(
        "--telemetry-timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for first telemetry packet",
    )
    parser.add_argument(
        "--confirm",
        action="store_true",
        help="Require typing START-<drone_id> before sending CMD_START",
    )
    parser.add_argument(
        "--start-only",
        action="store_true",
        help="Send CMD_START only (no CMD_GOTO). Useful for OFFBOARD/arm debugging.",
    )
    parser.add_argument(
        "--start-retries",
        type=int,
        default=1,
        help="How many times to send CMD_START (default: 1).",
    )
    parser.add_argument(
        "--start-retry-interval",
        type=float,
        default=1.0,
        help="Seconds between repeated CMD_START sends (default: 1.0).",
    )
    return parser.parse_args()


def send_or_fail(node: CommsNode, drone_id: int, cmd: CommandPacket, name: str) -> None:
    ok = node.send_command(drone_id, cmd)
    if not ok:
        raise RuntimeError(f"Failed to send {name}: drone {drone_id} IP unknown")
    log.info("%s sent", name)


def main() -> int:
    args = parse_args()

    if args.takeoff_wait < 0.0:
        raise SystemExit("--takeoff-wait must be >= 0")
    if args.start_retries < 1:
        raise SystemExit("--start-retries must be >= 1")
    if args.start_retry_interval < 0.0:
        raise SystemExit("--start-retry-interval must be >= 0")

    waiter = TelemetryWaiter(args.drone_id)

    node = CommsNode(
        listen_port=args.telem_port,
        cmd_port=args.cmd_port,
        config_path=args.config if Path(args.config).exists() else None,
    )
    node.on_telemetry(waiter.on_telemetry)

    signal.signal(signal.SIGINT, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

    node.start()
    try:
        log.info("Waiting for drone %d telemetry on UDP %d...", args.drone_id, args.telem_port)
        waiter.wait(args.telemetry_timeout)
        log.info("Telemetry link established")

        if args.confirm:
            text = input(f"Type START-{args.drone_id} to continue: ").strip()
            if text != f"START-{args.drone_id}":
                log.info("Confirmation mismatch. Exiting without flight commands.")
                return 0

        # In this firmware, CMD_START triggers OFFBOARD request + arm + takeoff.
        for i in range(args.start_retries):
            send_or_fail(node, args.drone_id, CommandPacket(CMD_START), f"CMD_START [{i + 1}/{args.start_retries}]")
            if i + 1 < args.start_retries and args.start_retry_interval > 0.0:
                time.sleep(args.start_retry_interval)

        if args.start_only:
            log.info("Start-only mode complete. Check ESP32 monitor for OFFBOARD mode confirmed and Armed confirmed.")
            return 0

        if args.takeoff_wait > 0.0:
            log.info("Waiting %.1f s before sending goal...", args.takeoff_wait)
            time.sleep(args.takeoff_wait)

        send_or_fail(
            node,
            args.drone_id,
            CommandPacket(CMD_GOTO, goal_x=args.goal_x, goal_y=args.goal_y),
            f"CMD_GOTO ({args.goal_x:.2f}, {args.goal_y:.2f})",
        )

        log.info("Done. Observe ESP32 monitor and QGroundControl for OFFBOARD/arm status.")
        return 0
    except KeyboardInterrupt:
        log.warning("Interrupted by user")
        return 130
    finally:
        node.stop()


if __name__ == "__main__":
    raise SystemExit(main())
