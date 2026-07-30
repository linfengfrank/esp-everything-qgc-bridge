#!/usr/bin/env python3
"""
Simple GCS mission: arm/takeoff -> fly through waypoints -> land.

This script uses the existing custom UDP protocol already implemented by the
laptop-side GCS:
  - CMD_START : arm + take off to the firmware's default cruise altitude
  - CMD_GOTO  : send a map-frame waypoint target
  - CMD_LAND  : land in place

Example:
    python laptop/simple_waypoint_mission.py \
        --drone-id 2 \
        --waypoint 0.0,0.0 \
        --waypoint 1.0,0.5 \
        --waypoint 2.0,0.0

    python laptop/simple_waypoint_mission.py \
        --drone-id 2 \
        --waypoints-file missions/waypoints.txt
"""

from __future__ import annotations

import argparse
import logging
import math
import signal
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from comms import CommsNode
from protocol import (
    CMD_GOTO,
    CMD_HOLD,
    CMD_LAND,
    CMD_START,
    NAV_ARRIVED,
    CommandPacket,
    TelemetryPacket,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("simple_waypoint_mission")


@dataclass
class TelemetrySnapshot:
    packet: Optional[TelemetryPacket] = None
    source_ip: Optional[str] = None
    received_at: float = 0.0


class TelemetryTracker:
    def __init__(self, drone_id: int, print_live: bool = False) -> None:
        self.drone_id = drone_id
        self._snapshot = TelemetrySnapshot()
        self._condition = threading.Condition()
        self._print_live = print_live

    def callback(self, packet: TelemetryPacket, source_ip: str) -> None:
        if packet.drone_id != self.drone_id:
            return

        with self._condition:
            self._snapshot = TelemetrySnapshot(
                packet=packet,
                source_ip=source_ip,
                received_at=time.monotonic(),
            )
            self._condition.notify_all()

            # Always show live position updates as soon as telemetry is received.
            #log.info(
            #    "Telemetry position: x=%.2f y=%.2f",
            #    packet.ned_x,
            #    packet.ned_y,
            #)

            if self._print_live:
                tag_text = "none" if packet.tag_id < 0 else str(packet.tag_id)
                log.info(
                    "TELEM drone=%d pos=(%.2f, %.2f) heading=%.2f state=%s tag=%s dist=%.2f stuck=%s reloc=%ds",
                    packet.drone_id,
                    packet.ned_x,
                    packet.ned_y,
                    packet.heading_rad,
                    packet.nav_state_name,
                    tag_text,
                    packet.tag_dist_m,
                    "yes" if packet.is_stuck else "no",
                    packet.reloc_age_s,
                )

    def latest(self) -> TelemetrySnapshot:
        with self._condition:
            return TelemetrySnapshot(
                packet=self._snapshot.packet,
                source_ip=self._snapshot.source_ip,
                received_at=self._snapshot.received_at,
            )

    def wait_for_first_packet(self, timeout_s: float) -> TelemetrySnapshot:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self._snapshot.packet is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        f"No telemetry from drone {self.drone_id} on UDP 5005."
                    )
                self._condition.wait(timeout=min(0.5, remaining))
            return self.latest()


def parse_waypoint(value: str) -> tuple[float, float]:
    parts = [item.strip() for item in value.split(",")]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError(
            "Waypoints must be given as x,y (for example 1.0,2.0)."
        )

    try:
        x_value = float(parts[0])
        y_value = float(parts[1])
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "Waypoint coordinates must be numeric."
        ) from exc

    return x_value, y_value


def load_waypoints_from_file(path: str) -> list[tuple[float, float]]:
    file_path = Path(path).expanduser()
    if not file_path.exists():
        raise FileNotFoundError(f"Waypoint file not found: {file_path}")

    waypoints: list[tuple[float, float]] = []
    for line_number, raw_line in enumerate(
        file_path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue

        if "," in line:
            waypoint = parse_waypoint(line)
        else:
            parts = line.split()
            if len(parts) != 2:
                raise ValueError(
                    f"Invalid waypoint format on line {line_number}: {raw_line}"
                )
            try:
                waypoint = (float(parts[0]), float(parts[1]))
            except ValueError as exc:
                raise ValueError(
                    f"Invalid waypoint coordinates on line {line_number}: {raw_line}"
                ) from exc

        waypoints.append(waypoint)

    return waypoints


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send a simple takeoff -> waypoint -> land mission from the laptop GCS"
    )
    parser.add_argument(
        "--drone-id",
        type=int,
        default=2,
        help="Drone ID to command (default: 2).",
    )
    parser.add_argument(
        "--config",
        default="setup.yaml",
        help="Path to setup.yaml (default: setup.yaml).",
    )
    parser.add_argument(
        "--telem-port",
        type=int,
        default=5005,
        help="Laptop telemetry UDP port (default: 5005).",
    )
    parser.add_argument(
        "--cmd-port",
        type=int,
        default=5006,
        help="ESP32 command UDP port (default: 5006).",
    )
    parser.add_argument(
        "--waypoint",
        dest="waypoints",
        action="append",
        type=parse_waypoint,
        default=[],
        help="Waypoint as x,y. Repeat this option for multiple waypoints.",
    )
    parser.add_argument(
        "--waypoints-file",
        help="Text file containing waypoints, one x,y entry per line (blank lines and # comments are allowed).",
    )
    parser.add_argument(
        "--takeoff-wait",
        type=float,
        default=3.0,
        help="Seconds to wait after CMD_START before sending the first waypoint.",
    )
    parser.add_argument(
        "--arrival-timeout",
        type=float,
        default=25.0,
        help="Seconds to wait for each waypoint to be reached before continuing.",
    )
    parser.add_argument(
        "--arrival-radius",
        type=float,
        default=0.25,
        help="Horizontal distance threshold (meters) to accept a waypoint as reached (default: 0.35).",
    )
    parser.add_argument(
        "--finish-action",
        choices=["hold", "land"],
        default="land",
        help="Action after final waypoint: hold current altitude/position or land (default: land).",
    )
    parser.add_argument(
        "--confirm",
        action="store_true",
        help="Require a confirmation prompt before sending any flight commands.",
    )
    parser.add_argument(
        "--live-telem",
        action="store_true",
        help="Print each incoming telemetry update from the drone.",
    )
    return parser.parse_args()


def send_command(comms: CommsNode, drone_id: int, command_type: int, name: str) -> None:
    ok = comms.send_command(drone_id, CommandPacket(command_type))
    if not ok:
        raise RuntimeError(f"Failed to send {name}: drone {drone_id} IP is unknown.")
    log.info("%s sent to drone %d.", name, drone_id)


def wait_for_arrival(
    tracker: TelemetryTracker,
    timeout_s: float,
    issued_after: float,
    goal_x: float,
    goal_y: float,
    arrival_radius_m: float,
) -> None:
    deadline = time.monotonic() + timeout_s
    saw_fresh_telemetry = False
    saw_non_arrived_state = False

    while time.monotonic() < deadline:
        snapshot = tracker.latest()
        packet = snapshot.packet

        if packet is not None and snapshot.received_at > issued_after:
            saw_fresh_telemetry = True
            dist_to_goal = math.hypot(packet.ned_x - goal_x, packet.ned_y - goal_y)

            log.info(
                "Current position: x=%.2f y=%.2f (goal x=%.2f y=%.2f, dist=%.2f m)",
                packet.ned_x,
                packet.ned_y,
                goal_x,
                goal_y,
                dist_to_goal,
            )

            if packet.nav_state != NAV_ARRIVED:
                saw_non_arrived_state = True

            # Require post-command telemetry so stale NAV_ARRIVED does not auto-pass.
            if dist_to_goal <= arrival_radius_m or (
                packet.nav_state == NAV_ARRIVED and saw_non_arrived_state
            ):
                log.info(
                    "Waypoint reached at (%.2f, %.2f), goal=(%.2f, %.2f), dist=%.2f m",
                    packet.ned_x,
                    packet.ned_y,
                    goal_x,
                    goal_y,
                    dist_to_goal,
                )
                return

        time.sleep(0.2)

    snapshot = tracker.latest()
    if snapshot.packet is not None:
        dist_to_goal = math.hypot(
            snapshot.packet.ned_x - goal_x,
            snapshot.packet.ned_y - goal_y,
        )
        log.warning(
            "Waypoint arrival timeout; fresh_telem=%s last nav_state=%s at (%.2f, %.2f), goal=(%.2f, %.2f), dist=%.2f m",
            "yes" if saw_fresh_telemetry else "no",
            snapshot.packet.nav_state_name,
            snapshot.packet.ned_x,
            snapshot.packet.ned_y,
            goal_x,
            goal_y,
            dist_to_goal,
        )
    else:
        log.warning("Waypoint arrival timeout; no telemetry received.")


def main() -> int:
    args = parse_args()

    if (
        args.takeoff_wait < 0.0
        or args.arrival_timeout <= 0.0
        or args.arrival_radius <= 0.0
    ):
        raise SystemExit(
            "Invalid timing values. Use non-negative takeoff wait, positive arrival timeout, and positive arrival radius."
        )

    waypoints = list(args.waypoints)
    if args.waypoints_file:
        waypoints.extend(load_waypoints_from_file(args.waypoints_file))

    if not waypoints:
        raise SystemExit("No waypoints supplied. Use --waypoint values or --waypoints-file.")

    tracker = TelemetryTracker(args.drone_id, print_live=args.live_telem)
    comms = CommsNode(
        listen_port=args.telem_port,
        cmd_port=args.cmd_port,
        config_path=args.config if Path(args.config).exists() else None,
    )
    comms.on_telemetry(tracker.callback)

    def handle_interrupt(signum: int, frame: object) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, handle_interrupt)

    comms.start()
    try:
        log.info("Waiting for telemetry from drone %d...", args.drone_id)
        tracker.wait_for_first_packet(30.0)
        log.info("Telemetry link established.")

        if args.confirm:
            confirmation = input(
                f"Type MISSION-{args.drone_id} to start the mission: "
            ).strip()
            if confirmation != f"MISSION-{args.drone_id}":
                log.info("Confirmation did not match. Mission cancelled.")
                return 0

        log.info("Sending CMD_START (arm + takeoff)")
        send_command(comms, args.drone_id, CMD_START, "CMD_START")
        if args.takeoff_wait > 0.0:
            time.sleep(args.takeoff_wait)

        for index, (goal_x, goal_y) in enumerate(waypoints, start=1):
            log.info(
                "Sending waypoint %d/%d to (%.2f, %.2f)",
                index,
                len(waypoints),
                goal_x,
                goal_y,
            )
            issued_after = time.monotonic()
            sent_ok = comms.send_command(
                args.drone_id,
                CommandPacket(CMD_GOTO, goal_x=goal_x, goal_y=goal_y),
            )
            if not sent_ok:
                raise RuntimeError(
                    f"Failed to send CMD_GOTO: drone {args.drone_id} IP is unknown."
                )

            wait_for_arrival(
                tracker,
                timeout_s=args.arrival_timeout,
                issued_after=issued_after,
                goal_x=goal_x,
                goal_y=goal_y,
                arrival_radius_m=args.arrival_radius,
            )
            time.sleep(0.5)

        if args.finish_action == "land":
            log.info("Sending CMD_LAND")
            send_command(comms, args.drone_id, CMD_LAND, "CMD_LAND")
        else:
            log.info("Sending CMD_HOLD")
            send_command(comms, args.drone_id, CMD_HOLD, "CMD_HOLD")
        return 0
    except KeyboardInterrupt:
        log.info("Mission interrupted. Sending CMD_LAND.")
        try:
            send_command(comms, args.drone_id, CMD_LAND, "CMD_LAND")
        except Exception:
            pass
        return 130
    finally:
        comms.stop()


if __name__ == "__main__":
    main()
