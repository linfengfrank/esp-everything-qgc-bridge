#!/usr/bin/env python3
"""
Simple GCS mission: arm/takeoff -> fly through waypoints -> land.

Custom UDP commands (see protocol.py):
  - CMD_START : arm + take off to the firmware's cruise altitude
  - CMD_GOTO  : fly to a map-frame (x, y); resent until the drone acts on it
  - CMD_LAND  : land in place

Waypoints are relative to the takeoff point unless --map-frame is given,
in which case they are arena coordinates and setup.yaml's start offset is sent.

Example:
    python3 laptop/simple_waypoint_mission.py --drone-id 22 --confirm \
        --waypoint 0.5,0.0 --waypoint=-0.5,0.5

    python3 laptop/simple_waypoint_mission.py --drone-id 22 --confirm \
        --waypoints-file waypoints/waypoints_example.txt

Exit code: 0 all waypoints reached, 1 error or waypoint timeout, 130 Ctrl+C.
"""

from __future__ import annotations

import argparse
import logging
import math
import signal
import sys
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
    NAV_FLYING,
    NAV_ROTATING,
    NAV_STUCK,
    CommandPacket,
    TelemetryPacket,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("simple_waypoint_mission")

GOTO_RESEND_S = 2.0   # resend CMD_GOTO if the drone hasn't started on it


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

            if self._print_live:
                tag_text = "none" if packet.tag_id < 0 else str(packet.tag_id)
                log.info(
                    "TELEM drone=%d pos=(%.2f, %.2f) heading=%.2f state=%s tag=%s dist=%.2f stuck=%s",
                    packet.drone_id,
                    packet.ned_x,
                    packet.ned_y,
                    packet.heading_rad,
                    packet.nav_state_name,
                    tag_text,
                    packet.tag_dist_m,
                    "yes" if packet.is_stuck else "no",
                )

    def latest(self) -> TelemetrySnapshot:
        with self._condition:
            return TelemetrySnapshot(
                packet=self._snapshot.packet,
                source_ip=self._snapshot.source_ip,
                received_at=self._snapshot.received_at,
            )

    def wait_for_first_packet(self, timeout_s: float, port: int, heard) -> TelemetrySnapshot:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self._snapshot.packet is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        f"No telemetry from drone {self.drone_id} on UDP {port} "
                        f"(heard drones: {sorted(heard()) or 'none'})."
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


def join_waypoint_values(argv: list[str]) -> list[str]:
    """Turn '--waypoint -1,0' into '--waypoint=-1,0' so argparse accepts it."""
    out, it = [], iter(argv)
    for arg in it:
        out.append(f"--waypoint={next(it, '')}" if arg == "--waypoint" else arg)
    return out


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send a simple takeoff -> waypoint -> land mission from the laptop GCS"
    )
    parser.add_argument(
        "--drone-id",
        type=int,
        required=True,
        help="Flashed ESP32 CONFIG_DRONE_ID.",
    )
    parser.add_argument(
        "--map-frame",
        action="store_true",
        help="Treat waypoints as arena coordinates: send the drone's setup.yaml start "
             "offset. Default: waypoints are relative to the takeoff point.",
    )
    parser.add_argument(
        "--config",
        default=str(Path(__file__).with_name("setup.yaml")),
        help="Path to setup.yaml, used only with --map-frame (default: laptop/setup.yaml).",
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
        default=10.0,
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
        help="Horizontal distance threshold (meters) to accept a waypoint as reached (default: 0.25).",
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for the first telemetry packet (default: 30).",
    )
    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=3.0,
        help="Land if telemetry is older than this during the mission (default: 3).",
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
    return parser.parse_args(join_waypoint_values(sys.argv[1:]))


def send_command(comms: CommsNode, drone_id: int, command_type: int, name: str,
                 repeat: int = 1) -> None:
    for _ in range(repeat):
        if not comms.send_command(drone_id, CommandPacket(command_type)):
            raise RuntimeError(f"Failed to send {name}: drone {drone_id} IP is unknown.")
        if repeat > 1:
            time.sleep(0.1)
    log.info("%s sent to drone %d.", name, drone_id)


def send_goto(comms: CommsNode, drone_id: int, goal_x: float, goal_y: float) -> None:
    if not comms.send_command(drone_id, CommandPacket(CMD_GOTO, goal_x=goal_x, goal_y=goal_y)):
        raise RuntimeError(f"Failed to send CMD_GOTO: drone {drone_id} IP is unknown.")


def wait_for_arrival(
    tracker: TelemetryTracker,
    resend,
    timeout_s: float,
    stale_timeout_s: float,
    goal_x: float,
    goal_y: float,
    arrival_radius_m: float,
) -> bool:
    start = last_sent = time.monotonic()
    deadline = start + timeout_s
    saw_moving = False           # drone reported ROTATING/FLYING/STUCK

    while time.monotonic() < deadline:
        now = time.monotonic()
        snapshot = tracker.latest()
        packet = snapshot.packet
        age = now - snapshot.received_at
        if age > stale_timeout_s:
            raise ConnectionError(f"Telemetry is stale ({age:.1f} s).")

        if snapshot.received_at > start:
            dist_to_goal = math.hypot(packet.ned_x - goal_x, packet.ned_y - goal_y)

            log.info(
                "Current position: x=%.2f y=%.2f (goal x=%.2f y=%.2f, dist=%.2f m)",
                packet.ned_x,
                packet.ned_y,
                goal_x,
                goal_y,
                dist_to_goal,
            )

            if packet.nav_state in (NAV_ROTATING, NAV_FLYING, NAV_STUCK):
                saw_moving = True

            if dist_to_goal <= arrival_radius_m:
                log.info(
                    "Waypoint reached at (%.2f, %.2f), goal=(%.2f, %.2f), dist=%.2f m",
                    packet.ned_x,
                    packet.ned_y,
                    goal_x,
                    goal_y,
                    dist_to_goal,
                )
                return True

            if not saw_moving and now - last_sent >= GOTO_RESEND_S:
                log.info("Drone not moving yet (nav=%s) — resending CMD_GOTO", packet.nav_state_name)
                resend()
                last_sent = now

        time.sleep(0.2)

    packet = tracker.latest().packet
    log.warning(
        "Waypoint arrival timeout; last nav_state=%s at (%.2f, %.2f), goal=(%.2f, %.2f), dist=%.2f m",
        packet.nav_state_name,
        packet.ned_x,
        packet.ned_y,
        goal_x,
        goal_y,
        math.hypot(packet.ned_x - goal_x, packet.ned_y - goal_y),
    )
    return False


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

    config_path: Optional[str] = args.config if args.map_frame else None
    if config_path and not Path(config_path).exists():
        raise SystemExit(f"--map-frame needs {config_path}, which was not found.")

    tracker = TelemetryTracker(args.drone_id, print_live=args.live_telem)
    comms = CommsNode(
        listen_port=args.telem_port,
        cmd_port=args.cmd_port,
        config_path=config_path,
    )
    if config_path is None:
        # Start offset (0, 0) so waypoints are relative to takeoff; this also
        # clears any offset a previous setup.yaml run left on the drone.
        comms.set_nav_tags([], {})
        log.info("Waypoints are relative to the takeoff point (start offset 0, 0).")
    else:
        log.info("Waypoints are in the arena frame (start offset from %s).", config_path)
    comms.on_telemetry(tracker.callback)

    def handle_interrupt(signum: int, frame: object) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, handle_interrupt)

    start_sent = land_sent = aborted = False
    comms.start()
    try:
        log.info("Waiting for telemetry from drone %d...", args.drone_id)
        tracker.wait_for_first_packet(args.connect_timeout, args.telem_port, comms.known_drones)
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
        start_sent = True
        if args.takeoff_wait > 0.0:
            time.sleep(args.takeoff_wait)

        missed = 0
        for index, (goal_x, goal_y) in enumerate(waypoints, start=1):
            log.info(
                "Sending waypoint %d/%d to (%.2f, %.2f)",
                index,
                len(waypoints),
                goal_x,
                goal_y,
            )

            def resend(gx: float = goal_x, gy: float = goal_y) -> None:
                send_goto(comms, args.drone_id, gx, gy)

            resend()
            if not wait_for_arrival(
                tracker,
                resend,
                timeout_s=args.arrival_timeout,
                stale_timeout_s=args.stale_timeout,
                goal_x=goal_x,
                goal_y=goal_y,
                arrival_radius_m=args.arrival_radius,
            ):
                missed += 1
                send_command(comms, args.drone_id, CMD_HOLD, "CMD_HOLD")   # stop before next leg
            time.sleep(0.5)

        if args.finish_action == "land":
            log.info("Sending CMD_LAND")
            send_command(comms, args.drone_id, CMD_LAND, "CMD_LAND", repeat=3)
            land_sent = True
        else:
            log.info("Sending CMD_HOLD")
            send_command(comms, args.drone_id, CMD_HOLD, "CMD_HOLD")

        if missed:
            log.warning("%d of %d waypoints not reached.", missed, len(waypoints))
            return 1
        return 0
    except KeyboardInterrupt:
        aborted = True
        log.warning("Mission interrupted.")
        return 130
    except Exception as exc:
        aborted = True
        log.error("Mission aborted: %s", exc)
        return 1
    finally:
        if aborted and start_sent and not land_sent:
            try:
                send_command(comms, args.drone_id, CMD_LAND, "fallback CMD_LAND", repeat=3)
            except Exception as exc:
                log.error("Could not send fallback LAND: %s", exc)
        comms.stop()


if __name__ == "__main__":
    sys.exit(main())
