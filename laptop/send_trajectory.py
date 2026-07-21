#!/usr/bin/env python3
"""
Send a trajectory file to one UAV as a sequence of CMD_GOTO commands.

Supported trajectory file formats:
  1) CSV with two columns: x,y
  2) Plain text with one point per line: x y

Example:
    python laptop/send_trajectory.py --config laptop/setup.yaml --drone-id 0 --trajectory my_path.txt --delay 1.0
"""

import argparse
import csv
import logging
import sys
import time
from pathlib import Path

from comms import CommsNode
from protocol import CMD_GOTO, CommandPacket


logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
log = logging.getLogger("trajectory_sender")


def load_trajectory(path: str | Path) -> list[tuple[float, float]]:
    """Load waypoints from a CSV or plain-text trajectory file."""
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"trajectory file not found: {p}")

    pts: list[tuple[float, float]] = []

    if p.suffix.lower() == ".csv":
        with p.open("r", newline="") as fh:
            reader = csv.reader(fh)
            for row_num, row in enumerate(reader, 1):
                if not row or not row[0].strip() or row[0].lstrip().startswith("#"):
                    continue
                if len(row) < 2:
                    raise ValueError(f"bad CSV row {row_num}: expected x,y")
                try:
                    x = float(row[0])
                    y = float(row[1])
                except ValueError as exc:
                    raise ValueError(f"bad numeric value at row {row_num}") from exc
                pts.append((x, y))
    else:
        with p.open("r") as fh:
            for line_num, line in enumerate(fh, 1):
                line = line.split("#", 1)[0].strip()
                if not line:
                    continue
                parts = line.replace(",", " ").split()
                if len(parts) < 2:
                    raise ValueError(f"bad trajectory line {line_num}: expected x y")
                try:
                    x = float(parts[0])
                    y = float(parts[1])
                except ValueError as exc:
                    raise ValueError(f"bad numeric value at line {line_num}") from exc
                pts.append((x, y))

    if not pts:
        raise ValueError(f"trajectory file contains no valid points: {p}")
    return pts


def wait_for_drone(comms: CommsNode, drone_id: int, timeout_s: float) -> None:
    """Block until the drone has spoken and its IP is known."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if drone_id in comms.known_drones():
            return
        time.sleep(0.2)

    if drone_id not in comms.known_drones():
        raise RuntimeError(
            f"drone {drone_id} did not appear on the telemetry channel within {timeout_s:.1f}s"
        )


def send_trajectory(
    comms: CommsNode,
    drone_id: int,
    trajectory: list[tuple[float, float]],
    delay_s: float = 1.0,
    wait_for_drone_s: float = 10.0,
) -> None:
    """Send all waypoints to the drone as repeated CMD_GOTO packets."""
    if not trajectory:
        raise ValueError("trajectory is empty")

    wait_for_drone(comms, drone_id, wait_for_drone_s)

    for idx, (x, y) in enumerate(trajectory, 1):
        log.info("sending point %d/%d to drone %d -> (%.3f, %.3f)", idx, len(trajectory), drone_id, x, y)
        ok = comms.send_command(drone_id, CommandPacket(CMD_GOTO, goal_x=x, goal_y=y))
        if not ok:
            raise RuntimeError(f"failed to send point {idx}/{len(trajectory)} to drone {drone_id}")

        if idx < len(trajectory) and delay_s > 0:
            time.sleep(delay_s)


def main() -> None:
    parser = argparse.ArgumentParser(description="Upload a trajectory file to one UAV")
    parser.add_argument("--config", default="laptop/setup.yaml", help="Path to the GCS config YAML")
    parser.add_argument("--drone-id", type=int, required=True, help="Drone ID to target")
    parser.add_argument("--trajectory", required=True, help="Path to the trajectory file")
    parser.add_argument("--delay", type=float, default=1.0, help="Delay between waypoints in seconds")
    parser.add_argument("--telem-port", type=int, default=5005, help="Telemetry listen port")
    parser.add_argument("--cmd-port", type=int, default=5006, help="Command send port")
    parser.add_argument("--wait-for-drone-s", type=float, default=10.0, help="How long to wait for drone telemetry")
    args = parser.parse_args()

    trajectory = load_trajectory(args.trajectory)
    log.info("loaded %d waypoints from %s", len(trajectory), args.trajectory)

    comms = CommsNode(
        listen_port=args.telem_port,
        cmd_port=args.cmd_port,
        config_path=args.config,
    )
    comms.start()
    try:
        send_trajectory(
            comms,
            drone_id=args.drone_id,
            trajectory=trajectory,
            delay_s=args.delay,
            wait_for_drone_s=args.wait_for_drone_s,
        )
        log.info("trajectory upload complete")
    finally:
        comms.stop()


if __name__ == "__main__":
    main()