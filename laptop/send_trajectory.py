#!/usr/bin/env python3
"""
Fly a CSV trajectory smoothly on one drone.

The whole trajectory is uploaded first; the ESP32 then plays it on its own
20 Hz clock (position + velocity feedforward to PX4), so WiFi latency and
jitter cannot disturb the flight.

CSV: columns t,x,y,z (s, NED m); a header row and '#' comments are allowed.
The trajectory is flown relative to where the drone hovers when it starts.

Example:
    python3 laptop/send_trajectory.py --drone-id 2 --takeoff --confirm \
        --trajectory trajectory/circle_traj.csv
"""

import argparse
import logging
import random
import time
from pathlib import Path

import numpy as np

from comms import CommsNode
from protocol import (
    CMD_HOLD,
    CMD_LAND,
    CMD_START,
    NAV_TRAJ,
    TRAJ_MAX_PTS,
    CommandPacket,
    build_traj_packets,
    build_traj_start,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("send_trajectory")

UPLOAD_GAP_S = 0.025   # ESP drains commands at 10 Hz from a 6-packet UDP queue


def load_trajectory(path: str, dt: float) -> np.ndarray:
    """Read t,x,y,z, resample every dt, return offsets from the first point."""
    data = np.atleast_2d(np.genfromtxt(path, delimiter=",", comments="#"))
    data = data[~np.isnan(data).any(axis=1)]           # drops the header row
    if data.shape[0] < 2 or data.shape[1] < 4:
        raise SystemExit(f"{path}: need at least 2 rows of t,x,y,z")
    t = data[:, 0]
    if np.any(np.diff(t) <= 0):
        raise SystemExit(f"{path}: time column must be strictly increasing")
    tq = np.arange(t[0], t[-1] + 1e-9, dt)
    pts = np.column_stack([np.interp(tq, t, data[:, c]) for c in (1, 2, 3)])
    return pts - pts[0]


def main() -> None:
    ap = argparse.ArgumentParser(description="Upload a CSV trajectory to one drone and fly it")
    ap.add_argument("--drone-id", type=int, required=True)
    ap.add_argument("--trajectory", required=True, help="CSV with columns t,x,y,z (s, NED m)")
    ap.add_argument("--config", default=str(Path(__file__).with_name("setup.yaml")),
                    help="Fleet config (optional)")
    ap.add_argument("--dt", type=float, default=0.05,
                    help="Upload sample period (s); 0.05 = ESP nav loop rate")
    ap.add_argument("--max-speed", type=float, default=0.30,
                    help="Refuse faster trajectories (m/s); keep below MAV_CMD_SPEED_CAP_MS")
    ap.add_argument("--takeoff", action="store_true", help="Send CMD_START (arm + take off) first")
    ap.add_argument("--takeoff-wait", type=float, default=8.0,
                    help="Seconds from CMD_START to trajectory upload")
    ap.add_argument("--finish", choices=["land", "hold"], default="land",
                    help="Action after the trajectory")
    ap.add_argument("--confirm", action="store_true",
                    help="Require typing TRAJ-<drone id> before sending any flight commands")
    ap.add_argument("--telem-port", type=int, default=5005)
    ap.add_argument("--cmd-port", type=int, default=5006)
    args = ap.parse_args()

    dt_ms = round(args.dt * 1000)
    if not 1 <= dt_ms <= 1000:
        raise SystemExit("--dt must be 0.001-1.0 s")
    pts = load_trajectory(args.trajectory, dt_ms / 1000)
    duration = (len(pts) - 1) * dt_ms / 1000
    vmax = np.linalg.norm(np.diff(pts, axis=0), axis=1).max() * 1000 / dt_ms
    log.info("%s: %d points, %.1f s, max speed %.3f m/s",
             args.trajectory, len(pts), duration, vmax)
    if vmax > args.max_speed + 1e-6:
        raise SystemExit(f"max speed {vmax:.3f} m/s exceeds --max-speed {args.max_speed} m/s")
    if len(pts) > TRAJ_MAX_PTS:
        raise SystemExit(f"{len(pts)} points > {TRAJ_MAX_PTS}; use a larger --dt or a shorter trajectory")

    latest = {}

    def on_telemetry(pkt, _ip):
        if pkt.drone_id == args.drone_id:
            latest["pkt"] = pkt

    def wait_for(cond, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if "pkt" in latest and cond(latest["pkt"]):
                return True
            time.sleep(0.05)
        return False

    def send(cmd_type: int, name: str) -> None:
        if not comms.send_command(args.drone_id, CommandPacket(cmd_type)):
            raise RuntimeError(f"{name} not sent: drone {args.drone_id} IP unknown")
        log.info("%s sent", name)

    comms = CommsNode(
        listen_port=args.telem_port,
        cmd_port=args.cmd_port,
        config_path=args.config if Path(args.config).exists() else None,
    )
    comms.on_telemetry(on_telemetry)
    comms.start()
    flying = not args.takeoff   # without --takeoff the drone must already hover
    try:
        if not wait_for(lambda p: True, 30.0):
            raise RuntimeError(f"no telemetry from drone {args.drone_id}")

        if args.confirm:
            answer = input(f"Type TRAJ-{args.drone_id} to fly {args.trajectory}: ").strip()
            if answer != f"TRAJ-{args.drone_id}":
                log.info("confirmation did not match; trajectory cancelled")
                return

        if args.takeoff:
            send(CMD_START, "CMD_START")
            flying = True
            time.sleep(args.takeoff_wait)

        # Upload, then start; retry if the drone did not switch to TRAJ.
        traj_id = random.randint(1, 255)
        packets = build_traj_packets(traj_id, pts)
        start = build_traj_start(traj_id, dt_ms)
        for attempt in range(1, 6):
            for p in packets:
                comms.send_raw(args.drone_id, p)
                time.sleep(UPLOAD_GAP_S)
            comms.send_raw(args.drone_id, start)
            if wait_for(lambda p: p.nav_state == NAV_TRAJ, 1.5):
                break
            log.warning("attempt %d: trajectory not started (see ESP log)", attempt)
        else:
            raise RuntimeError("drone refused the trajectory")

        log.info("playing trajectory %d (%.1f s)", traj_id, duration)
        end = time.monotonic() + duration + 10.0
        while latest["pkt"].nav_state == NAV_TRAJ and time.monotonic() < end:
            p = latest["pkt"]
            log.info("pos (%.2f, %.2f)", p.ned_x, p.ned_y)
            time.sleep(1.0)
        log.info("trajectory ended, nav state %s", latest["pkt"].nav_state_name)

        if args.finish == "land":
            for _ in range(3):   # UDP: repeat LAND
                send(CMD_LAND, "CMD_LAND")
        else:
            send(CMD_HOLD, "CMD_HOLD")
    except (Exception, KeyboardInterrupt):
        if flying:
            log.error("aborted — sending CMD_LAND")
            for _ in range(3):
                comms.send_command(args.drone_id, CommandPacket(CMD_LAND))
        raise
    finally:
        comms.stop()


if __name__ == "__main__":
    main()
