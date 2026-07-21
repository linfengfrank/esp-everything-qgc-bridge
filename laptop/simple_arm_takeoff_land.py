#!/usr/bin/env python3
"""
Simple arm -> takeoff -> hover -> land test for the EXISTING Python GCS.

This script uses the project's custom UDP protocol:
    ESP32 -> laptop telemetry: UDP 5005
    laptop -> ESP32 commands:  UDP 5006

It must be placed in the project's laptop/ directory beside:
    comms.py, protocol.py, setup.yaml

Current firmware behavior:
    CMD_START = switch to Offboard + arm + take off to CRUISE_ALT_M (0.5 m)
    CMD_HOLD  = cancel navigation and hold current position
    CMD_LAND  = land in place

The custom telemetry packet does not contain altitude, armed state, or PX4 mode.
For the first test, use QGroundControl and the ESP32 serial monitor to verify
those states.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

try:
    from comms import CommsNode
    from protocol import (
        CMD_HOLD,
        CMD_LAND,
        CMD_START,
        CommandPacket,
        TelemetryPacket,
    )
except ImportError as exc:
    raise SystemExit(
        "Cannot import comms.py and protocol.py.\n"
        "Copy this file into the project's laptop directory."
    ) from exc


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("simple_flight_test")


@dataclass
class TelemetrySnapshot:
    packet: Optional[TelemetryPacket] = None
    source_ip: Optional[str] = None
    received_at: float = 0.0


class TelemetryTracker:
    """Thread-safe storage for one drone's latest custom telemetry."""

    def __init__(self, drone_id: int) -> None:
        self.drone_id = drone_id
        self._snapshot = TelemetrySnapshot()
        self._condition = threading.Condition()

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

        #log.info("Received telemetry from drone %d at %s", self.drone_id, source_ip)

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


def send_command(
    comms: CommsNode,
    drone_id: int,
    command_type: int,
    name: str,
) -> None:
    """Send a custom command after the drone IP has been learned."""
    ok = comms.send_command(drone_id, CommandPacket(command_type))
    if not ok:
        raise RuntimeError(
            f"Failed to send {name}: drone {drone_id} IP is unknown."
        )
    log.info("%s sent to drone %d.", name, drone_id)


def wait_and_report(
    tracker: TelemetryTracker,
    duration_s: float,
    phase: str,
    stale_timeout_s: float,
) -> None:
    """Wait while reporting horizontal position and checking telemetry age."""
    deadline = time.monotonic() + duration_s

    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return

        snapshot = tracker.latest()
        if snapshot.packet is None:
            raise ConnectionError("Telemetry disappeared.")

        age = time.monotonic() - snapshot.received_at
        if age > stale_timeout_s:
            raise ConnectionError(
                f"Telemetry is stale ({age:.1f} s) during {phase}."
            )

        packet = snapshot.packet
        log.info(
            "%s | remaining %.0f s | map=(%.2f, %.2f) | nav=%s | age=%.2f s",
            phase,
            remaining,
            packet.ned_x,
            packet.ned_y,
            packet.nav_state_name,
            age,
        )
        time.sleep(min(1.0, remaining))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Arm, take off, hover, and land using the existing custom GCS."
    )
    parser.add_argument(
        "--drone-id",
        type=int,
        default=2,
        help="ESP32 CONFIG_DRONE_ID. Current sdkconfig value: 2.",
    )
    parser.add_argument(
        "--config",
        default="setup.yaml",
        help="Path to setup.yaml. Default: setup.yaml.",
    )
    parser.add_argument(
        "--telem-port",
        type=int,
        default=5005,
        help="Laptop telemetry UDP port. Default: 5005.",
    )
    parser.add_argument(
        "--cmd-port",
        type=int,
        default=5006,
        help="ESP32 command UDP port. Default: 5006.",
    )
    parser.add_argument(
        "--takeoff-wait",
        type=float,
        default=12.0,
        help=(
            "Seconds allowed for Offboard, arming, and takeoff before HOLD. "
            "Default: 12."
        ),
    )
    parser.add_argument(
        "--hover-time",
        type=float,
        default=5.0,
        help="Seconds to hold before landing. Default: 5.",
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for the first telemetry packet.",
    )
    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=3.0,
        help="Abort if telemetry is older than this value.",
    )
    parser.add_argument(
        "--monitor-only",
        action="store_true",
        help="Check communication without sending flight commands.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.takeoff_wait < 1.0 or args.hover_time < 0.0:
        raise SystemExit("Invalid takeoff or hover duration.")

    config_path: Optional[str] = args.config
    if not Path(args.config).exists():
        log.warning(
            "%s was not found. Continuing without automatic nav-tag setup.",
            args.config,
        )
        config_path = None

    tracker = TelemetryTracker(args.drone_id)
    comms = CommsNode(
        listen_port=args.telem_port,
        cmd_port=args.cmd_port,
        config_path=config_path,
    )
    comms.on_telemetry(tracker.callback)

    start_sent = False
    land_sent = False

    def handle_interrupt(signum: int, frame: object) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, handle_interrupt)
    comms.start()

    try:
        log.info(
            "Waiting for drone %d telemetry on UDP %d...",
            args.drone_id,
            args.telem_port,
        )
        snapshot = tracker.wait_for_first_packet(args.connect_timeout)
        assert snapshot.packet is not None

        log.info(
            "Connected: drone=%d, ESP32_IP=%s, map=(%.2f, %.2f), nav=%s",
            snapshot.packet.drone_id,
            snapshot.source_ip,
            snapshot.packet.ned_x,
            snapshot.packet.ned_y,
            snapshot.packet.nav_state_name,
        )

        if args.monitor_only:
            log.info("Monitor-only mode: no flight commands will be sent.")
            wait_and_report(
                tracker,
                duration_s=15.0,
                phase="MONITOR",
                stale_timeout_s=args.stale_timeout,
            )
            return 0

        print(
            "\nThe current firmware maps CMD_START to:\n"
            "  OFFBOARD mode -> arm -> take off to 0.5 m\n\n"
            "Before continuing:\n"
            "  1. Validate communication with --monitor-only.\n"
            "  2. For a bench test, remove all propellers.\n"
            "  3. For flight, clear the area and prepare supervised manual takeover.\n"
            "  4. Confirm all ToF sensors are healthy in the ESP32 monitor.\n"
            "  5. Confirm PX4 local position is valid.\n"
            "  6. Keep QGroundControl open for observation only.\n"
        )

        confirmation = input(
            f"Type ARM-{args.drone_id} to start the test: "
        ).strip()
        if confirmation != f"ARM-{args.drone_id}":
            log.info("Confirmation did not match. No flight command was sent.")
            return 0

        # CMD_START is the existing protocol's combined arm-and-takeoff command.
        send_command(comms, args.drone_id, CMD_START, "CMD_START")
        start_sent = True

        wait_and_report(
            tracker,
            duration_s=args.takeoff_wait,
            phase="START/TAKEOFF",
            stale_timeout_s=args.stale_timeout,
        )

        # Ensure no navigation goal remains active and capture a hold setpoint.
        send_command(comms, args.drone_id, CMD_HOLD, "CMD_HOLD")

        wait_and_report(
            tracker,
            duration_s=args.hover_time,
            phase="HOLD",
            stale_timeout_s=args.stale_timeout,
        )

        send_command(comms, args.drone_id, CMD_LAND, "CMD_LAND")
        land_sent = True

        log.info(
            "Landing requested. Verify descent, touchdown, and disarming in "
            "QGroundControl or the ESP32 serial monitor."
        )
        time.sleep(2.0)
        return 0

    except KeyboardInterrupt:
        log.warning("Ctrl+C received.")
        if start_sent and not land_sent:
            try:
                send_command(comms, args.drone_id, CMD_LAND, "fallback CMD_LAND")
                time.sleep(1.0)
            except Exception as exc:
                log.error("Could not send fallback LAND: %s", exc)
        return 130

    except Exception as exc:
        log.error("Test aborted: %s", exc)
        if start_sent and not land_sent:
            try:
                send_command(comms, args.drone_id, CMD_LAND, "fallback CMD_LAND")
                time.sleep(1.0)
            except Exception as land_exc:
                log.error("Could not send fallback LAND: %s", land_exc)
        return 1

    finally:
        comms.stop()


if __name__ == "__main__":
    sys.exit(main())
