#!/usr/bin/env python3
"""
Check communication health between PX4 and ESP32.

The check observes two independent signals:
1) ESP32 custom telemetry packets on UDP 5005 (PKT_TELEM)
2) MAVLink HEARTBEAT packets on UDP 14550 forwarded by ESP32

Interpretation:
- Telemetry present + MAVLink heartbeat present: PX4<->ESP32 link is healthy.
- Telemetry present but no MAVLink heartbeat: ESP32 is reachable, PX4 UART/bridge likely unhealthy.
- MAVLink heartbeat present but no telemetry: bridge works, custom telemetry path likely unhealthy.
"""

from __future__ import annotations

import argparse
import select
import socket
import sys
import time
from dataclasses import dataclass, field

from protocol import TelemetryPacket, parse_telemetry


@dataclass
class CheckStats:
    start_ts: float
    telem_count: int = 0
    hb_count: int = 0
    telem_sources: set[str] = field(default_factory=set)
    hb_sysids: set[int] = field(default_factory=set)
    first_telem_ts: float | None = None
    first_hb_ts: float | None = None
    last_telem: TelemetryPacket | None = None


def parse_mavlink_heartbeats(payload: bytes) -> list[int]:
    """Return a list of sysids for HEARTBEAT messages found in a UDP payload."""
    sysids: list[int] = []
    i = 0
    n = len(payload)

    while i < n:
        magic = payload[i]

        # MAVLink v1
        if magic == 0xFE:
            if i + 6 > n:
                break
            length = payload[i + 1]
            frame_len = 6 + length + 2
            if i + frame_len > n:
                break

            msgid = payload[i + 5]
            sysid = payload[i + 3]
            if msgid == 0:
                sysids.append(sysid)
            i += frame_len
            continue

        # MAVLink v2
        if magic == 0xFD:
            if i + 10 > n:
                break
            length = payload[i + 1]
            incompat_flags = payload[i + 2]
            has_signature = (incompat_flags & 0x01) != 0
            signature_len = 13 if has_signature else 0
            frame_len = 10 + length + 2 + signature_len
            if i + frame_len > n:
                break

            msgid = payload[i + 7] | (payload[i + 8] << 8) | (payload[i + 9] << 16)
            sysid = payload[i + 5]
            if msgid == 0:
                sysids.append(sysid)
            i += frame_len
            continue

        i += 1

    return sysids


def open_udp_socket(port: int) -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", port))
    s.setblocking(False)
    return s


def run_check(duration_s: float, telem_port: int, mavlink_port: int, drone_id: int | None) -> int:
    stats = CheckStats(start_ts=time.monotonic())
    telem_sock = open_udp_socket(telem_port)
    mav_sock = open_udp_socket(mavlink_port)

    print(f"Listening {duration_s:.1f}s: telemetry UDP {telem_port}, MAVLink UDP {mavlink_port}")
    if drone_id is not None:
        print(f"Filtering telemetry for drone_id={drone_id}")

    end_ts = time.monotonic() + duration_s
    try:
        while time.monotonic() < end_ts:
            timeout = max(0.0, end_ts - time.monotonic())
            readable, _, _ = select.select([telem_sock, mav_sock], [], [], min(0.5, timeout))

            for sock in readable:
                data, (src_ip, _) = sock.recvfrom(4096)

                if sock is telem_sock:
                    pkt = parse_telemetry(data)
                    if pkt is None:
                        continue
                    if drone_id is not None and pkt.drone_id != drone_id:
                        continue

                    stats.telem_count += 1
                    stats.telem_sources.add(src_ip)
                    stats.last_telem = pkt
                    if stats.first_telem_ts is None:
                        stats.first_telem_ts = time.monotonic()
                        print(f"Telemetry detected from {src_ip} (drone_id={pkt.drone_id})")
                    continue

                if sock is mav_sock:
                    hb_sysids = parse_mavlink_heartbeats(data)
                    if not hb_sysids:
                        continue

                    stats.hb_count += len(hb_sysids)
                    stats.hb_sysids.update(hb_sysids)
                    if stats.first_hb_ts is None:
                        stats.first_hb_ts = time.monotonic()
                        print("MAVLink HEARTBEAT detected on bridge port")
                    continue
    finally:
        telem_sock.close()
        mav_sock.close()

    elapsed = max(1e-6, time.monotonic() - stats.start_ts)
    telem_rate = stats.telem_count / elapsed
    hb_rate = stats.hb_count / elapsed

    print("\n=== Link Check Summary ===")
    print(f"Duration: {elapsed:.1f}s")
    print(f"Telemetry packets: {stats.telem_count} ({telem_rate:.1f}/s)")
    print(f"Telemetry sources: {sorted(stats.telem_sources) if stats.telem_sources else 'none'}")
    if stats.last_telem is not None:
        print(
            "Last telemetry: "
            f"drone_id={stats.last_telem.drone_id} "
            f"pos=({stats.last_telem.ned_x:.2f},{stats.last_telem.ned_y:.2f}) "
            f"nav={stats.last_telem.nav_state_name}"
        )
    print(f"MAVLink heartbeats: {stats.hb_count} ({hb_rate:.1f}/s)")
    print(f"Heartbeat sysids: {sorted(stats.hb_sysids) if stats.hb_sysids else 'none'}")

    telem_ok = stats.telem_count > 0
    hb_ok = stats.hb_count > 0

    if telem_ok and hb_ok:
        print("\nRESULT: PASS - PX4 <-> ESP32 communication appears healthy.")
        return 0

    print("\nRESULT: FAIL")
    if not telem_ok and not hb_ok:
        print("No telemetry and no MAVLink heartbeat seen.")
        print("Check WiFi link, ESP32 power, laptop firewall, and configured laptop IP in sdkconfig.")
    elif telem_ok and not hb_ok:
        print("Telemetry is present, but no bridged MAVLink HEARTBEAT was seen.")
        print("Likely PX4 UART link or MAVLink bridge config issue on ESP32/PX4.")
    else:
        print("MAVLink HEARTBEAT is present, but no custom telemetry was seen.")
        print("Likely issue in ESP32 wifi_task custom telemetry path.")

    return 2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check PX4 <-> ESP32 communication via telemetry + MAVLink heartbeat"
    )
    parser.add_argument("--duration", type=float, default=10.0, help="Check duration in seconds")
    parser.add_argument("--telem-port", type=int, default=5005, help="ESP32 telemetry UDP port")
    parser.add_argument("--mavlink-port", type=int, default=14550, help="MAVLink bridge UDP listen port")
    parser.add_argument("--drone-id", type=int, default=None, help="Optional drone ID filter")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration <= 0.0:
        print("Duration must be > 0")
        return 1

    return run_check(
        duration_s=args.duration,
        telem_port=args.telem_port,
        mavlink_port=args.mavlink_port,
        drone_id=args.drone_id,
    )


if __name__ == "__main__":
    sys.exit(main())
