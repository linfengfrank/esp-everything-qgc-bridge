#!/usr/bin/env python3
"""Live view of a drone's ESP32 camera over WiFi.

    python3 laptop/camera_stream.py --esp-ip 192.168.1.222 [--fps 10] [--quality 60]

The drone streams only while this viewer sends keepalives (wire format:
main/camera_stream.c).  Keys: q/Esc quit, s save the current frame.
"""

from __future__ import annotations

import argparse
import select
import socket
import sys
import time
from dataclasses import dataclass, field

from protocol import (
    CAMERA_STREAM_MAX_FPS,
    CAMERA_STREAM_MAX_QUALITY,
    CAMERA_STREAM_MIN_QUALITY,
    CAMERA_STREAM_PORT,
    CAMERA_VERSION,
    CameraChunk,
    build_camera_stream_command,
    camera_packet_version,
    parse_camera_chunk,
)

COMMAND_PORT = 5006
KEEPALIVE_S = 1.0
FRAME_TIMEOUT_S = 0.75      # drop a frame still missing datagrams after this
MAX_JPEG_BYTES = 256 * 1024
MAX_PENDING = 16
REBOOT_BACKSTEP = 32        # an id this far below the last shown one = reboot
STATS_S = 2.0


@dataclass
class Frame:
    frame_id: int
    jpeg: bytes
    age_ms: int             # capture -> END sent, measured on the ESP32
    esp_drops: int
    first_rx: float         # when this frame's first datagram arrived


@dataclass
class _Pending:
    first_rx: float
    pieces: dict[int, bytes] = field(default_factory=dict)
    size: int | None = None


class FrameAssembler:
    """Reassembles one drone's JPEG frames from out-of-order datagrams.

    Returns only frames newer than the last one returned.  `dropped` counts
    frame ids skipped between returned frames (lost, incomplete or never
    sent).  A new boot_nonce, or a large step back in frame_id, means the
    drone rebooted and restarts the sequence.
    """

    def __init__(self) -> None:
        self.pending: dict[int, _Pending] = {}
        self.nonce: int | None = None
        self.last_id: int | None = None
        self.dropped = 0

    def add(self, c: CameraChunk, now: float) -> Frame | None:
        rebooted = (self.last_id is not None
                    and c.frame_id + REBOOT_BACKSTEP < self.last_id)
        if c.boot_nonce != self.nonce or rebooted:
            self.nonce, self.last_id = c.boot_nonce, None
            self.pending.clear()
        if self.last_id is not None and c.frame_id <= self.last_id:
            return None                             # late or duplicate

        p = self.pending.get(c.frame_id)
        if p is None:
            if len(self.pending) >= MAX_PENDING:
                del self.pending[min(self.pending)]
            p = self.pending[c.frame_id] = _Pending(now)
        if c.is_end:
            p.size = c.frame_size
        elif c.offset + len(c.payload) <= MAX_JPEG_BYTES:
            p.pieces[c.offset] = c.payload
        if p.size is None:
            return None
        parts, end = [], 0
        for offset in sorted(p.pieces):
            if offset != end:
                return None                         # a datagram is still missing
            parts.append(p.pieces[offset])
            end += len(p.pieces[offset])
        if end != p.size:
            return None
        if self.last_id is not None:
            self.dropped += c.frame_id - self.last_id - 1
        self.last_id = c.frame_id
        self.pending = {k: v for k, v in self.pending.items() if k > c.frame_id}
        return Frame(c.frame_id, b"".join(parts), c.age_ms, c.esp_drops, p.first_rx)

    def expire(self, now: float) -> None:
        for k in [k for k, p in self.pending.items()
                  if now - p.first_rx > FRAME_TIMEOUT_S]:
            del self.pending[k]


def drain_socket(sock: socket.socket, asm: FrameAssembler, esp_ip: str,
                 max_datagrams: int = 2000):
    """Read every queued datagram from esp_ip.

    Returns (newest frame or None, datagrams read, foreign ECAM version or
    None).  Frames completed earlier in the same pass are counted as dropped.
    """
    newest, count, bad_version = None, 0, None
    while count < max_datagrams:
        try:
            data, (ip, _) = sock.recvfrom(2048)
        except (BlockingIOError, InterruptedError):
            break
        count += 1
        if ip != esp_ip:
            continue
        chunk = parse_camera_chunk(data)
        if chunk is None:
            version = camera_packet_version(data)
            if version not in (None, CAMERA_VERSION):
                bad_version = version
            continue
        frame = asm.add(chunk, time.monotonic())
        if frame is not None:
            if newest is not None:
                asm.dropped += 1
            newest = frame
    return newest, count, bad_version


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--esp-ip", required=True,
                        help="drone IP, 192.168.1.(200 + drone id)")
    parser.add_argument("--fps", type=int, default=0,
                        help=f"1-{CAMERA_STREAM_MAX_FPS} (default: firmware, 10)")
    parser.add_argument("--quality", type=int, default=0,
                        help=f"JPEG quality {CAMERA_STREAM_MIN_QUALITY}-"
                             f"{CAMERA_STREAM_MAX_QUALITY} (default: firmware, 60)")
    parser.add_argument("--scale", type=float, default=2.0,
                        help="window scale (default: 2)")
    args = parser.parse_args()
    try:
        build_camera_stream_command(True, args.fps, args.quality)
    except ValueError as exc:
        parser.error(str(exc))
    if args.scale <= 0:
        parser.error("--scale must be positive")
    return args


def main() -> int:
    args = _parse_args()
    try:
        import cv2
        import numpy as np
    except ImportError:
        sys.exit("Missing dependencies: python3 -m pip install -r laptop/requirements.txt")

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    try:
        rx.bind(("0.0.0.0", CAMERA_STREAM_PORT))
    except OSError as exc:
        sys.exit(f"Cannot listen on UDP {CAMERA_STREAM_PORT}: {exc}")
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.esp_ip, COMMAND_PORT)
    keepalive = build_camera_stream_command(True, args.fps, args.quality)

    asm = FrameAssembler()
    window = "ESP32 camera"
    size = (round(320 * args.scale), round(240 * args.scale))
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.imshow(window, np.zeros((size[1], size[0]), np.uint8))
    print(f"Requesting {args.esp_ip}; q/Esc quits, s saves a frame.")

    next_keepalive = stats_start = time.monotonic()
    last_warning = float("-inf")
    warned_version = False
    shown = ages = latencies = 0
    image = None
    try:
        while True:
            now = time.monotonic()
            if now >= next_keepalive:
                next_keepalive = now + KEEPALIVE_S
                try:
                    tx.sendto(keepalive, target)
                except OSError as exc:        # e.g. drone off: keep trying
                    if now - last_warning > 5:
                        print(f"keepalive failed: {exc}", file=sys.stderr)
                        last_warning = now

            select.select([rx], [], [], 0.005)
            frame, _, bad_version = drain_socket(rx, asm, args.esp_ip)
            if bad_version is not None and not warned_version:
                warned_version = True
                print(f"drone sends ECAM v{bad_version}, viewer expects "
                      f"v{CAMERA_VERSION}: reflash the drone", file=sys.stderr)
            if frame is not None:
                gray = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8),
                                    cv2.IMREAD_GRAYSCALE)
                if gray is None:
                    asm.dropped += 1
                else:
                    image = gray
                    view = cv2.resize(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR),
                                      size, interpolation=cv2.INTER_NEAREST)
                    cv2.putText(view, f"#{frame.frame_id}  esp {frame.age_ms} ms  "
                                f"drops {asm.dropped}/{frame.esp_drops}",
                                (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 255, 0), 1, cv2.LINE_AA)
                    cv2.imshow(window, view)
                    shown += 1
                    ages += frame.age_ms
                    latencies += (time.monotonic() - frame.first_rx) * 1000

            now = time.monotonic()
            asm.expire(now)
            if now - stats_start >= STATS_S:
                n = max(shown, 1)
                print(f"{shown / (now - stats_start):4.1f} fps  "
                      f"esp age {ages / n:.0f} ms  laptop {latencies / n:.0f} ms  "
                      f"drops {asm.dropped}")
                stats_start, shown, ages, latencies = now, 0, 0, 0

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s") and image is not None:
                path = time.strftime("esp32_camera_%Y%m%d_%H%M%S.png")
                cv2.imwrite(path, image)
                print(f"saved {path}")
    except KeyboardInterrupt:
        pass
    finally:
        try:
            tx.sendto(build_camera_stream_command(False), target)
        except OSError:
            pass
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
