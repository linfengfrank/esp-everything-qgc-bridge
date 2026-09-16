#!/usr/bin/env python3
"""View the ESP32-S3 camera over the project's WiFi link.

The firmware keeps the sensor in QVGA grayscale mode for AprilTag detection.
When this program sends a keepalive to the ESP32, completed detector frames
are JPEG-compressed and sent as MTU-safe UDP chunks on port 5009.

Example:

    python3 laptop/camera_stream.py --esp-ip 192.168.1.222

The ESP32 IP is printed as ``Got IP`` on its serial console.  The computer
running this script must be CONFIG_HOST_IPV4_ADDR in the ESP32 configuration.

Keys:
    q or Escape  close the viewer
    s            save the current (unannotated) frame
"""

from __future__ import annotations

import argparse
import socket
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

from protocol import (
    CAMERA_STREAM_PORT,
    CameraChunk,
    build_camera_stream_command,
    parse_camera_chunk,
)

COMMAND_PORT = 5006
KEEPALIVE_INTERVAL_S = 1.0
ASSEMBLY_TIMEOUT_S = 1.5
MAX_JPEG_BYTES = 512 * 1024


@dataclass
class _PendingFrame:
    drone_id: int
    frame_id: int
    width: int
    height: int
    source: tuple[str, int]
    updated_at: float
    pieces: dict[int, bytes] = field(default_factory=dict)
    expected_size: int | None = None


@dataclass
class CompleteFrame:
    drone_id: int
    frame_id: int
    width: int
    height: int
    source: tuple[str, int]
    jpeg: bytes


class FrameAssembler:
    """Reassemble out-of-order JPEG chunks and discard incomplete frames."""

    def __init__(self, timeout_s: float = ASSEMBLY_TIMEOUT_S):
        self.timeout_s = timeout_s
        self.pending: dict[tuple[str, int, int], _PendingFrame] = {}
        self.dropped_frames = 0

    def expire(self, now: float) -> None:
        expired = [key for key, frame in self.pending.items()
                   if now - frame.updated_at > self.timeout_s]
        for key in expired:
            del self.pending[key]
            self.dropped_frames += 1

    def add(self, chunk: CameraChunk, source: tuple[str, int],
            now: float) -> CompleteFrame | None:
        self.expire(now)

        key = (source[0], chunk.drone_id, chunk.frame_id)
        frame = self.pending.get(key)
        if frame is None:
            frame = _PendingFrame(
                drone_id=chunk.drone_id,
                frame_id=chunk.frame_id,
                width=chunk.width,
                height=chunk.height,
                source=source,
                updated_at=now,
            )
            self.pending[key] = frame

        if (frame.width != chunk.width or frame.height != chunk.height
                or chunk.offset > MAX_JPEG_BYTES
                or chunk.offset + len(chunk.payload) > MAX_JPEG_BYTES):
            del self.pending[key]
            self.dropped_frames += 1
            return None

        frame.updated_at = now
        if chunk.is_end:
            if chunk.frame_size > MAX_JPEG_BYTES:
                del self.pending[key]
                self.dropped_frames += 1
                return None
            frame.expected_size = chunk.frame_size
        elif chunk.payload:
            frame.pieces[chunk.offset] = chunk.payload

        return self._complete(key, frame)

    def _complete(self, key: tuple[str, int, int],
                  frame: _PendingFrame) -> CompleteFrame | None:
        if frame.expected_size is None:
            return None

        offset = 0
        ordered = []
        for piece_offset in sorted(frame.pieces):
            piece = frame.pieces[piece_offset]
            if piece_offset != offset:
                return None
            ordered.append(piece)
            offset += len(piece)

        if offset != frame.expected_size:
            return None

        jpeg = b"".join(ordered)
        del self.pending[key]
        return CompleteFrame(
            drone_id=frame.drone_id,
            frame_id=frame.frame_id,
            width=frame.width,
            height=frame.height,
            source=frame.source,
            jpeg=jpeg,
        )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Display the ESP32 QVGA camera stream over UDP")
    parser.add_argument(
        "--esp-ip",
        help="ESP32 IP from its serial 'Got IP' message (required unless --listen-only)",
    )
    parser.add_argument(
        "--listen-only", action="store_true",
        help="Do not send stream keepalives (use only if another client enabled it)",
    )
    parser.add_argument("--bind", default="0.0.0.0",
                        help="Local interface to bind (default: all)")
    parser.add_argument("--port", type=int, default=CAMERA_STREAM_PORT,
                        help=f"Local stream port (default: {CAMERA_STREAM_PORT})")
    parser.add_argument("--command-port", type=int, default=COMMAND_PORT,
                        help=f"ESP32 command port (default: {COMMAND_PORT})")
    parser.add_argument("--drone-id", type=int,
                        help="Ignore frames from other drone IDs")
    parser.add_argument("--scale", type=float, default=2.0,
                        help="Display scale factor (default: 2.0)")
    parser.add_argument("--save-dir", type=Path, default=Path("camera_frames"),
                        help="Directory used by the 's' key")
    args = parser.parse_args()

    if not args.listen_only and not args.esp_ip:
        parser.error("--esp-ip is required unless --listen-only is used")
    if args.scale <= 0:
        parser.error("--scale must be greater than zero")
    if args.drone_id is not None and not 0 <= args.drone_id <= 255:
        parser.error("--drone-id must be between 0 and 255")
    return args


def main() -> int:
    args = _parse_args()

    try:
        import cv2
        import numpy as np
    except ImportError:
        print("Missing viewer dependencies. Install them with:\n"
              "  python3 -m pip install -r 'laptop/requirements(1).txt'",
              file=sys.stderr)
        return 2

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
    try:
        rx.bind((args.bind, args.port))
    except OSError as exc:
        print(f"Cannot bind UDP {args.bind}:{args.port}: {exc}", file=sys.stderr)
        return 2
    rx.settimeout(0.05)

    control = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    control_target = ((args.esp_ip, args.command_port)
                      if not args.listen_only else None)
    enable_packet = build_camera_stream_command(True)
    disable_packet = build_camera_stream_command(False)

    assembler = FrameAssembler()
    window = "ESP32 camera preview"
    next_keepalive = 0.0
    last_frame_time: float | None = None
    smoothed_fps = 0.0
    completed_frames = 0
    last_image = None

    waiting = np.zeros((240, 640, 3), dtype=np.uint8)
    cv2.putText(waiting, "Waiting for ESP32 camera stream...", (35, 105),
                cv2.FONT_HERSHEY_SIMPLEX, 0.72, (220, 220, 220), 2,
                cv2.LINE_AA)
    cv2.putText(waiting, f"UDP :{args.port}  |  q/Esc to quit", (35, 145),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (150, 150, 150), 1,
                cv2.LINE_AA)

    try:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.imshow(window, waiting)
    except cv2.error as exc:
        print(f"OpenCV could not open a display window: {exc}", file=sys.stderr)
        rx.close()
        control.close()
        return 2

    target_text = (f"requesting {args.esp_ip}:{args.command_port}"
                   if control_target else "listen-only")
    print(f"Listening on UDP {args.bind}:{args.port}; {target_text}")
    print("Press q/Escape to quit, or s to save the current frame.")

    try:
        while True:
            now = time.monotonic()
            if control_target and now >= next_keepalive:
                control.sendto(enable_packet, control_target)
                next_keepalive = now + KEEPALIVE_INTERVAL_S

            try:
                data, source = rx.recvfrom(2048)
            except socket.timeout:
                data = None
                source = None

            if data is not None:
                chunk = parse_camera_chunk(data)
                if (chunk is not None
                        and (args.drone_id is None
                             or chunk.drone_id == args.drone_id)):
                    complete = assembler.add(chunk, source, now)
                    if complete is not None:
                        encoded = np.frombuffer(complete.jpeg, dtype=np.uint8)
                        gray = cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE)
                        if gray is None:
                            assembler.dropped_frames += 1
                        else:
                            completed_frames += 1
                            if last_frame_time is not None:
                                instant_fps = 1.0 / max(now - last_frame_time, 1e-6)
                                smoothed_fps = (instant_fps if smoothed_fps == 0.0
                                                else 0.8 * smoothed_fps
                                                + 0.2 * instant_fps)
                            last_frame_time = now
                            last_image = gray

                            view = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                            status = (f"drone {complete.drone_id}  "
                                      f"frame {complete.frame_id}  "
                                      f"{smoothed_fps:.1f} fps  "
                                      f"{len(complete.jpeg) / 1024:.1f} KiB  "
                                      f"drops {assembler.dropped_frames}")
                            cv2.putText(view, status, (6, 18),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                                        (0, 255, 0), 1, cv2.LINE_AA)
                            if args.scale != 1.0:
                                view = cv2.resize(
                                    view, None, fx=args.scale, fy=args.scale,
                                    interpolation=cv2.INTER_NEAREST)
                            cv2.imshow(window, view)

            assembler.expire(now)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s") and last_image is not None:
                args.save_dir.mkdir(parents=True, exist_ok=True)
                stamp = time.strftime("%Y%m%d_%H%M%S")
                path = args.save_dir / f"esp32_camera_{stamp}.jpg"
                if cv2.imwrite(str(path), last_image):
                    print(f"Saved {path}")
                else:
                    print(f"Could not save {path}", file=sys.stderr)
    except KeyboardInterrupt:
        pass
    finally:
        if control_target:
            try:
                control.sendto(disable_packet, control_target)
            except OSError:
                pass
        rx.close()
        control.close()
        cv2.destroyAllWindows()

    print(f"Received {completed_frames} complete frames; "
          f"dropped {assembler.dropped_frames} incomplete frames.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
