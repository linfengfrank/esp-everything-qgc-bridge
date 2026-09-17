import select
import socket
import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from camera_stream import FRAME_TIMEOUT_S, FrameAssembler, drain_socket
from protocol import (
    build_camera_stream_command,
    camera_packet_version,
    parse_camera_chunk,
)

# Pinned independently of protocol.py: must match main/camera_stream.c.
HEADER = "<4sBBBBIIIHHHHH"
CHUNK = 1400


def datagrams(frame_id, jpeg, nonce=7, version=2, age_ms=120, esp_drops=0):
    """The datagrams the firmware sends for one JPEG."""
    def pkt(flags, offset, size, payload):
        return struct.pack(HEADER, b"ECAM", version, flags, 22, nonce, frame_id,
                           offset, size, 320, 240, len(payload), age_ms,
                           esp_drops) + payload
    out = [pkt(0, off, 0, jpeg[off:off + CHUNK])
           for off in range(0, len(jpeg), CHUNK)]
    return out + [pkt(0x02, len(jpeg), len(jpeg), b"")]


def feed(asm, packets, now=0.0):
    shown = []
    for data in packets:
        frame = asm.add(parse_camera_chunk(data), now)
        if frame is not None:
            shown.append(frame.frame_id)
    return shown


JPEG = bytes(range(256)) * 12            # 3072 bytes -> 3 data datagrams


def test_keepalive_bytes_match_firmware():
    # main/wifi_task.h: WIFI_PKT_CMD 0x02, CMD_CAMERA_STREAM 0x07
    assert build_camera_stream_command() == bytes((0x02, 0x07, 1, 0, 0))
    assert build_camera_stream_command(True, 12, 75) == bytes.fromhex("0207010c4b")
    assert build_camera_stream_command(False) == bytes((0x02, 0x07, 0, 0, 0))
    for fps, quality in ((16, 0), (-1, 0), (0, 9), (0, 91)):
        with pytest.raises(ValueError):
            build_camera_stream_command(True, fps, quality)


def test_parse_header():
    data, *_, end = datagrams(5, JPEG, nonce=9, age_ms=127, esp_drops=3)
    assert struct.calcsize(HEADER) == 30
    chunk = parse_camera_chunk(data)
    assert (chunk.boot_nonce, chunk.frame_id, chunk.offset, chunk.age_ms,
            chunk.esp_drops, len(chunk.payload)) == (9, 5, 0, 127, 3, CHUNK)
    assert parse_camera_chunk(end).is_end
    assert parse_camera_chunk(data[:-1]) is None          # bad payload_len
    assert parse_camera_chunk(end[:29]) is None
    old = datagrams(5, JPEG, version=1)[0]
    assert parse_camera_chunk(old) is None
    assert camera_packet_version(old) == 1
    assert camera_packet_version(b"hello") is None


def test_out_of_order_assembly():
    asm = FrameAssembler()
    packets = datagrams(1, JPEG)
    frame = None
    for data in reversed(packets):
        frame = asm.add(parse_camera_chunk(data), 0.0) or frame
    assert frame.jpeg == JPEG and frame.age_ms == 120
    assert asm.dropped == 0


def test_lost_datagram_drops_frame_and_is_counted():
    asm = FrameAssembler()
    assert feed(asm, datagrams(1, JPEG)) == [1]
    broken = datagrams(2, JPEG)
    del broken[1]
    assert feed(asm, broken) == []
    assert feed(asm, datagrams(4, JPEG)) == [4]        # 2 lost, 3 never sent
    assert asm.dropped == 2
    assert asm.pending == {}


def test_late_and_duplicate_datagrams_are_ignored():
    asm = FrameAssembler()
    feed(asm, datagrams(1, JPEG) + datagrams(2, JPEG))
    assert feed(asm, datagrams(1, JPEG) + datagrams(2, JPEG)) == []
    assert feed(asm, datagrams(3, JPEG)) == [3]
    assert asm.dropped == 0


def test_incomplete_frame_expires():
    asm = FrameAssembler()
    feed(asm, datagrams(1, JPEG)[:-1], now=0.0)
    asm.expire(FRAME_TIMEOUT_S + 0.01)
    assert asm.pending == {}


def test_reboot_restarts_sequence():
    asm = FrameAssembler()
    feed(asm, datagrams(500, JPEG, nonce=7))
    assert feed(asm, datagrams(1, JPEG, nonce=8)) == [1]     # new nonce
    feed(asm, datagrams(400, JPEG, nonce=8))
    assert feed(asm, datagrams(1, JPEG, nonce=8)) == [1]     # same nonce, id jump back
    assert asm.dropped == 398


def test_drain_returns_newest_and_filters_source():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        for data in datagrams(1, JPEG) + datagrams(2, JPEG) + [b"ECAM\x01junk"]:
            tx.sendto(data, rx.getsockname())
        asm, returned, count, bad_version = FrameAssembler(), [], 0, None
        while count < 9 and select.select([rx], [], [], 1.0)[0]:
            frame, n, version = drain_socket(rx, asm, "127.0.0.1")
            returned += [frame.frame_id] if frame else []
            count += n
            bad_version = version or bad_version
        assert returned[-1] == 2
        assert len(returned) + asm.dropped == 2   # frame 1 shown or counted
        assert bad_version == 1

        for data in datagrams(3, JPEG):
            tx.sendto(data, rx.getsockname())
        other, count = FrameAssembler(), 0
        while count < 4 and select.select([rx], [], [], 1.0)[0]:
            frame, n, _ = drain_socket(rx, other, "10.0.0.1")
            assert frame is None
            count += n
        assert count == 4 and other.pending == {}
    finally:
        rx.close()
        tx.close()
