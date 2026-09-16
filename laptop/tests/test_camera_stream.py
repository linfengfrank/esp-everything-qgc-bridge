import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from camera_stream import FrameAssembler
from protocol import (
    CAMERA_FLAG_END,
    CAMERA_MAGIC,
    CAMERA_VERSION,
    CMD_CAMERA_STREAM,
    PKT_CMD,
    build_camera_stream_command,
    parse_camera_chunk,
)


HEADER = "<4sBBBBIIIHHH"


def make_chunk(frame_id: int, offset: int, payload: bytes = b"",
               frame_size: int = 0, flags: int = 0):
    raw = struct.pack(
        HEADER,
        CAMERA_MAGIC, CAMERA_VERSION, flags, 3, 0,
        frame_id, offset, frame_size, 320, 240, len(payload),
    ) + payload
    chunk = parse_camera_chunk(raw)
    assert chunk is not None
    return chunk


def test_camera_stream_command() -> None:
    assert build_camera_stream_command(True) == bytes(
        (PKT_CMD, CMD_CAMERA_STREAM, 1))
    assert build_camera_stream_command(False) == bytes(
        (PKT_CMD, CMD_CAMERA_STREAM, 0))


def test_parse_camera_data_and_end_chunks() -> None:
    data = struct.pack(
        HEADER,
        CAMERA_MAGIC, CAMERA_VERSION, 1, 7, 0,
        42, 0, 0, 320, 240, 4,
    ) + b"jpeg"
    chunk = parse_camera_chunk(data)
    assert chunk is not None
    assert chunk.drone_id == 7
    assert chunk.frame_id == 42
    assert chunk.payload == b"jpeg"
    assert chunk.is_end is False

    end = struct.pack(
        HEADER,
        CAMERA_MAGIC, CAMERA_VERSION, CAMERA_FLAG_END, 7, 0,
        42, 4, 4, 320, 240, 0,
    )
    end_chunk = parse_camera_chunk(end)
    assert end_chunk is not None
    assert end_chunk.is_end is True
    assert end_chunk.frame_size == 4


def test_parse_camera_chunk_rejects_bad_payload_length() -> None:
    raw = struct.pack(
        HEADER,
        CAMERA_MAGIC, CAMERA_VERSION, 0, 1, 0,
        1, 0, 0, 320, 240, 5,
    ) + b"four"
    assert parse_camera_chunk(raw) is None


def test_assembler_accepts_out_of_order_chunks() -> None:
    assembler = FrameAssembler()
    source = ("192.168.1.20", 5009)

    assert assembler.add(make_chunk(9, 3, b"def"), source, 1.0) is None
    assert assembler.add(
        make_chunk(9, 6, frame_size=6, flags=CAMERA_FLAG_END),
        source, 1.1,
    ) is None
    complete = assembler.add(make_chunk(9, 0, b"abc"), source, 1.2)

    assert complete is not None
    assert complete.jpeg == b"abcdef"
    assert complete.drone_id == 3


def test_assembler_expires_incomplete_frame() -> None:
    assembler = FrameAssembler(timeout_s=0.5)
    source = ("192.168.1.20", 5009)
    assembler.add(make_chunk(4, 0, b"partial"), source, 1.0)
    assembler.expire(1.6)

    assert not assembler.pending
    assert assembler.dropped_frames == 1
