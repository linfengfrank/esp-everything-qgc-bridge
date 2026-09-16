"""
Packet definitions for ESP32 ↔ laptop UDP protocol.

Telemetry (ESP32 → laptop, 10 Hz):
    Fixed-size packet.

Command (laptop → ESP32, event-driven):
    Fixed 22-byte mission packet, plus variable-size control packets.
"""

import math
import struct
from dataclasses import dataclass, field
from typing import Optional

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

PKT_TELEM     = 0x01   # telemetry from drone
PKT_CMD       = 0x02   # command to drone
PKT_AT_DEBUG  = 0x04   # live AprilTag detections (debug stream)

AT_DEBUG_PORT = 5008   # UDP port the AprilTag debug stream arrives on
CAMERA_STREAM_PORT = 5009

CMD_GOTO          = 0x01   # navigate to (goal_x, goal_y)
CMD_LAND          = 0x02   # land immediately
CMD_HOLD          = 0x03   # hold position, cancel goal
CMD_SET_NAV_TAGS  = 0x04   # send navigation tag map positions to drone
CMD_START         = 0x05   # arm and take off
CMD_SET_PEERS     = 0x06   # update nearby drone positions for inter-drone avoidance
CMD_CAMERA_STREAM = 0x07   # on-demand camera preview keepalive

VFH_BINS    = 32

# Nav state values (must match nav_state_t in nav_task.h)
NAV_IDLE       = 0
NAV_ROTATING   = 1
NAV_FLYING     = 2
NAV_ARRIVED    = 3
NAV_STUCK      = 4
NAV_RETREATING = 5

NAV_STATE_NAMES = {
    NAV_IDLE:       "IDLE",
    NAV_ROTATING:   "ROTATING",
    NAV_FLYING:     "FLYING",
    NAV_ARRIVED:    "ARRIVED",
    NAV_STUCK:      "STUCK",
    NAV_RETREATING: "RETREATING",
}

# ---------------------------------------------------------------------------
# Telemetry packet
# ---------------------------------------------------------------------------

# Wire format: pkt_type, drone_id, ned_x, ned_y, heading_rad,
#              nav_state, tag_id, tag_dist_m, vfh_blocked[32], is_stuck, reloc_age_s
_TELEM_HDR_FMT  = "<BBfffBbf32sBH"
_TELEM_HDR_SIZE = struct.calcsize(_TELEM_HDR_FMT)   # 55 bytes


@dataclass
class TelemetryPacket:
    drone_id:    int
    ned_x:       float
    ned_y:       float
    heading_rad: float
    nav_state:   int
    tag_id:      int              # −1 if no tag visible
    tag_dist_m:  float
    vfh_blocked: list            # VFH_BINS bools
    is_stuck:    bool
    reloc_age_s: int              # seconds since last nav-tag fix (0xFFFF = never)

    @property
    def nav_state_name(self) -> str:
        return NAV_STATE_NAMES.get(self.nav_state, f"UNKNOWN({self.nav_state})")


def parse_telemetry(data: bytes) -> Optional[TelemetryPacket]:
    """Parse a raw UDP payload into a TelemetryPacket. Returns None on error."""
    if len(data) < _TELEM_HDR_SIZE:
        return None

    fields = struct.unpack_from(_TELEM_HDR_FMT, data, 0)
    (pkt_type, drone_id, ned_x, ned_y, heading_rad,
     nav_state, tag_id, tag_dist_m,
     vfh_raw, is_stuck, reloc_age_s) = fields

    if pkt_type != PKT_TELEM:
        return None

    vfh_blocked = [bool(b) for b in vfh_raw]

    return TelemetryPacket(
        drone_id    = drone_id,
        ned_x       = ned_x,
        ned_y       = ned_y,
        heading_rad = heading_rad,
        nav_state   = nav_state,
        tag_id      = tag_id,
        tag_dist_m  = tag_dist_m,
        vfh_blocked = vfh_blocked,
        is_stuck    = bool(is_stuck),
        reloc_age_s = reloc_age_s,
    )

# ---------------------------------------------------------------------------
# AprilTag debug packet  (drone → laptop, UDP port AT_DEBUG_PORT, 10 Hz)
#
# Unlike the tag_id field in telemetry (a latched mission claim that never
# clears), this stream reports every raw detection in the most recent camera
# frame — tags appear and disappear in real time.
# ---------------------------------------------------------------------------

# Header: pkt_type, drone_id, frame_ms, proc_ms, latched_id, raw_count, count
_AT_DEBUG_HDR_FMT  = "<BBIHbBB"
_AT_DEBUG_HDR_SIZE = struct.calcsize(_AT_DEBUG_HDR_FMT)   # 11 bytes

# Per detection: id, hamming, margin, cx, cy, tx, ty, tz, pose_err
_AT_DET_FMT  = "<bB7f"
_AT_DET_SIZE = struct.calcsize(_AT_DET_FMT)               # 30 bytes


@dataclass
class TagDetection:
    id:       int     # tag ID (tag16h5)
    hamming:  int     # corrected bit errors
    margin:   float   # decision margin — higher = more confident
    cx:       float   # tag centre in image pixels
    cy:       float
    tx:       float   # camera-frame translation (m): X=right, Y=down, Z=forward
    ty:       float
    tz:       float
    pose_err: float   # pose reprojection error; < 0 = pose not computed

    @property
    def has_pose(self) -> bool:
        """True if this detection passed the firmware quality gate
        (hamming ≤ 1 and margin > 55) and carries a pose estimate."""
        return self.pose_err >= 0.0

    @property
    def range_m(self) -> float:
        """Straight-line camera-to-tag distance (m)."""
        return math.sqrt(self.tx**2 + self.ty**2 + self.tz**2)


@dataclass
class AtDebugPacket:
    drone_id:   int
    frame_ms:   int    # esp_timer ms when the frame was processed (0 = none yet)
    proc_ms:    int    # frame processing time (detect + pose) in ms
    latched_id: int    # mission tag claim (telemetry tag_id), −1 = none
    raw_count:  int    # detections in frame before quality filtering
    detections: list   # of TagDetection (may be truncated to 8 by firmware)


def parse_at_debug(data: bytes) -> Optional[AtDebugPacket]:
    """Parse a raw UDP payload into an AtDebugPacket. Returns None on error."""
    if len(data) < _AT_DEBUG_HDR_SIZE:
        return None

    (pkt_type, drone_id, frame_ms, proc_ms,
     latched_id, raw_count, count) = struct.unpack_from(_AT_DEBUG_HDR_FMT, data, 0)

    if pkt_type != PKT_AT_DEBUG:
        return None

    detections = []
    offset = _AT_DEBUG_HDR_SIZE
    for _ in range(count):
        if offset + _AT_DET_SIZE > len(data):
            break
        detections.append(TagDetection(*struct.unpack_from(_AT_DET_FMT, data, offset)))
        offset += _AT_DET_SIZE

    return AtDebugPacket(
        drone_id   = drone_id,
        frame_ms   = frame_ms,
        proc_ms    = proc_ms,
        latched_id = latched_id,
        raw_count  = raw_count,
        detections = detections,
    )

# ---------------------------------------------------------------------------
# Command packet
# ---------------------------------------------------------------------------

# pkt_type, cmd_type, goal_x, goal_y, found_tag_ids[12] (int8_t, −1 = unused)
_CMD_FMT  = "<BBff12b"
_CMD_SIZE = struct.calcsize(_CMD_FMT)   # 22 bytes

MAX_FOUND_TAGS = 12


@dataclass
class CommandPacket:
    cmd_type:      int
    goal_x:        float     = 0.0
    goal_y:        float     = 0.0
    found_tag_ids: list      = field(default_factory=list)  # up to 8 tag IDs


def build_command(cmd: CommandPacket) -> bytes:
    """Serialise a CommandPacket to bytes ready to send over UDP."""
    tag_ids = (cmd.found_tag_ids + [-1] * MAX_FOUND_TAGS)[:MAX_FOUND_TAGS]
    return struct.pack(_CMD_FMT, PKT_CMD, cmd.cmd_type,
                       cmd.goal_x, cmd.goal_y, *tag_ids)


def build_camera_stream_command(enabled: bool = True) -> bytes:
    """Build the 3-byte camera-preview enable/keepalive command."""
    return struct.pack("<BBB", PKT_CMD, CMD_CAMERA_STREAM, int(enabled))


# ---------------------------------------------------------------------------
# Camera JPEG chunk (drone → laptop, UDP port CAMERA_STREAM_PORT)
# ---------------------------------------------------------------------------

CAMERA_MAGIC       = b"ECAM"
CAMERA_VERSION     = 1
CAMERA_FLAG_START  = 0x01
CAMERA_FLAG_END    = 0x02

# magic, version, flags, drone_id, reserved, frame_id, offset, frame_size,
# width, height, payload_len.  Integers are little-endian like the rest of the
# project's wire protocol.
_CAMERA_HDR_FMT  = "<4sBBBBIIIHHH"
_CAMERA_HDR_SIZE = struct.calcsize(_CAMERA_HDR_FMT)  # 26 bytes


@dataclass
class CameraChunk:
    flags:        int
    drone_id:     int
    frame_id:     int
    offset:       int
    frame_size:   int
    width:        int
    height:       int
    payload:      bytes

    @property
    def is_end(self) -> bool:
        return bool(self.flags & CAMERA_FLAG_END)


def parse_camera_chunk(data: bytes) -> Optional[CameraChunk]:
    """Parse one camera UDP datagram. Returns None for malformed data."""
    if len(data) < _CAMERA_HDR_SIZE:
        return None

    (magic, version, flags, drone_id, _reserved, frame_id, offset,
     frame_size, width, height, payload_len) = struct.unpack_from(
         _CAMERA_HDR_FMT, data, 0)

    if magic != CAMERA_MAGIC or version != CAMERA_VERSION:
        return None
    if payload_len != len(data) - _CAMERA_HDR_SIZE:
        return None
    if flags & CAMERA_FLAG_END:
        if payload_len != 0 or frame_size == 0 or offset != frame_size:
            return None
    elif frame_size != 0:
        return None

    return CameraChunk(
        flags=flags,
        drone_id=drone_id,
        frame_id=frame_id,
        offset=offset,
        frame_size=frame_size,
        width=width,
        height=height,
        payload=data[_CAMERA_HDR_SIZE:],
    )


# ---------------------------------------------------------------------------
# Navigation-tag position packet  (laptop → drone)
# ---------------------------------------------------------------------------

MAX_NAV_TAGS = 16

# Wire format per tag entry: int8_t id + float map_x + float map_y
_NAV_TAG_ENTRY_FMT = "<bff"
_NAV_TAG_ENTRY_SIZE = struct.calcsize(_NAV_TAG_ENTRY_FMT)  # 9 bytes


@dataclass
class NavTag:
    """A navigation AprilTag at a known map-frame position."""
    id:    int      # AprilTag ID
    map_x: float    # NED north in map frame (m)
    map_y: float    # NED east  in map frame (m)


def build_nav_tags_command(tags: list[NavTag],
                           start_x: float = 0.0,
                           start_y: float = 0.0) -> bytes:
    """
    Build a CMD_SET_NAV_TAGS packet for one specific drone.

    tags:    list of NavTag in map frame (up to MAX_NAV_TAGS).
    start_x: drone's start position in map frame (NED north, m).
    start_y: drone's start position in map frame (NED east,  m).

    Tag positions are pre-converted to the drone's odom frame
    (map_pos − start_offset) so the drone never needs to know the
    map frame directly.
    """
    count = min(len(tags), MAX_NAV_TAGS)
    # Header: pkt_type, cmd_type, count, start_map_x, start_map_y
    buf = struct.pack("<BBBff", PKT_CMD, CMD_SET_NAV_TAGS, count, start_x, start_y)
    for t in tags[:count]:
        odom_x = t.map_x - start_x
        odom_y = t.map_y - start_y
        buf += struct.pack(_NAV_TAG_ENTRY_FMT, t.id, odom_x, odom_y)
    return buf


# ---------------------------------------------------------------------------
# Peer drone positions packet  (laptop → drone)
# ---------------------------------------------------------------------------

MAX_PEERS = 7


def build_peers_command(peers: list[tuple[float, float]]) -> bytes:
    """
    Build a CMD_SET_PEERS packet.

    peers: list of (map_x, map_y) positions in map frame, up to MAX_PEERS.
    Returns bytes ready to send over UDP.
    """
    count = min(len(peers), MAX_PEERS)
    buf = struct.pack("<BBB", PKT_CMD, CMD_SET_PEERS, count)
    for mx, my in peers[:count]:
        buf += struct.pack("<ff", mx, my)
    return buf
