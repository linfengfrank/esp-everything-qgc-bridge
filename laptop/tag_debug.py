#!/usr/bin/env python3
"""
tag_debug.py — Live AprilTag monitor.

The ESP32 streams WIFI_PKT_AT_DEBUG (0x04) packets to UDP port 5008 at 10 Hz.
Each packet carries every raw detection from the most recent camera frame, so
this view is real time: a tag appears the frame it enters the camera's view
and vanishes the frame it leaves.  No flying required — just power the drone.

    python3 tag_debug.py [--drone-id 7] [--good-only] [--log]

Default is a full-screen live display (auto-falls back to --log when output
is not a terminal):

  * no tag in view      → a dim "no tag in view" placeholder
  * one or more tags    → one row per tag: ID, body-frame offset (fwd/right),
                          distance, decision margin, hamming, pose error
  * detections that fail the firmware quality gate (hamming > 1 or
    margin <= 55) are shown dimmed with the reason — hide with --good-only
  * drone powered off   → the panel turns red "LINK DOWN" instead of
                          silently showing stale data

Note on the OLD confusing behaviour: the tag_id field in normal telemetry is
a LATCHED mission claim (at_detect_my_tag_id() in firmware).  It is set once,
when the drone claims its landing tag, and never clears in flight — that is
why it used to "stick" at the first tag seen.  That field is still shown here
as "latched", clearly separated from the live per-frame rows.

Tag IDs listed under nav_tags in setup.yaml are labelled [nav] (the config is
auto-loaded from this script's directory; override with --config).

This script binds UDP :5008 (the debug stream) and, when free, also :5005 to
show position/nav state from telemetry.  If a CommsNode script
(run_mission.py etc.) holds :5005 the live tag view still works.

Requires the firmware from the same commit (adds the 5008 debug stream) —
reflash the ESP32 if the panel says "waiting for AprilTag debug stream".

Latency: the view lags reality by roughly the per-frame processing time shown
in the header as "(NNN ms/frame)" plus <0.2 s of transport.  If it feels
sluggish, raise quad_decimate in at_detect.c — bigger is faster, at the cost
of losing far/small tags (the ms/frame figure shows the effect live).
"""

import argparse
import math
import socket
import sys
import time
from collections import deque
from pathlib import Path

from protocol import AT_DEBUG_PORT, parse_at_debug, parse_telemetry

TELEM_PORT     = 5005
LINK_TIMEOUT_S = 1.5    # no debug packet for this long → link down
FRAME_STALL_S  = 2.5    # packets arrive but frame_ms frozen → camera stalled

# Camera mount: pitched 45° nose-down, 2 cm ahead of centre
CAM_PITCH_DEG    = 45.0
CAM_FWD_OFFSET_M = 0.02

# Firmware keeps a tag pose (telemetry tag_dist_m) only when reprojection
# error < 0.5 (at_detect.c); no pose ever affects flight.
POSE_ERR_LIMIT = 0.5


def cam_to_body(tx: float, ty: float, tz: float) -> tuple:
    """Camera-frame translation → body-frame (fwd, right, hdist).
    The 45°-pitched camera transform, without the heading rotation."""
    pitch = math.radians(CAM_PITCH_DEG)
    fwd   = math.cos(pitch) * tz + math.sin(pitch) * ty + CAM_FWD_OFFSET_M
    right = tx
    return fwd, right, math.hypot(fwd, right)


def load_nav_tag_ids(path: Path) -> set:
    """Tag IDs listed under nav_tags in setup.yaml (empty set on any problem)."""
    try:
        import yaml
        with open(path) as f:
            cfg = yaml.safe_load(f)
        return set(int(k) for k in (cfg.get("nav_tags") or {}))
    except Exception:
        return set()


# ---------------------------------------------------------------------------
# ANSI helpers
# ---------------------------------------------------------------------------

class C:
    enabled = True
    RESET  = "\x1b[0m"
    BOLD   = "\x1b[1m"
    DIM    = "\x1b[2m"
    RED    = "\x1b[31m"
    GREEN  = "\x1b[32m"
    YELLOW = "\x1b[33m"
    CYAN   = "\x1b[36m"

    @classmethod
    def paint(cls, text: str, *codes: str) -> str:
        if not cls.enabled or not codes:
            return text
        return "".join(codes) + text + cls.RESET


# ---------------------------------------------------------------------------
# Per-drone state
# ---------------------------------------------------------------------------

class DroneView:
    def __init__(self, drone_id: int):
        self.drone_id   = drone_id
        self.at         = None    # latest AtDebugPacket
        self.at_rx_t    = 0.0     # wall time of latest debug packet
        self.at_count   = 0       # debug packets received
        self.telem      = None    # latest TelemetryPacket
        self.telem_rx_t = 0.0
        self.frame_times = deque(maxlen=12)   # wall times of NEW frame_ms
        self.last_frame_ms       = None
        self.last_frame_change_t = 0.0

    def update_at(self, pkt, now: float) -> bool:
        """Returns True when the display should refresh immediately
        (first packet from this drone, or a new detector frame)."""
        self.at        = pkt
        self.at_rx_t   = now
        self.at_count += 1
        new_frame = pkt.frame_ms != self.last_frame_ms
        if new_frame:
            self.last_frame_ms       = pkt.frame_ms
            self.last_frame_change_t = now
            if pkt.frame_ms != 0:
                self.frame_times.append(now)
        return new_frame or self.at_count == 1

    def update_telem(self, pkt, now: float) -> None:
        self.telem      = pkt
        self.telem_rx_t = now

    # ---- derived state -----------------------------------------------------

    def link_up(self, now: float) -> bool:
        return self.at is not None and (now - self.at_rx_t) < LINK_TIMEOUT_S

    def camera_stalled(self, now: float) -> bool:
        """Packets flowing but the detector has not produced a new frame."""
        return (self.link_up(now) and self.at.frame_ms != 0
                and (now - self.last_frame_change_t) > FRAME_STALL_S)

    def detector_fps(self, now: float):
        ft = self.frame_times
        if len(ft) < 2 or (now - ft[-1]) > 3.0:
            return None
        span = ft[-1] - ft[0]
        return (len(ft) - 1) / span if span > 0 else None

    def good_dets(self) -> list:
        if self.at is None:
            return []
        return [d for d in self.at.detections if d.has_pose]

    def weak_dets(self) -> list:
        if self.at is None:
            return []
        return [d for d in self.at.detections if not d.has_pose]


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------

def fmt_tag_row(det, nav_ids: set) -> str:
    """One display row for a quality-passing detection."""
    fwd, right, hdist = cam_to_body(det.tx, det.ty, det.tz)
    label = C.paint(f"TAG {det.id:>2}", C.BOLD, C.GREEN)
    nav   = C.paint(" [nav]", C.CYAN) if det.id in nav_ids else ""
    err   = f"err {det.pose_err:.3f}"
    if det.pose_err >= POSE_ERR_LIMIT:
        err = C.paint(err + " HIGH", C.YELLOW)
    return (f"    {label}{nav}  fwd {fwd:+5.2f}m  right {right:+5.2f}m  "
            f"dist {hdist:4.2f}m  range {det.range_m:4.2f}m  "
            f"margin {det.margin:5.1f}  ham {det.hamming}  {err}")


def fmt_weak_row(det) -> str:
    """One display row for a detection rejected by the quality gate."""
    reason = "hamming>1" if det.hamming > 1 else "margin<=55"
    return C.paint(f"    tag {det.id:>2}?  margin {det.margin:5.1f}  "
                   f"ham {det.hamming}  px({det.cx:3.0f},{det.cy:3.0f})  "
                   f"rejected: {reason}", C.DIM)


def fmt_latched(latched_id: int) -> str:
    if latched_id < 0:
        return "latched: none"
    return C.paint(f"latched: TAG {latched_id}", C.YELLOW)


def render(views: dict, now: float, telem_ok: bool, good_only: bool,
           nav_ids: set) -> str:
    lines = []
    clock = time.strftime("%H:%M:%S")
    telem_note = "" if telem_ok else \
        C.paint("  (:5005 busy — pos/nav hidden)", C.DIM)
    lines.append(C.paint(" AprilTag live monitor", C.BOLD)
                 + f"  stream :{AT_DEBUG_PORT}"
                 + (f" + telemetry :{TELEM_PORT}" if telem_ok else "")
                 + telem_note + f"   {clock}")
    lines.append("")

    if not views:
        lines.append(C.paint("  waiting for AprilTag debug stream on "
                             f"UDP :{AT_DEBUG_PORT} ...", C.DIM))
        lines.append(C.paint("  (no packets yet — is the drone powered and "
                             "flashed with the 5008 debug stream?)", C.DIM))

    for did in sorted(views):
        v   = views[did]
        up  = v.link_up(now)
        age = now - v.at_rx_t

        # -- drone header ----------------------------------------------------
        fps = v.detector_fps(now)
        fps_s = f"detector {fps:4.1f} fps" if fps else "detector   -- fps"
        if v.at is not None and v.at.frame_ms != 0:
            proc_s = f"{v.at.proc_ms:4d} ms"
            if v.at.proc_ms > 400:
                proc_s = C.paint(proc_s, C.YELLOW)
            fps_s += f" ({proc_s}/frame)"
        link_s = (C.paint("link OK", C.GREEN) + f" ({age:4.1f}s)") if up else \
                 C.paint(f"LINK DOWN {age:5.1f}s", C.BOLD, C.RED)
        latched = v.at.latched_id if v.at else -1
        lines.append(f" {C.paint(f'DRONE {did}', C.BOLD)}   {link_s}   "
                     f"{fps_s}   {fmt_latched(latched)}   "
                     f"pkts {v.at_count}")

        # -- telemetry line --------------------------------------------------
        if v.telem is not None and (now - v.telem_rx_t) < LINK_TIMEOUT_S:
            t = v.telem
            reloc = "never" if t.reloc_age_s == 0xFFFF else f"{t.reloc_age_s}s"
            lines.append(f"    pos ({t.ned_x:+6.2f}, {t.ned_y:+6.2f}) m   "
                         f"hdg {math.degrees(t.heading_rad):+4.0f}°   "
                         f"nav {t.nav_state_name:<10}  reloc {reloc}")

        # -- tag rows --------------------------------------------------------
        if not up:
            lines.append(C.paint("    no packets — drone off or WiFi lost "
                                 "(stale data hidden)", C.RED))
        elif v.at.frame_ms == 0:
            lines.append(C.paint("    waiting for first camera frame ...",
                                 C.YELLOW))
        elif v.camera_stalled(now):
            stall = now - v.last_frame_change_t
            lines.append(C.paint(f"    camera stalled — no new frame for "
                                 f"{stall:.1f}s", C.BOLD, C.YELLOW))
        else:
            good = v.good_dets()
            weak = v.weak_dets()
            n_shown = len(good) + (0 if good_only else len(weak))
            if n_shown == 0:
                lines.append(C.paint("    — no tag in view —", C.DIM))
            else:
                head = f"    {len(good)} tag{'s' if len(good) != 1 else ''} in view"
                extra = []
                if weak and not good_only:
                    extra.append(f"{len(weak)} rejected")
                if v.at.raw_count > len(v.at.detections):
                    extra.append(f"+{v.at.raw_count - len(v.at.detections)} truncated")
                if extra:
                    head += C.paint(f"  ({', '.join(extra)})", C.DIM)
                lines.append(head)
                for d in good:
                    lines.append(fmt_tag_row(d, nav_ids))
                if not good_only:
                    for d in weak:
                        lines.append(fmt_weak_row(d))
        lines.append("")

    lines.append(C.paint(" latched = mission claim from telemetry tag_id; set "
                         "once, only cleared on landing abort.", C.DIM))
    lines.append(C.paint(" Rows above are per-frame live truth.  Ctrl-C to "
                         "quit.", C.DIM))
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# --log mode: print only state changes (nothing when nothing changes)
# ---------------------------------------------------------------------------

class ChangeLogger:
    def __init__(self, nav_ids: set):
        self.nav_ids = nav_ids
        self.state   = {}   # drone_id -> dict(link, tags, latched)

    @staticmethod
    def _stamp(t: float) -> str:
        return f"[{t:8.1f}s]"

    def check(self, views: dict, now: float, t0: float) -> None:
        t = now - t0
        for did in sorted(views):
            v  = views[did]
            st = self.state.setdefault(
                did, {"link": None, "tags": None, "latched": None})

            link = v.link_up(now)
            if link != st["link"]:
                if link:
                    print(f"{self._stamp(t)} drone {did}  LINK UP")
                elif st["link"] is not None:
                    print(f"{self._stamp(t)} drone {did}  LINK DOWN "
                          "(no packets — power off or WiFi lost)")
                st["link"] = link
                if not link:
                    st["tags"] = None     # re-announce tags on link return
                    continue

            if not link or v.at is None or v.at.frame_ms == 0:
                continue

            latched = v.at.latched_id
            if latched != st["latched"]:
                print(f"{self._stamp(t)} drone {did}  latched mission tag: "
                      f"{'none' if latched < 0 else latched}")
                st["latched"] = latched

            good = v.good_dets()
            sig  = tuple(sorted(d.id for d in good))
            if sig != st["tags"]:
                st["tags"] = sig
                if not good:
                    print(f"{self._stamp(t)} drone {did}  no tag in view")
                else:
                    for d in good:
                        fwd, right, hdist = cam_to_body(d.tx, d.ty, d.tz)
                        nav = " [nav]" if d.id in self.nav_ids else ""
                        print(f"{self._stamp(t)} drone {did}  TAG {d.id}{nav}  "
                              f"fwd {fwd:+.2f}m right {right:+.2f}m "
                              f"dist {hdist:.2f}m  margin {d.margin:.1f} "
                              f"ham {d.hamming} err {d.pose_err:.3f}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def open_sockets(at_port: int, telem_port: int):
    at_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    at_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    at_sock.bind(("", at_port))
    at_sock.setblocking(False)

    # Telemetry is optional: no SO_REUSEADDR so we never steal packets from a
    # running CommsNode script — if the port is busy we simply do without.
    telem_sock = None
    try:
        telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        telem_sock.bind(("", telem_port))
        telem_sock.setblocking(False)
    except OSError:
        if telem_sock is not None:
            telem_sock.close()
        telem_sock = None
    return at_sock, telem_sock


def drain(sock, handler) -> None:
    while True:
        try:
            data, _ = sock.recvfrom(2048)
        except (BlockingIOError, InterruptedError):
            return
        handler(data)


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Live AprilTag monitor (real-time detections from the "
                    "ESP32 debug stream on UDP :5008)")
    ap.add_argument("--port", type=int, default=AT_DEBUG_PORT,
                    help=f"AprilTag debug UDP port (default: {AT_DEBUG_PORT})")
    ap.add_argument("--telem-port", type=int, default=TELEM_PORT,
                    help=f"Telemetry UDP port, optional (default: {TELEM_PORT})")
    ap.add_argument("--drone-id", type=int, default=None,
                    help="Only show this drone (default: all)")
    ap.add_argument("--good-only", action="store_true",
                    help="Hide detections rejected by the quality gate")
    ap.add_argument("--config", type=Path,
                    default=Path(__file__).resolve().parent / "setup.yaml",
                    help="setup.yaml used to label [nav] tags")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--log", action="store_true",
                      help="Line-per-change log instead of the live screen")
    mode.add_argument("--ui", action="store_true",
                      help="Force the live screen even when not a TTY")
    ap.add_argument("--no-color", action="store_true", help="Disable colours")
    args = ap.parse_args()

    use_ui = args.ui or (not args.log and sys.stdout.isatty())
    C.enabled = use_ui and not args.no_color

    try:
        at_sock, telem_sock = open_sockets(args.port, args.telem_port)
    except OSError as exc:
        print(f"Cannot bind UDP :{args.port} — {exc}")
        print("Is another tag_debug.py already running?")
        return 1

    nav_ids = load_nav_tag_ids(args.config)
    views: dict[int, DroneView] = {}
    t0 = time.monotonic()

    dirty = False   # a new detector frame arrived → render without waiting

    def on_at(data: bytes) -> None:
        nonlocal dirty
        pkt = parse_at_debug(data)
        if pkt is None or args.drone_id not in (None, pkt.drone_id):
            return
        view = views.setdefault(pkt.drone_id, DroneView(pkt.drone_id))
        if view.update_at(pkt, time.monotonic()):
            dirty = True

    def on_telem(data: bytes) -> None:
        pkt = parse_telemetry(data)
        if pkt is None or args.drone_id not in (None, pkt.drone_id):
            return
        views.setdefault(pkt.drone_id,
                         DroneView(pkt.drone_id)).update_telem(pkt, time.monotonic())

    socks = [at_sock] + ([telem_sock] if telem_sock else [])

    if not use_ui:
        print(f"Listening for AprilTag debug on UDP :{args.port}"
              + (f" (+ telemetry :{args.telem_port})" if telem_sock else
                 f" (telemetry :{args.telem_port} busy — skipped)"))
        print("Logging state changes only; silence means nothing changed. "
              "Ctrl-C to stop.")
        logger = ChangeLogger(nav_ids)

    import select
    last_render = 0.0
    try:
        if use_ui:
            sys.stdout.write("\x1b[2J\x1b[?25l")   # clear screen, hide cursor
        while True:
            ready, _, _ = select.select(socks, [], [], 0.05)
            for s in ready:
                drain(s, on_at if s is at_sock else on_telem)

            now = time.monotonic()
            if use_ui:
                # Render immediately on a fresh detector frame; otherwise tick
                # at 10 Hz to keep ages/clock moving.
                if dirty or now - last_render >= 0.1:
                    dirty = False
                    last_render = now
                    frame = render(views, now, telem_sock is not None,
                                   args.good_only, nav_ids)
                    out = "\x1b[H"
                    for ln in frame.split("\n"):
                        out += ln + "\x1b[K\n"
                    out += "\x1b[J"
                    sys.stdout.write(out)
                    sys.stdout.flush()
            else:
                logger.check(views, now, t0)
    except KeyboardInterrupt:
        pass
    finally:
        if use_ui:
            sys.stdout.write("\x1b[?25h\x1b[0m\n")   # restore cursor
            sys.stdout.flush()

    print("Stopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
