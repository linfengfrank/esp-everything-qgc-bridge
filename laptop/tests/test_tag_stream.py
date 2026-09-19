"""tag_stream.py logic: ESP overlay state, labelling, drawing, CLI.

No sockets and no window — everything here is the pure part of the viewer.
AT-debug packets are packed from struct below, pinned to main/wifi_task.h the
same way test_camera_stream.py pins the camera header.
"""

import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

import tag_stream
from apriltag_host import (
    GATE_MAX_HAMMING,
    GATE_MIN_MARGIN,
    PIXEL_CENTRE_OFFSET,
    Detection,
)
from protocol import parse_at_debug
from tag_debug import FRAME_STALL_S, LINK_TIMEOUT_S
from tag_stream import (
    EspMarker,
    EspOverlay,
    annotate,
    det_label,
    det_record,
    detector_params,
    esp_label,
    esp_record,
    hud_lines,
    parse_args,
    to_view,
    view_geometry,
)

# Pinned independently of protocol.py: must match wifi_at_debug_pkt_t and
# wifi_at_det_t in main/wifi_task.h.
AT_HDR = "<BBIHbBB"     # pkt_type, drone_id, frame_ms, proc_ms, latched, raw, count
AT_DET = "<bB7f"        # id, hamming, margin, cx, cy, tx, ty, tz, pose_err
PKT_AT_DEBUG = 0x04


def at_packet(frame_ms, dets=(), proc_ms=620, latched=-1, drone_id=22,
              raw_count=None):
    """One WIFI_PKT_AT_DEBUG datagram; dets are (id, hamming, margin, cx, cy,
    tx, ty, tz, pose_err) tuples (pose_err < 0 = the drone's gate rejected it)."""
    body = b"".join(struct.pack(AT_DET, *d) for d in dets)
    raw = len(dets) if raw_count is None else raw_count
    return struct.pack(AT_HDR, PKT_AT_DEBUG, drone_id, frame_ms, proc_ms,
                       latched, raw, len(dets)) + body


GOOD_DET = (5, 0, 205.0, 148.0, 191.0, -0.03, 0.57, 1.05, 3e-6)
WEAK_DET = (9, 2, 3.0, 40.0, 60.0, 0.0, 0.0, 0.0, -1.0)


def make_det(**kw) -> Detection:
    base = dict(id=5, hamming=0, decision_margin=205.0, centre=(120.0, 120.0),
                corners=((100.0, 100.0), (140.0, 100.0), (140.0, 140.0),
                         (100.0, 140.0)),
                pose_valid=True, t=(-0.03, 0.57, 1.05), R=tuple(range(9)),
                pose_err=3e-6)
    return Detection(**{**base, **kw})


def parse(pkt):
    parsed = parse_at_debug(pkt)
    assert parsed is not None
    return parsed


# ---------------------------------------------------------------------------
# Wire format
# ---------------------------------------------------------------------------

def test_at_debug_sizes_match_firmware():
    assert struct.calcsize(AT_HDR) == 11
    assert struct.calcsize(AT_DET) == 30
    pkt = parse(at_packet(1234, [GOOD_DET, WEAK_DET], latched=5))
    assert (pkt.drone_id, pkt.frame_ms, pkt.proc_ms, pkt.latched_id) == (
        22, 1234, 620, 5)
    assert [d.id for d in pkt.detections] == [5, 9]
    assert pkt.detections[0].has_pose and not pkt.detections[1].has_pose


# ---------------------------------------------------------------------------
# ESP overlay state
# ---------------------------------------------------------------------------

def test_new_frame_is_reported_once():
    esp = EspOverlay()
    assert esp.update(parse(at_packet(1000, [GOOD_DET])), 10.0) is True
    assert esp.update(parse(at_packet(1000, [GOOD_DET])), 10.1) is False
    assert esp.update(parse(at_packet(1700, [GOOD_DET])), 10.7) is True
    assert esp.count == 3


def test_markers_carry_the_detections_and_their_age():
    esp = EspOverlay()
    esp.update(parse(at_packet(1000, [GOOD_DET, WEAK_DET])), 10.0)
    markers = esp.markers(10.3)
    assert [m.id for m in markers] == [5, 9]
    assert markers[0].gated and not markers[1].gated
    assert (markers[0].cx, markers[0].cy) == (148.0, 191.0)
    # the drone's own quality numbers, for comparing the two detectors
    assert markers[0].margin == pytest.approx(205.0)
    assert markers[0].hamming == 0
    assert markers[0].pose_err == pytest.approx(3e-6)
    assert markers[1].hamming == 2 and markers[1].pose_err < 0
    # age = time since the result landed + the time the drone spent on it
    assert markers[0].age_s == pytest.approx(0.3 + 0.620)
    assert esp.result_age_s(10.3) == pytest.approx(0.92)


def test_markers_can_hide_the_drones_own_rejects():
    esp = EspOverlay()
    esp.update(parse(at_packet(1000, [GOOD_DET, WEAK_DET])), 10.0)
    assert [m.id for m in esp.markers(10.1, show_rejected=False)] == [5]


def test_nothing_is_drawn_before_the_first_packet():
    esp = EspOverlay()
    assert esp.markers(10.0) == []
    assert not esp.link_up(10.0) and not esp.fresh(10.0)
    assert "waiting" in esp.status(10.0)


def test_a_quiet_link_hides_the_markers():
    esp = EspOverlay()
    esp.update(parse(at_packet(1000, [GOOD_DET])), 10.0)
    assert esp.markers(10.0 + LINK_TIMEOUT_S - 0.1)
    assert esp.markers(10.0 + LINK_TIMEOUT_S + 0.1) == []
    assert "LINK DOWN" in esp.status(10.0 + LINK_TIMEOUT_S + 0.1)


def test_a_frozen_frame_ms_hides_the_markers():
    """Packets keep arriving at 10 Hz but the detector has stopped."""
    esp = EspOverlay()
    now = 10.0
    esp.update(parse(at_packet(1000, [GOOD_DET])), now)
    while now < 10.0 + FRAME_STALL_S + 0.5:
        now += 0.1
        esp.update(parse(at_packet(1000, [GOOD_DET])), now)   # same frame_ms
    assert esp.link_up(now) and esp.stalled(now)
    assert esp.markers(now) == []
    assert "STALLED" in esp.status(now)


def test_no_camera_frame_yet_is_not_treated_as_a_detection():
    esp = EspOverlay()
    esp.update(parse(at_packet(0)), 10.0)
    assert not esp.fresh(10.0)
    assert esp.markers(10.0) == []
    assert "first camera frame" in esp.status(10.0)


def test_detector_fps_and_status():
    esp = EspOverlay()
    for i in range(6):
        esp.update(parse(at_packet(1000 + i * 700, [GOOD_DET]), ), 10.0 + i * 0.7)
    now = 10.0 + 5 * 0.7
    assert esp.detector_fps(now) == pytest.approx(1 / 0.7, rel=1e-6)
    status = esp.status(now)
    assert "1.4 fps" in status and "620 ms/frame" in status
    assert "latched none" in status
    esp.update(parse(at_packet(9000, [GOOD_DET], latched=5)), now + 0.7)
    assert "latched 5" in esp.status(now + 0.7)


# ---------------------------------------------------------------------------
# Labels
# ---------------------------------------------------------------------------

def test_label_shows_id_margin_hamming_and_distance():
    label = det_label(make_det(), frozenset(), detail=True)
    # |(-0.03, 0.57, 1.05)| -> 1.20 m from the camera to the tag
    assert label == "tag 5 m205 h0 1.20m"


def test_nav_tags_are_labelled():
    label = det_label(make_det(id=14), {12, 14}, detail=True)
    assert label.startswith("tag 14 [nav]")
    assert "[nav]" not in det_label(make_det(id=13), {12, 14},
                                    detail=True)


@pytest.mark.parametrize("kw,reason", [
    (dict(hamming=2), "hamming>1"),
    (dict(decision_margin=12.0), "margin<=55"),
])
def test_rejected_labels_carry_the_reason(kw, reason):
    det = make_det(pose_valid=False, t=None, R=None, pose_err=None, **kw)
    label = det_label(det, frozenset(), detail=True)
    assert label.startswith("tag 5?") and label.endswith(reason)


def test_a_pose_the_firmware_would_discard_is_marked():
    """at_detect.c only acts on a pose with err < 0.5, so the label must say
    so — otherwise the viewer shows a distance the drone would never use."""
    from apriltag_host import POSE_ERR_LIMIT

    ok = det_label(make_det(pose_err=POSE_ERR_LIMIT - 0.01), frozenset(),
                   detail=True)
    high = det_label(make_det(pose_err=POSE_ERR_LIMIT), frozenset(),
                     detail=True)
    assert ok == "tag 5 m205 h0 1.20m"
    assert high == "tag 5 m205 h0 1.20m err0.50 HIGH"


def test_the_esp_label_shows_the_drones_own_margin():
    """So the drone's margin reads straight against the laptop's."""
    esp = EspOverlay()
    esp.update(parse(at_packet(1000, [GOOD_DET, WEAK_DET])), 10.0)
    good, weak = esp.markers(10.68)
    assert esp_label(good) == "esp 5 m205 1.3s old"
    assert esp_label(weak) == "esp 9? m3 1.3s old"


def test_the_viewer_uses_the_firmware_gate():
    """The viewer never invents its own thresholds: one gate, apriltag_host's,
    which mirrors at_detect.c."""
    assert (GATE_MIN_MARGIN, GATE_MAX_HAMMING) == (55.0, 1)
    assert "?" not in det_label(make_det(decision_margin=55.01), frozenset())
    assert det_label(make_det(decision_margin=55.0), frozenset(),
                     detail=True).endswith("margin<=55")


def test_det_record_is_json_friendly():
    import json
    rec = det_record(make_det())
    assert rec["id"] == 5 and rec["rejected"] is None
    assert len(rec["corners"]) == 4
    json.dumps(rec)
    weak = det_record(make_det(hamming=2, pose_valid=False, t=None, R=None,
                               pose_err=None))
    assert weak["rejected"] == "hamming>1" and weak["t"] is None


def test_esp_record_keeps_the_drones_own_quality_numbers():
    """--save-dir is how the two detectors get compared: hamming, margin and
    pose_err have to survive into the log, not just id and position."""
    import json
    esp = EspOverlay()
    esp.update(parse(at_packet(1000, [GOOD_DET, WEAK_DET])), 10.0)
    good, weak = (esp_record(m) for m in esp.markers(10.2))
    assert good == {"id": 5, "cx": 148.0, "cy": 191.0, "age_s": 0.82,
                    "gated": True, "margin": 205.0, "hamming": 0,
                    "pose_err": 3e-06}
    assert weak["id"] == 9 and weak["gated"] is False
    assert weak["hamming"] == 2 and weak["margin"] == 3.0
    assert weak["pose_err"] == -1.0
    json.dumps([good, weak])


def test_hud_lines_report_what_the_viewer_is_doing():
    lines = hud_lines(frame_id=42, fps=9.6, esp_age_ms=130, latency_ms=12.0,
                      drops=3, esp_drops=1, detect_ms=1.84,
                      params=detector_params(parse_args(
                          ["--esp-ip", "127.0.0.1"])),
                      n_good=1, n_rejected=2,
                      esp_status="esp 1.4 fps 620 ms/frame")
    assert len(lines) == 3
    assert "#42" in lines[0] and "9.6 fps" in lines[0] and "det 1.8 ms" in lines[0]
    assert "esp 130 ms" in lines[0] and "drops 3/1" in lines[0]
    assert "dec 1.5 sig 1 sharp 0.75" in lines[1]
    assert "gate h<=1 m>55" in lines[1]
    assert "1 tag (+2 rejected)" in lines[1]
    assert lines[2] == "esp 1.4 fps 620 ms/frame"
    assert "rejected" not in hud_lines(
        frame_id=1, fps=1.0, esp_age_ms=0, latency_ms=0.0, drops=0,
        esp_drops=0, detect_ms=0.0,
        params=detector_params(parse_args(["--esp-ip", "127.0.0.1"])),
        n_good=0, n_rejected=0,
        esp_status="")[1]


def hud_for(**kw) -> list:
    base = dict(frame_id=1, fps=1.0, esp_age_ms=0, latency_ms=0.0, drops=0,
                esp_drops=0, detect_ms=0.0,
                params=detector_params(parse_args(["--esp-ip", "127.0.0.1"])),
                n_good=1, n_rejected=0,
                esp_status="")
    return hud_lines(**{**base, **kw})


def test_detections_past_the_buffer_are_reported():
    """Detector.truncated: >32 tags in view must not be silently under-counted."""
    assert "truncated" not in hud_for()[1]
    assert hud_for(truncated=3)[1].endswith("1 tag (+3 truncated)")
    assert hud_for(n_rejected=2, truncated=3)[1].endswith(
        "1 tag (+2 rejected, +3 truncated)")


# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------

def test_half_pixel_offset_is_applied_once():
    assert to_view(100.5, 60.5, 1.0) == (100, 60)
    assert to_view(100.5, 60.5, 2.0) == (200, 120)
    # Scale first, THEN shift half a VIEW pixel: subtracting 0.5 before
    # scaling moves the whole overlay 0.5 * (scale - 1) px up and left.
    assert to_view(100.0, 60.0, 4.0) == (400, 240)
    assert to_view(100.5, 60.5, 4.0) == (402, 242)
    assert PIXEL_CENTRE_OFFSET == 0.5


def test_view_geometry_follows_the_decoded_frame():
    """The overlay is drawn in the decoded frame's pixels, whatever its size."""
    assert view_geometry((240, 320), 2.0) == (640, 480, 2.0)
    assert view_geometry((240, 320, 3), 1.0) == (320, 240, 1.0)
    assert view_geometry((480, 640), 2.0) == (1280, 960, 2.0)   # not 640x480
    w, h, scale = view_geometry((96, 128), 0.5)
    assert (w, h) == (64, 48) and scale == 0.5


def blank(h=240, w=320):
    return np.zeros((h, w, 3), np.uint8)


def test_annotate_draws_the_tag_where_the_tag_is():
    view = blank()
    annotate(view, [make_det()], [], [], scale=1.0)
    assert view[95:145, 95:145].any()                 # the outline
    assert not view[150:].any()                       # and nowhere else
    assert not view[:, :95].any()                     # (the label sits above)
    assert not view[:60].any()
    # corner 0 is marked, so the tag's orientation is visible
    assert tuple(view[100, 100]) != (0, 0, 0)


def test_annotate_scales_with_the_view():
    view = blank(480, 640)
    annotate(view, [make_det()], [], [], scale=2.0)
    assert view[190:290, 190:290].any()
    assert not view[:150].any()


def test_rejected_detections_can_be_hidden():
    det = make_det(hamming=2, pose_valid=False, t=None, R=None, pose_err=None)
    shown, hidden = blank(), blank()
    annotate(shown, [det], [], [], scale=1.0, show_rejected=True)
    annotate(hidden, [det], [], [], scale=1.0, show_rejected=False)
    assert shown.any() and not hidden.any()


def test_annotate_draws_the_esp_layer_and_the_hud():
    view = blank()
    annotate(view, [], [EspMarker(5, 148.0, 191.0, 1.1, True)],
             ["hello", "world"], scale=1.0)
    assert view[180:200, 138:158].any()               # the magenta cross
    assert view[:30, :80].any()                       # the HUD text
    esp_only = blank()
    annotate(esp_only, [], [EspMarker(5, 148.0, 191.0, 1.1, True)], [],
             scale=1.0)
    assert not esp_only[:30, :80].any()               # no HUD when none given


def ink(view) -> int:
    """Pixels the drawing actually lit — a truncated label lights fewer."""
    return int(view.any(axis=2).sum())


def test_a_label_at_the_right_edge_is_not_truncated():
    """OpenCV silently cuts text at the edge, and a cut label reads as another
    tag ('esp 17? 1.2s old' as 'esp 1'); it must be slid left instead."""
    def draw(x0):
        det = make_det(id=12, hamming=2, decision_margin=2.4, pose_valid=False,
                       t=None, R=None, pose_err=None,
                       corners=((x0, 100.0), (x0 + 40, 100.0),
                                (x0 + 40, 140.0), (x0, 140.0)),
                       centre=(x0 + 20, 120.0))
        view = blank()
        annotate(view, [det], [], [], scale=1.0, nav_ids={12})
        return view

    assert ink(draw(270.0)) == pytest.approx(ink(draw(20.0)), rel=0.02)


def test_an_esp_label_at_the_right_edge_is_not_truncated():
    def draw(cx):
        view = blank()
        annotate(view, [], [EspMarker(17, cx, 120.0, 1.2, False, 3.0, 2, -1.0)],
                 [], scale=1.0)
        return view

    assert ink(draw(300.0)) == pytest.approx(ink(draw(100.0)), rel=0.02)


def test_the_esp_cross_is_hollow():
    """A solid cross would hide the payload bits the green outline is judged on."""
    view = blank()
    annotate(view, [], [EspMarker(5, 148.0, 191.0, 1.1, True, 206.0)], [],
             scale=1.0)
    x, y = to_view(148.0, 191.0, 1.0)
    assert view[y - 6:y + 7, x - 6:x + 7].max() > 200    # the ticks are drawn
    # Only antialiasing spills into the middle: the tag shows through.
    assert view[y - 1:y + 2, x - 1:x + 2].max() < 32


def test_hud_shrinks_to_fit_a_narrow_view():
    line = "#301  5.1 fps  esp 124 ms  lat 29 ms  drops 0/0  det 1.9 ms"
    narrow, wide = blank(240, 320), blank(480, 640)
    annotate(narrow, [], [], [line], scale=1.0)
    annotate(wide, [], [], [line], scale=2.0)
    assert narrow.any() and not narrow[:, -4:].any()   # nothing runs off --scale 1
    assert wide[:, 300:500].any()                      # full size in a big window


def test_the_hud_is_complete_at_scale_1():
    """The shrink stops at MIN_TEXT_SCALE, where line 2 (340 px) still does not
    fit a 320 px view, so it must wrap rather than be cut mid-word."""
    lines = hud_for(n_rejected=2,
                    esp_status="esp 1.4 fps 620 ms/frame  lag 1.1s  latched none")
    width = 320
    scale = tag_stream._hud_scale(lines, width)
    assert scale == tag_stream.MIN_TEXT_SCALE
    assert cv2.getTextSize(lines[1], tag_stream.FONT, scale,
                           1)[0][0] > width - 12

    wrapped = tag_stream._hud_wrap(lines, width, scale)
    assert len(wrapped) == len(lines) + 1
    # Wrapped, not truncated: every word survives, only the separator at the
    # break is spent on the line break.
    assert " ".join(wrapped).split() == " ".join(lines).split()
    assert wrapped[-2].endswith("rejected)")
    for line in wrapped:
        assert cv2.getTextSize(line, tag_stream.FONT, scale,
                               1)[0][0] <= width - 12

    view = blank()
    annotate(view, [], [], lines, scale=1.0)
    assert not view[:, -4:].any()                    # nothing runs off the edge


def test_annotate_survives_detections_at_the_image_edge():
    edge = make_det(corners=((0.0, 0.0), (8.0, 0.0), (8.0, 8.0), (0.0, 8.0)),
                    centre=(4.0, 4.0))
    view = blank()
    annotate(view, [edge], [EspMarker(3, 319.0, 239.0, 0.5, False)],
             ["x" * 200], scale=1.0)
    assert view.any()


def test_annotate_leaves_an_empty_frame_empty():
    view = blank()
    annotate(view, [], [], [], scale=1.0)
    assert not view.any()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def test_defaults_are_the_firmware_parameters():
    args = parse_args(["--esp-ip", "127.0.0.1"])
    params = detector_params(args)
    assert (params.quad_decimate, params.quad_sigma, params.decode_sharpening,
            params.refine_edges) == (1.5, 1.0, 0.75, True)
    assert not args.detail and not args.headless
    assert args.scale == 2.0 and args.frames == 0 and args.save_dir is None
    assert args.config.name == "setup.yaml"


def test_detector_flags_are_honoured():
    args = parse_args(["--esp-ip", "127.0.0.1", "--decimate", "1.0",
                       "--sigma", "0", "--detail"])
    params = detector_params(args)
    assert (params.quad_decimate, params.quad_sigma) == (1.0, 0.0)
    # The rest of the detector is not overridable: it stays the firmware's.
    assert (params.decode_sharpening, params.refine_edges) == (0.75, True)
    assert args.detail


def test_save_dir_is_created(tmp_path):
    target = tmp_path / "run" / "frames"
    args = parse_args(["--esp-ip", "127.0.0.1", "--headless", "--frames", "5",
                       "--save-dir", str(target)])
    assert target.is_dir() and args.frames == 5 and args.headless


@pytest.mark.parametrize("argv", [
    [],                                                   # --esp-ip required
    ["--esp-ip", "127.0.0.1", "--scale", "0"],
    ["--esp-ip", "127.0.0.1", "--decimate", "0.5"],
    ["--esp-ip", "127.0.0.1", "--sigma", "-1"],
    ["--esp-ip", "127.0.0.1", "--frames", "-1"],
    ["--esp-ip", "127.0.0.1", "--fps", "99"],
    ["--esp-ip", "127.0.0.1", "--quality", "5"],
])
def test_bad_arguments_exit_cleanly(argv, capsys):
    with pytest.raises(SystemExit) as exc:
        parse_args(argv)
    assert exc.value.code == 2                # parser.error, not a traceback
    assert capsys.readouterr().err


def test_a_bad_save_dir_exits_cleanly(tmp_path):
    clash = tmp_path / "file"
    clash.write_text("not a directory")
    with pytest.raises(SystemExit):
        parse_args(["--esp-ip", "127.0.0.1", "--save-dir", str(clash)])


def test_esp_ip_is_resolved_to_a_numeric_address():
    # drain_socket() compares against the numeric source address of datagrams.
    assert parse_args(["--esp-ip", "localhost"]).esp_ip.startswith("127.")


def test_reuses_camera_stream_and_tag_debug_rather_than_copying():
    import camera_stream
    import tag_debug
    assert tag_stream.FrameAssembler is camera_stream.FrameAssembler
    assert tag_stream.drain_socket is camera_stream.drain_socket
    assert tag_stream.KEEPALIVE_S is camera_stream.KEEPALIVE_S
    assert tag_stream.COMMAND_PORT is camera_stream.COMMAND_PORT
    assert tag_stream.load_nav_tag_ids is tag_debug.load_nav_tag_ids


def test_wrapping_keeps_the_double_spaces_between_hud_fields():
    # The HUD separates fields with two spaces; splitting on " " alone drops
    # one of each pair, so a wrapped line would silently reflow.
    line = "aaaa bbbb  cccc dddd"
    scale = 0.4
    wide = cv2.getTextSize(line, tag_stream.FONT, scale, 1)[0][0] + 20
    assert tag_stream._hud_wrap([line], wide, scale) == [line]
    narrow = cv2.getTextSize("aaaa bbbb  cccc", tag_stream.FONT,
                             scale, 1)[0][0] + 12
    assert tag_stream._hud_wrap([line], narrow, scale) == ["aaaa bbbb  cccc",
                                                           "dddd"]


def test_a_label_slid_left_to_fit_does_not_bury_an_earlier_one():
    # _text() pulls a right-edge label left so the whole string fits, which can
    # drop it onto one already drawn.
    view = blank()
    taken = []
    tag_stream._text(view, "tag 5 m210 h0 1.02m", 150, 100, (0, 255, 0),
                     0.4, taken=taken)
    tag_stream._text(view, "tag 12? [nav] m0.4 h2 hamming>1", 600, 100,
                     (150, 150, 150), 0.4, taken=taken)
    first, second = taken
    assert second[1] >= first[3]            # pushed onto its own row
    assert second[2] <= view.shape[1] - 2   # still fully on screen


def test_visible_markers_filters_the_picture_only():
    good = EspMarker(5, 148.0, 191.0, 1.1, True, 206.0, 0, 3e-07)
    bad = EspMarker(17, 40.0, 30.0, 1.1, False, 3.0, 2, -1.0)
    markers = [good, bad]
    assert tag_stream.visible_markers(markers, True, True) == markers
    assert tag_stream.visible_markers(markers, True, False) == [good]
    assert tag_stream.visible_markers(markers, False, True) == []


def test_png_name_numbers_by_run_not_by_frame_id():
    # frame_id restarts when the drone reboots mid-recording; run numbering
    # does not, so a recording never overwrites its own earlier frames.
    assert tag_stream.png_name(1) == "tag_stream_000001.png"
    assert len({tag_stream.png_name(n) for n in range(5)}) == 5


def test_esp_record_keeps_the_drone_s_tiny_pose_errors():
    # The drone reports ~4e-07; rounding to 6 decimals would log every one
    # of them as 0.0, which is the one number the record exists to carry.
    m = EspMarker(5, 148.0, 191.0, 1.1, True, 206.0, 0, 3.95e-07)
    assert esp_record(m)["pose_err"] == pytest.approx(3.95e-07, rel=1e-3)


def test_the_plain_label_is_just_the_id_and_the_distance():
    # The demo view: no margin, no hamming, no [nav], nothing to read past
    # "which tag, how far".
    assert det_label(make_det(), frozenset()) == "5  1.20 m"
    assert det_label(make_det(id=14, pose_valid=False, t=None, R=None,
                              pose_err=None), {12, 14}) == "14"


def test_the_plain_view_draws_only_outlines_and_ids():
    """Same frame, plain and --detail: the plain one must put far less ink on
    the picture — no HUD, no corner dot, no drone markers, no rejects."""
    rejected = make_det(id=9, hamming=2, decision_margin=3.0, pose_valid=False,
                        t=None, R=None, pose_err=None,
                        corners=((20.0, 20.0), (40.0, 20.0), (40.0, 40.0),
                                 (20.0, 40.0)))
    dets = [make_det(), rejected]
    markers = [EspMarker(5, 120.0, 120.0, 1.1, True, 206.0, 0, 3e-07)]
    hud = hud_for(n_rejected=1)

    plain = blank()
    annotate(plain, dets, [], [], scale=1.0, show_rejected=False)
    detailed = blank()
    annotate(detailed, dets, markers, hud, scale=1.0, show_rejected=True,
             detail=True)

    assert plain.any()                          # the accepted tag is outlined
    assert not plain[:40, :60].any()            # ...and the reject is not
    assert not plain[:20].any()                 # no HUD rows
    assert np.count_nonzero(plain) < np.count_nonzero(detailed) / 2
