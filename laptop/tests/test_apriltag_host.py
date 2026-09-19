"""The host build of the drone's own AprilTag detector (laptop/apriltag_host.py).

Markers are rendered with cv2.aruco's DICT_APRILTAG_16h5, which uses the same
code table as components/esp-apriltag, so the IDs here are the IDs the drone
would report.
"""

import shutil
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

import apriltag_host
from apriltag_host import (
    EXPECTED_NCODES,
    FIRMWARE_INTRINSICS,
    GATE_MIN_MARGIN,
    TAG_SIZE_M,
    Detection,
    Detector,
    DetectorParams,
    gate_reason,
    passes_gate,
)

pytestmark = [
    pytest.mark.skipif(
        shutil.which(apriltag_host._compiler_name()) is None,
        reason="no C compiler: cannot build the host AprilTag library"),
    pytest.mark.skipif(
        not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "DICT_APRILTAG_16h5"),
        reason="opencv without aruco tag16h5 markers"),
]

# The rendered marker's black border spans exactly this square, in AprilTag
# pixel coordinates (0, 0 = top-left corner of the top-left pixel).
TAG_X0, TAG_Y0, TAG_PX = 100, 60, 120
IMG_H, IMG_W = 240, 320


@pytest.fixture(scope="module")
def dictionary():
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)


@pytest.fixture(scope="module")
def detector():
    with Detector() as det:
        yield det


def render(dictionary, tag_id, side=TAG_PX, x0=TAG_X0, y0=TAG_Y0, img=None):
    """One marker on a white quiet zone, as a contiguous uint8 (H, W)."""
    if img is None:
        img = np.full((IMG_H, IMG_W), 255, np.uint8)
    img[y0:y0 + side, x0:x0 + side] = cv2.aruco.generateImageMarker(
        dictionary, tag_id, side)
    return img


# tag16h5 is a 4x4 payload inside a 1-cell black border: 6 cells a side.
TAG_CELLS = 6


def flip_one_bit(img, cell=(1, 1), side=TAG_PX, x0=TAG_X0, y0=TAG_Y0):
    """Invert one payload cell of a rendered marker → a hamming-1 detection.

    tag16h5's minimum distance is 5, so one flipped bit still decodes to the
    same ID, which the firmware gate keeps (hamming <= 1).
    """
    px = side // TAG_CELLS
    cx0, cy0 = x0 + cell[0] * px, y0 + cell[1] * px
    out = img.copy()
    out[cy0:cy0 + px, cx0:cx0 + px] = 255 - out[cy0:cy0 + px, cx0:cx0 + px]
    return out


def make_det(**kw) -> Detection:
    base = dict(id=5, hamming=0, decision_margin=200.0, centre=(0.0, 0.0),
                corners=((0.0, 0.0),) * 4, pose_valid=True,
                t=(0.0, 0.0, 1.0), R=tuple(range(9)), pose_err=1e-5)
    return Detection(**{**base, **kw})


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def test_library_builds():
    lib = apriltag_host.library_path()
    assert lib.is_file() and lib.stat().st_size > 0
    assert lib.parent == apriltag_host.BUILD_DIR


def test_warm_build_does_not_run_the_compiler(monkeypatch):
    apriltag_host.library_path()        # make sure it is built and stamped

    def boom(*a, **k):
        raise AssertionError("the compiler must not run on a cache hit")

    monkeypatch.setattr(apriltag_host.subprocess, "run", boom)
    assert apriltag_host.library_path().is_file()


def test_editing_a_header_forces_a_rebuild(tmp_path, monkeypatch):
    """Headers are not on the command line but they are part of the library.

    Patching a struct or a #define under components/esp-apriltag is exactly
    what this repo does, and a stale cached library would then no longer be the
    firmware's detector.  Done on a copy: writing a probe header into the
    tracked component would change the build stamp for every other process.
    """
    src_dir = tmp_path / "apriltag"
    shutil.copytree(apriltag_host._APRILTAG_DIR, src_dir)
    monkeypatch.setattr(apriltag_host, "_APRILTAG_DIR", src_dir)

    srcs = apriltag_host._sources()
    before = apriltag_host._stamp(srcs)
    header = src_dir / "common" / "zarray.h"
    header.write_text(header.read_text() + "\n/* edited by the test */\n")
    assert header in apriltag_host._headers()
    assert apriltag_host._stamp(srcs) != before


def test_no_compiler_gives_an_actionable_error(monkeypatch):
    monkeypatch.setenv("CC", "definitely-not-a-compiler")
    monkeypatch.setattr(apriltag_host.shutil, "which", lambda _: None)
    with pytest.raises(apriltag_host.ApriltagHostError) as exc:
        apriltag_host.library_path(rebuild=True)
    assert "no C compiler" in str(exc.value)


def test_windows_refuses_visual_studio_cl(monkeypatch):
    monkeypatch.setattr(apriltag_host.sys, "platform", "win32")
    monkeypatch.setenv("CC", "cl.exe")
    with pytest.raises(apriltag_host.ApriltagHostError) as exc:
        apriltag_host._compiler()
    assert "conda install -c conda-forge gcc" in str(exc.value)


# ---------------------------------------------------------------------------
# The truncated tag16h5 family (regression guard for the ncodes bug:
# codedata[] held 20 entries while ncodes said 30, so IDs 20-29 read past the
# end of the table and could never be decoded).
# ---------------------------------------------------------------------------

def test_family_ncodes_matches_the_code_table(detector):
    assert detector.ncodes == EXPECTED_NCODES == 22


@pytest.mark.parametrize("tag_id", [0, 1, 5, 11, 12, 19])
def test_tags_decode(detector, dictionary, tag_id):
    dets = detector.detect(render(dictionary, tag_id))
    assert [d.id for d in dets] == [tag_id]
    assert dets[0].hamming == 0


@pytest.mark.parametrize("tag_id", [20, 21])
def test_nav_tags_20_and_21_decode(detector, dictionary, tag_id):
    """These are the IDs the ncodes bug made undetectable."""
    dets = detector.detect(render(dictionary, tag_id))
    assert [d.id for d in dets] == [tag_id]


@pytest.mark.parametrize("tag_id", [22, 25, 29])
def test_ids_outside_the_truncated_table_are_not_detected(detector, dictionary,
                                                          tag_id):
    assert detector.detect(render(dictionary, tag_id)) == []


def test_blank_image_detects_nothing(detector):
    assert detector.detect(np.full((IMG_H, IMG_W), 255, np.uint8)) == []


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------

def test_corners_land_on_the_rendered_square(detector, dictionary):
    det = detector.detect(render(dictionary, 5))[0]
    want = {(TAG_X0, TAG_Y0), (TAG_X0 + TAG_PX, TAG_Y0),
            (TAG_X0, TAG_Y0 + TAG_PX), (TAG_X0 + TAG_PX, TAG_Y0 + TAG_PX)}
    for x, y in det.corners:
        nearest = min(want, key=lambda c: abs(c[0] - x) + abs(c[1] - y))
        assert abs(nearest[0] - x) < 0.25 and abs(nearest[1] - y) < 0.25
    assert len({(round(x), round(y)) for x, y in det.corners}) == 4
    assert det.centre[0] == pytest.approx(TAG_X0 + TAG_PX / 2, abs=0.25)
    assert det.centre[1] == pytest.approx(TAG_Y0 + TAG_PX / 2, abs=0.25)


def test_corner_order_is_stable(detector, dictionary):
    """Corner 0 is what the viewer marks, so it must not wander."""
    a = detector.detect(render(dictionary, 5))[0]
    b = detector.detect(render(dictionary, 5))[0]
    assert a.corners == b.corners


def test_pose_distance_is_sane(detector, dictionary):
    det = detector.detect(render(dictionary, 5))[0]
    assert det.pose_valid and det.pose_err < 1e-3
    # A tag TAG_PX wide at focal length fx sits at about fx * size / width.
    expect_z = FIRMWARE_INTRINSICS.fx * TAG_SIZE_M / TAG_PX
    assert det.t[2] == pytest.approx(expect_z, rel=0.1)
    assert det.range_m == pytest.approx(det.t[2], rel=0.15)
    assert len(det.R) == 9


def test_a_smaller_tag_is_further_away(detector, dictionary):
    near = detector.detect(render(dictionary, 5, side=120))[0]
    far = detector.detect(render(dictionary, 5, side=40, x0=140, y0=100))[0]
    assert far.t[2] > 2 * near.t[2]


# ---------------------------------------------------------------------------
# The firmware quality gate
# ---------------------------------------------------------------------------

def test_a_clean_tag_passes_the_gate(detector, dictionary):
    det = detector.detect(render(dictionary, 5))[0]
    assert det.decision_margin > GATE_MIN_MARGIN
    assert gate_reason(det) is None and passes_gate(det)
    assert det.pose_valid and det.t is not None


def test_pose_is_skipped_for_gate_failures(dictionary):
    """The firmware never poses a detection it would throw away; nor do we."""
    with Detector(min_margin=1e9) as strict:
        det = strict.detect(render(dictionary, 5))[0]
        assert det.id == 5 and det.decision_margin > 0
        assert not det.pose_valid
        assert det.t is None and det.R is None and det.pose_err is None
        assert det.range_m is None
        assert gate_reason(det, strict.min_margin) == "margin<=1e+09"


def test_gate_reasons():
    assert gate_reason(make_det(hamming=2)) == "hamming>1"
    assert gate_reason(make_det(decision_margin=55.0)) == "margin<=55"
    assert gate_reason(make_det(decision_margin=55.1)) is None
    assert not passes_gate(make_det(hamming=2))
    assert passes_gate(make_det(hamming=1, decision_margin=56.0))


def test_firmware_constants_match_at_detect_c():
    p = apriltag_host.FIRMWARE_PARAMS
    assert (p.quad_decimate, p.quad_sigma, p.decode_sharpening,
            p.nthreads, p.refine_edges) == (1.5, 1.0, 0.75, 1, True)
    assert (apriltag_host.GATE_MAX_HAMMING, GATE_MIN_MARGIN) == (1, 55.0)
    assert TAG_SIZE_M == 0.12
    assert (FIRMWARE_INTRINSICS.fx, FIRMWARE_INTRINSICS.fy) == (163.5047216,
                                                                153.22210511)
    assert (FIRMWARE_INTRINSICS.cx, FIRMWARE_INTRINSICS.cy) == (154.00573087,
                                                                107.10222796)


def test_the_margin_gate_is_strictly_greater_in_c_too(detector, dictionary):
    """at_detect.c poses on `margin > 55`, so a margin exactly at the limit is
    rejected — in the shim as well, or the two would disagree at the boundary."""
    img = render(dictionary, 5)
    margin = detector.detect(img)[0].decision_margin
    with Detector(min_margin=margin) as at_limit:
        assert not at_limit.detect(img)[0].pose_valid
    with Detector(min_margin=margin - 1e-6) as just_under:
        assert just_under.detect(img)[0].pose_valid


def test_a_hamming_1_tag_passes_the_gate_and_is_posed(detector, dictionary):
    """The gate is `hamming <= 1`, not `< 1` — one corrected bit still counts."""
    det = detector.detect(flip_one_bit(render(dictionary, 5)))[0]
    assert det.id == 5 and det.hamming == 1
    assert det.decision_margin > GATE_MIN_MARGIN
    assert gate_reason(det) is None and passes_gate(det)
    assert det.pose_valid and det.t is not None and det.pose_err is not None


def test_custom_parameters_reach_the_detector(dictionary):
    """The CLI can override the firmware's parameters (tag_stream --decimate)."""
    params = DetectorParams(quad_decimate=1.0)
    with Detector(params) as det:
        assert det.params.quad_decimate == 1.0
        assert [d.id for d in det.detect(render(dictionary, 7))] == [7]


# ---------------------------------------------------------------------------
# The detector really is the firmware's.  These pin the numbers, not just the
# plumbing: without them the shim could detect with parameters of its own and
# every test above would still pass.
# ---------------------------------------------------------------------------

# tag 5 rendered at TAG_PX, detected with FIRMWARE_PARAMS.  3.4.5 computes
# decision_margin AFTER decode_sharpening, which is why this is in the hundreds
# where pupil-apriltags 3.1.0 scores tens: the firmware's `> 55` gate only
# means something on this scale.
CLEAN_TAG_MARGIN = 360.4


def test_the_decision_margin_scale_is_the_firmwares(detector, dictionary):
    det = detector.detect(render(dictionary, 5))[0]
    assert det.decision_margin == pytest.approx(CLEAN_TAG_MARGIN, rel=0.02)


def test_decode_sharpening_reaches_the_c_detector(dictionary):
    """Sharpening is what lifts the margin onto the firmware's scale."""
    img = render(dictionary, 5)
    with Detector(DetectorParams(decode_sharpening=0.0)) as blunt, \
         Detector(DetectorParams(decode_sharpening=0.75)) as sharp:
        blunt_m = blunt.detect(img)[0].decision_margin
        sharp_m = sharp.detect(img)[0].decision_margin
    assert sharp_m > 2 * blunt_m


def test_quad_decimate_reaches_the_c_detector(dictionary):
    """Decimation decides whether a small tag is FOUND, not what it scores.

    Quads are searched in the decimated image while the payload is decoded from
    the original, so decimation leaves the margin alone and instead costs small
    tags outright: this 12 px one survives the firmware's 1.5 and is gone by 3.0.
    """
    img = render(dictionary, 5, side=12, x0=140, y0=100)
    with Detector(DetectorParams(quad_decimate=1.5, quad_sigma=0.0)) as fine, \
         Detector(DetectorParams(quad_decimate=3.0, quad_sigma=0.0)) as coarse:
        assert [d.id for d in fine.detect(img)] == [5]
        assert [d for d in coarse.detect(img) if d.id == 5] == []


def test_quad_sigma_reaches_the_c_detector(dictionary):
    """The blur only touches the decode when it is not decimated away.

    apriltag.c decodes the payload from the ORIGINAL image and only finds quads
    in the decimated one, so at quad_decimate 1.5 the two are separate buffers
    and quad_sigma leaves the margin alone.  At 1.0 they are the same buffer,
    the blur lands on what is decoded — and that is why detect() copies there.
    """
    img = render(dictionary, 5, side=24, x0=140, y0=100)
    with Detector(DetectorParams(quad_decimate=1.0, quad_sigma=0.0)) as crisp, \
         Detector(DetectorParams(quad_decimate=1.0, quad_sigma=1.0)) as blurred:
        crisp_m = crisp.detect(img)[0].decision_margin
        blurred_m = blurred.detect(img)[0].decision_margin
    assert blurred_m < crisp_m - 30


def test_refine_edges_reaches_the_c_detector(dictionary):
    """With refinement the corners land on the rendered square; without, ~0.5 px off."""
    img = render(dictionary, 5)
    want = {(TAG_X0, TAG_Y0), (TAG_X0 + TAG_PX, TAG_Y0),
            (TAG_X0, TAG_Y0 + TAG_PX), (TAG_X0 + TAG_PX, TAG_Y0 + TAG_PX)}

    def worst_corner_error(refine: bool) -> float:
        with Detector(DetectorParams(refine_edges=refine)) as det:
            corners = det.detect(img)[0].corners
        return max(min(abs(wx - x) + abs(wy - y) for wx, wy in want)
                   for x, y in corners)

    assert worst_corner_error(True) < 0.1
    assert worst_corner_error(False) > 0.2


# ---------------------------------------------------------------------------
# Input handling and lifecycle
# ---------------------------------------------------------------------------

def test_detect_rejects_anything_but_a_contiguous_uint8_image(detector):
    with pytest.raises(TypeError):
        detector.detect([[0, 1], [2, 3]])
    with pytest.raises(TypeError):
        detector.detect(np.zeros((8, 8), np.float32))
    with pytest.raises(ValueError):
        detector.detect(np.zeros((8, 8, 3), np.uint8))
    with pytest.raises(ValueError):
        detector.detect(np.zeros((8, 8), np.uint8)[:, ::2])   # not contiguous
    with pytest.raises(ValueError):
        detector.detect(np.zeros((0, 8), np.uint8))


@pytest.mark.parametrize("params", [
    DetectorParams(quad_decimate=1.0, quad_sigma=1.0),   # blurs in place
    DetectorParams(),                                    # firmware defaults
])
def test_detect_leaves_the_caller_s_image_alone(dictionary, params):
    """apriltag.c only copies the image for the decimate step (quad_decimate >
    1); at 1.0 the quad_sigma blur lands in the caller's buffer, so
    `tag_stream.py --decimate 1.0` would display and save a blurred frame."""
    img = render(dictionary, 5)
    before = img.copy()
    with Detector(params) as det:
        assert [d.id for d in det.detect(img)] == [5]
    assert np.array_equal(img, before)


def test_extra_detections_are_counted_not_returned(dictionary):
    """Beyond max_detections the count must still be honest (HUD, jsonl)."""
    img = render(dictionary, 5, side=80, x0=20, y0=60)
    render(dictionary, 7, side=80, x0=200, y0=60, img=img)
    with Detector() as det:
        assert sorted(d.id for d in det.detect(img)) == [5, 7]
        assert det.truncated == 0
    with Detector(max_detections=1) as clipped:
        assert len(clipped.detect(img)) == 1
        assert clipped.truncated == 1


def test_a_non_contiguous_view_works_once_copied(detector, dictionary):
    img = render(dictionary, 5)
    flipped = img[:, ::-1]
    with pytest.raises(ValueError):
        detector.detect(flipped)
    assert detector.detect(np.ascontiguousarray(flipped)) is not None


def test_close_is_idempotent_and_detect_after_close_raises(dictionary):
    det = Detector()
    det.detect(render(dictionary, 5))
    det.close()
    det.close()
    with pytest.raises(apriltag_host.ApriltagHostError):
        det.detect(render(dictionary, 5))


def test_many_detectors_do_not_leak_the_c_objects(dictionary):
    """Each Detector owns a C detector and family; 40 in a row must be fine."""
    img = render(dictionary, 5)
    for _ in range(40):
        with Detector() as det:
            assert [d.id for d in det.detect(img)] == [5]
