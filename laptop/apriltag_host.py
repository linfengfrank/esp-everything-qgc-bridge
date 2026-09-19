#!/usr/bin/env python3
"""AprilTag detection on the laptop using the drone's OWN detector.

The firmware detects with components/esp-apriltag (AprilTag 3.4.5, a truncated
tag16h5 code table).  A pip-installed AprilTag is not a substitute: 3.1.0
computes decision_margin before decode_sharpening, so the same real tag scores
~207 in the firmware and ~58 in pupil-apriltags, which makes the firmware's
`margin > 55` gate meaningless.  This module compiles the in-tree sources for
the host (apriltag_host_shim.c gives them a flat C API) and drives them through
ctypes, so laptop-side detections carry firmware semantics.

    from apriltag_host import Detector, FIRMWARE_PARAMS, passes_gate
    with Detector() as det:
        for d in det.detect(gray):          # gray: contiguous uint8 (H, W)
            print(d.id, d.decision_margin, passes_gate(d))

The library is built on first use into laptop/.apriltag-host/ (gitignored) and
rebuilt only when a source, a header, the shim or the build command changes.
"""

from __future__ import annotations

import ctypes
import hashlib
import os
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

# ---------------------------------------------------------------------------
# Firmware constants — mirror main/at_detect.c (at_detect_task).  Change them
# here when the firmware changes, and nowhere else on the laptop side.
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class DetectorParams:
    quad_decimate:     float = 1.5
    quad_sigma:        float = 1.0
    refine_edges:      bool  = True
    decode_sharpening: float = 0.75
    nthreads:          int   = 1

    def summary(self) -> str:
        return (f"dec {self.quad_decimate:g} sig {self.quad_sigma:g} "
                f"sharp {self.decode_sharpening:g} "
                f"refine {int(self.refine_edges)}")

    @property
    def blurs_in_place(self) -> bool:
        """True when detecting would write the blur into the caller's buffer.

        apriltag.c only copies the image when quad_decimate > 1 (the decimate
        step allocates); at <= 1 the quad_sigma blur lands on the image it was
        handed.  quad_decimate is a float in C, so compare the value it holds.
        """
        decimate = ctypes.c_float(self.quad_decimate).value
        return self.quad_sigma != 0.0 and not decimate > 1.0


@dataclass(frozen=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


FIRMWARE_PARAMS = DetectorParams()          # at_detect.c sets exactly these
FIRMWARE_INTRINSICS = Intrinsics(fx=163.5047216, fy=153.22210511,
                                 cx=154.00573087, cy=107.10222796)
TAG_SIZE_M = 0.12                           # TAG_SIZE in at_detect.c

# Quality gate: `det->hamming <= 1 && det->decision_margin > 55.0`
GATE_MAX_HAMMING = 1
GATE_MIN_MARGIN  = 55.0

# Firmware acts on a pose only when estimate_tag_pose() error is below this.
POSE_ERR_LIMIT = 0.5

# AprilTag puts (0, 0) at the top-left CORNER of the top-left pixel; OpenCV
# puts it at that pixel's CENTRE.  Detections below are in AprilTag
# convention; subtract this once, where you draw or call solvePnP.
PIXEL_CENTRE_OFFSET = 0.5

# tag16h5 as this project ships it: IDs 0-21 (0-11 landing, 12-21 nav).
EXPECTED_NCODES = 22

MAX_DETECTIONS = 32     # per frame; extras are counted, not returned

# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parent
_APRILTAG_DIR = _REPO / "components" / "esp-apriltag" / "apriltag"
_SHIM = _HERE / "apriltag_host_shim.c"
BUILD_DIR = _HERE / ".apriltag-host"

_LIB_NAME = ("libapriltag_host.dylib" if sys.platform == "darwin"
             else "libapriltag_host.so")
_STAMP_NAME = "build.stamp"

_ABI_VERSION = 1        # must match ATH_ABI_VERSION in apriltag_host_shim.c


class ApriltagHostError(RuntimeError):
    """The host AprilTag library could not be built or loaded."""


def _sources() -> list[Path]:
    """The firmware's AprilTag sources, minus the ESP-only bits.

    getopt.c and pjpeg*.c are upstream example/JPEG helpers: nothing here calls
    them and they drag in extra symbols.  apriltag_psram_alloc.h is deliberately
    NOT force-included — that header is ESP-only.
    """
    srcs = [_APRILTAG_DIR / n for n in ("apriltag.c", "apriltag_quad_thresh.c",
                                        "apriltag_pose.c", "tag16h5.c")]
    srcs += sorted(p for p in (_APRILTAG_DIR / "common").glob("*.c")
                   if p.name != "getopt.c" and not p.name.startswith("pjpeg"))
    srcs.append(_SHIM)
    missing = [p for p in srcs if not p.is_file()]
    if missing:
        raise ApriltagHostError(
            "missing AprilTag sources: " + ", ".join(str(p) for p in missing))
    return srcs


def _headers() -> list[Path]:
    """Every header the build can include from the in-tree AprilTag tree.

    They are not on the compiler's command line, but a struct or #define edited
    there changes the library as much as a .c does — and patching those headers
    in place is exactly what this repo does.
    """
    return sorted(_APRILTAG_DIR.rglob("*.h"))


def _compiler_name() -> str:
    """What to compile with, unresolved: a cache hit then needs no PATH lookup,
    and a machine with no compiler can still use an already-built library."""
    return os.environ.get("CC") or "cc"


def _compiler() -> str:
    cc = _compiler_name()
    found = shutil.which(cc)
    if found is None:
        raise ApriltagHostError(
            f"no C compiler: {cc!r} is not on PATH.  Install the Xcode command "
            "line tools (xcode-select --install) or set CC to a working "
            "compiler; laptop/apriltag_host.py needs one to build the drone's "
            "AprilTag detector for this machine.")
    return found


# -ffp-contract=off keeps the host from fusing multiply-adds, so margins and
# poses stay comparable with the drone's.
_CFLAGS = ["-O2", "-ffp-contract=off", "-shared", "-fPIC", "-w"]
_LDFLAGS = ["-lm", "-lpthread"]


def _build_args(out: Path) -> list[str]:
    """Everything on the command line except the compiler and the sources."""
    return ([*_CFLAGS, f"-I{_APRILTAG_DIR}", f"-I{_APRILTAG_DIR / 'common'}"]
            + ["-o", str(out)] + _LDFLAGS)


def _stamp(srcs: list[Path]) -> str:
    """Identity of this build: every source and header, plus how it is compiled.

    Reading ~60 small files is a few milliseconds and needs no compiler, so a
    cache hit stays cheap while any edit under components/esp-apriltag, .c or
    .h, still forces a rebuild.
    """
    h = hashlib.sha256()
    h.update(f"abi{_ABI_VERSION}\n".encode())
    # The output path varies (temp file), so hash the flags, not the full line.
    h.update("\n".join([_compiler_name(), *_CFLAGS, *_LDFLAGS,
                        str(_APRILTAG_DIR)]).encode())
    for p in srcs:
        h.update(p.name.encode())
        h.update(p.read_bytes())
    for p in _headers():
        h.update(str(p.relative_to(_APRILTAG_DIR)).encode())
        h.update(p.read_bytes())
    return h.hexdigest()


def library_path(rebuild: bool = False) -> Path:
    """Path to the built shared library, compiling it if it is out of date."""
    srcs = _sources()
    lib = BUILD_DIR / _LIB_NAME
    stamp_file = BUILD_DIR / _STAMP_NAME
    want = _stamp(srcs)
    if not rebuild and lib.is_file():
        try:
            if stamp_file.read_text().strip() == want:
                return lib                      # cache hit: no compiler run
        except OSError:
            pass

    cc = _compiler()
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    print(f"apriltag_host: compiling {len(srcs)} sources into "
          f"{BUILD_DIR.name}/ (a few seconds, once)", file=sys.stderr)
    # Compile to a temp name and rename, so a failed or concurrent build never
    # leaves a half-written library behind a valid stamp.
    fd, tmp_name = tempfile.mkstemp(dir=BUILD_DIR, suffix=".tmp")
    os.close(fd)
    tmp = Path(tmp_name)
    cmd = [cc, *[str(p) for p in srcs], *_build_args(tmp)]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if proc.returncode != 0:
            tail = "\n".join((proc.stderr or proc.stdout).splitlines()[-20:])
            raise ApriltagHostError(
                f"building the host AprilTag library failed "
                f"(exit {proc.returncode}).\n"
                f"  command: {' '.join(cmd)}\n"
                f"  compiler output (last lines):\n{tail}")
        os.replace(tmp, lib)
    finally:
        tmp.unlink(missing_ok=True)
    stamp_file.write_text(want + "\n")
    return lib


# ---------------------------------------------------------------------------
# ctypes binding
# ---------------------------------------------------------------------------

class _CDetection(ctypes.Structure):
    """Must match ath_detection_t in apriltag_host_shim.c."""
    _fields_ = [
        ("id",              ctypes.c_int32),
        ("hamming",         ctypes.c_int32),
        ("pose_valid",      ctypes.c_int32),
        ("reserved",        ctypes.c_int32),
        ("decision_margin", ctypes.c_double),
        ("c",               ctypes.c_double * 2),
        ("p",               (ctypes.c_double * 2) * 4),
        ("t",               ctypes.c_double * 3),
        ("R",               ctypes.c_double * 9),
        ("pose_err",        ctypes.c_double),
    ]


_lib = None     # loaded once per process; unloading a CDLL is not supported


def _load() -> ctypes.CDLL:
    global _lib
    if _lib is not None:
        return _lib
    path = library_path()
    try:
        lib = ctypes.CDLL(str(path))
    except OSError as exc:
        raise ApriltagHostError(f"cannot load {path}: {exc}") from exc

    lib.ath_abi_version.restype = ctypes.c_int
    lib.ath_abi_version.argtypes = []
    lib.ath_detector_create.restype = ctypes.c_void_p
    lib.ath_detector_create.argtypes = [ctypes.c_double, ctypes.c_double,
                                        ctypes.c_int, ctypes.c_double,
                                        ctypes.c_int]
    lib.ath_detector_destroy.restype = None
    lib.ath_detector_destroy.argtypes = [ctypes.c_void_p]
    lib.ath_family_ncodes.restype = ctypes.c_int
    lib.ath_family_ncodes.argtypes = [ctypes.c_void_p]
    lib.ath_detect.restype = ctypes.c_int
    lib.ath_detect.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint8),
        ctypes.c_int32, ctypes.c_int32, ctypes.c_int32,
        ctypes.c_double, ctypes.c_double, ctypes.c_double,
        ctypes.c_double, ctypes.c_double,
        ctypes.c_int32, ctypes.c_double,
        ctypes.POINTER(_CDetection), ctypes.c_int32,
    ]

    got = lib.ath_abi_version()
    if got != _ABI_VERSION:
        raise ApriltagHostError(
            f"{path} has ABI {got}, this module expects {_ABI_VERSION}; "
            f"delete {BUILD_DIR} and retry")
    _lib = lib
    return _lib


@dataclass(frozen=True)
class Detection:
    """One AprilTag detection, in AprilTag pixel convention.

    Subtract PIXEL_CENTRE_OFFSET from centre/corners before drawing them on an
    OpenCV image or feeding them to solvePnP — once, not twice.
    """
    id:              int
    hamming:         int
    decision_margin: float
    centre:          tuple[float, float]
    corners:         tuple[tuple[float, float], ...]   # 4, counter-clockwise
    pose_valid:      bool
    t:               tuple[float, float, float] | None   # camera frame (m)
    R:               tuple[float, ...] | None            # 9, row-major
    pose_err:        float | None

    @property
    def range_m(self) -> float | None:
        """Straight-line camera-to-tag distance (m), or None without a pose."""
        if self.t is None:
            return None
        return (self.t[0] ** 2 + self.t[1] ** 2 + self.t[2] ** 2) ** 0.5


def gate_reason(det: Detection, min_margin: float = GATE_MIN_MARGIN,
                max_hamming: int = GATE_MAX_HAMMING) -> str | None:
    """Why the firmware would throw this detection away, or None if it keeps it."""
    if det.hamming > max_hamming:
        return f"hamming>{max_hamming}"
    if det.decision_margin <= min_margin:
        return f"margin<={min_margin:g}"
    return None


def passes_gate(det: Detection, min_margin: float = GATE_MIN_MARGIN,
                max_hamming: int = GATE_MAX_HAMMING) -> bool:
    return gate_reason(det, min_margin, max_hamming) is None


class Detector:
    """The firmware's AprilTag detector, running on this machine.

    Holds C resources: use it as a context manager, or call close().
    """

    def __init__(self, params: DetectorParams = FIRMWARE_PARAMS,
                 intrinsics: Intrinsics = FIRMWARE_INTRINSICS,
                 tag_size_m: float = TAG_SIZE_M,
                 min_margin: float = GATE_MIN_MARGIN,
                 max_hamming: int = GATE_MAX_HAMMING,
                 max_detections: int = MAX_DETECTIONS):
        self.params = params
        self.intrinsics = intrinsics
        self.tag_size_m = tag_size_m
        self.min_margin = min_margin
        self.max_hamming = max_hamming
        self.truncated = 0          # detections dropped by max_detections
        self._lib = _load()
        self._buf = (_CDetection * max(1, max_detections))()
        self._handle = self._lib.ath_detector_create(
            float(params.quad_decimate), float(params.quad_sigma),
            int(bool(params.refine_edges)), float(params.decode_sharpening),
            int(params.nthreads))
        if not self._handle:
            raise ApriltagHostError("ath_detector_create() returned NULL")

    # -- lifecycle ----------------------------------------------------------

    def close(self) -> None:
        handle, self._handle = getattr(self, "_handle", None), None
        if handle:
            self._lib.ath_detector_destroy(handle)

    def __enter__(self) -> "Detector":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:       # interpreter teardown: nothing useful to do
            pass

    # -- detection ----------------------------------------------------------

    @property
    def ncodes(self) -> int:
        """IDs the compiled tag16h5 family can decode (22 in this repo)."""
        self._check_open()
        return self._lib.ath_family_ncodes(self._handle)

    def _check_open(self) -> None:
        if not getattr(self, "_handle", None):
            raise ApriltagHostError("Detector is closed")

    def detect(self, gray) -> list[Detection]:
        """Detect in a contiguous uint8 (H, W) numpy array.

        `gray` is never modified.  The C library borrows the buffer, exactly as
        the firmware detects straight on the camera frame buffer (stride ==
        width) — but it blurs that buffer in place when quad_decimate <= 1 and
        quad_sigma != 0, so those parameters detect on a private copy instead.
        """
        import numpy as np

        self._check_open()
        if not isinstance(gray, np.ndarray):
            raise TypeError(f"detect() wants a numpy array, got {type(gray).__name__}")
        if gray.dtype != np.uint8:
            raise TypeError(f"detect() wants dtype uint8, got {gray.dtype}")
        if gray.ndim != 2:
            raise ValueError(f"detect() wants a 2-D (H, W) grayscale image, "
                             f"got shape {gray.shape}")
        if not gray.flags["C_CONTIGUOUS"]:
            raise ValueError("detect() wants a C-contiguous array; "
                             "use numpy.ascontiguousarray()")
        height, width = gray.shape
        if width < 1 or height < 1:
            raise ValueError(f"detect() wants a non-empty image, got {gray.shape}")

        # Kept alive by this local for the whole call.
        buf = gray.copy(order="C") if self.params.blurs_in_place else gray
        ptr = buf.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
        total = self._lib.ath_detect(
            self._handle, ptr, width, height, width,
            self.tag_size_m, self.intrinsics.fx, self.intrinsics.fy,
            self.intrinsics.cx, self.intrinsics.cy,
            self.max_hamming, self.min_margin,
            self._buf, len(self._buf))
        if total < 0:
            raise ApriltagHostError("ath_detect() rejected its arguments")
        kept = min(total, len(self._buf))
        self.truncated = total - kept
        return [_to_detection(self._buf[i]) for i in range(kept)]


def _to_detection(c: _CDetection) -> Detection:
    valid = bool(c.pose_valid)
    return Detection(
        id              = int(c.id),
        hamming         = int(c.hamming),
        decision_margin = float(c.decision_margin),
        centre          = (c.c[0], c.c[1]),
        corners         = tuple((c.p[i][0], c.p[i][1]) for i in range(4)),
        pose_valid      = valid,
        t               = (c.t[0], c.t[1], c.t[2]) if valid else None,
        R               = tuple(c.R) if valid else None,
        pose_err        = float(c.pose_err) if valid else None,
    )


if __name__ == "__main__":      # build check: python3 laptop/apriltag_host.py
    with Detector() as _d:
        print(f"built {library_path()}")
        print(f"tag16h5 ncodes = {_d.ncodes} (expected {EXPECTED_NCODES})")
        print(f"params: {_d.params.summary()}  gate: hamming <= "
              f"{_d.max_hamming}, margin > {_d.min_margin:g}")
