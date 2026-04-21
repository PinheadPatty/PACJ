"""
Capture RG10 via v4l2, repair common buffer layout issues, demosaic, save JPEG.

Edit the TUNING block below; command-line flags only override paths and verbosity.

Discoloration (magenta/green, mesh): often an *odd* RAW_VERTICAL_ROLL_ROWS with the wrong
Bayer line phase vs OpenCV — enable BAYER_ROW_PHASE_COMPENSATE, or use an even roll, or
tune RAW_COLUMN_SHIFT_COLS / BAYER_PATTERN / DEMOSAIC_QUALITY.

After demosaic, optional BGR seam repair (SEAM_REPAIR_*) softens the np.roll boundary for
OpenCV ArUco and other vision on the final color image.
"""

import argparse
import cv2
import numpy as np
import os
import re
import subprocess
import sys
from typing import Optional, Tuple

from take_raw_photo import _BAYER_CODES, _query_v4l2_video_format

# =============================================================================
# TUNING — edit these (avoid duplicating logic on the command line).
# =============================================================================

# --- Sensor (v4l2-ctl on subdev) ---
SENSOR_EXPOSURE = 1200
SENSOR_ANALOGUE_GAIN = 150
SENSOR_DIGITAL_GAIN = 2000

# Capture this many frames in one v4l2 stream; stale data is often only in the first.
# Use 2 so the first frame is dropped and the file keeps the latest (see v4l2-ctl docs).
# If your driver instead *concatenates* frames in the file, enable
# V4L2_PREFER_LAST_FRAME_IF_DOUBLED so we read the last height rows.
V4L2_STREAM_COUNT = 2
V4L2_PREFER_LAST_FRAME_IF_DOUBLED = True

# --- Bayer demosaic (OpenCV): "rg" | "gr" | "bg" | "gb" ---
# Base sensor layout. If BAYER_ROW_PHASE_COMPENSATE is True, "rg"<->"gr" and "bg"<->"gb"
# are chosen automatically when RAW_VERTICAL_ROLL_ROWS is odd (vertical roll by 1 row
# swaps R vs G lines vs what OpenCV expects — common cause of magenta/green + mesh).
BAYER_PATTERN = "rg"

# Flip rg<->gr / bg<->gb when vertical roll is odd (set False only if you tune roll+pattern together).
BAYER_ROW_PHASE_COMPENSATE = True

# Horizontal shift of the Bayer grid in pixels (-1, 0, +1). Try ±1 if color is still wrong.
RAW_COLUMN_SHIFT_COLS = 0

# Demosaic: "fast" (default OpenCV), "ea" edge-aware, "vng" (slower, often fewer false colors).
DEMOSAIC_QUALITY = "ea"

# First row of the DMA buffer to use as the top of the frame. Must satisfy:
#   RAW_ROW_WINDOW_START + frame_height <= total_lines_in_file
# For a normal 720-line capture, only 0 is valid. Use 720 when the buffer has
# 1440 lines (two stacked frames). For a 1-row *alignment* nudge, use
# RAW_VERTICAL_ROLL_ROWS instead — do not use this constant for that.
RAW_ROW_WINDOW_START = 0

# --- Vertical buffer / “wrap” (all in whole rows) ---
# Half-frame reversal (DMA often delivers bottom half first): swap top/bottom halves
# of the Bayer plane before any roll.
SWAP_RAW_TOP_BOTTOM_HALVES = False

# Fine alignment: shifts content along the vertical axis after the optional swap.
# Positive = np.roll(..., +k, axis=0) moves row i to row i-k (content appears to
# shift down). Tune in ±1 steps, or jump by ~height//2 (e.g. 360 @ 720p) to explore.
RAW_VERTICAL_ROLL_ROWS = 410

# Same two controls on the final BGR image (after demosaic). Usually leave off if
# the raw-stage correction is enough.
SWAP_BGR_TOP_BOTTOM_HALVES = False
BGR_VERTICAL_ROLL_ROWS = 0

# --- Raw-roll seam repair (BGR, after demosaic) ---
# np.roll on the Bayer plane leaves a sharp row band that demosaic spreads; interpolating
# those rows on the final BGR image keeps OpenCV ArUco edges cleaner than a bright seam.
SEAM_REPAIR_AFTER_RAW_ROLL = True
# Half-width in rows around the wrap seam: anchors are seam±(half_width+1); try 1–4.
SEAM_REPAIR_HALF_WIDTH = 2

# --- Color / toning (after demosaic) ---
TARGET_LUMA_MEAN = 120.0  # higher → brighter overall
SATURATION_MULTIPLIER = 1.05  # lower reduces false-color pop after bad demosaic (was 1.2)

# Simple gray-world style balance; set False to only scale brightness uniformly.
ENABLE_SIMPLE_AWB = True

# Extra per-channel multipliers after AWB (tint): 1.0 = neutral
MANUAL_B_GAIN = 1.0
MANUAL_G_GAIN = 1.0
MANUAL_R_GAIN = 1.0

# =============================================================================

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_GOOD_OUTPUT_DIR = os.path.join(_SCRIPT_DIR, "good_photo_captures")
_ROLL_SWEEP_DIR = os.path.join(_GOOD_OUTPUT_DIR, "roll_sweep")

# Odd vertical np.roll(..., axis=0) swaps R vs G lines vs OpenCV's BayerRG/GR/etc. naming.
_BAYER_ROW_FLIP = {"rg": "gr", "gr": "rg", "bg": "gb", "gb": "bg"}


def _resolved_bayer_pattern(raw_vertical_roll_rows: int) -> str:
    key = BAYER_PATTERN
    if BAYER_ROW_PHASE_COMPENSATE and (raw_vertical_roll_rows % 2) != 0:
        key = _BAYER_ROW_FLIP.get(key, key)
    return key


def _repair_roll_seam_bgr(
    img: np.ndarray, roll_rows: int, half_width: int
) -> np.ndarray:
    """
    Replace rows near the np.roll wrap boundary (from RAW_VERTICAL_ROLL_ROWS) with linear
    interpolation between clean rows above and below. Operates on uint8 BGR for OpenCV
    ArUco and downstream vision.
    """
    if roll_rows == 0 or half_width <= 0:
        return img
    if img.ndim != 3 or img.shape[2] != 3:
        return img
    h = img.shape[0]
    if h < 3:
        return img

    seam = (h - roll_rows) % h
    if seam == 0:
        return img

    r_top = max(0, seam - half_width - 1)
    r_bot = min(h - 1, seam + half_width + 1)
    if r_bot <= r_top + 1:
        return img

    out = img.astype(np.float32)
    top_f = out[r_top]
    bot_f = out[r_bot]
    span = float(r_bot - r_top)
    for row in range(r_top + 1, r_bot):
        t = (row - r_top) / span
        out[row] = (1.0 - t) * top_f + t * bot_f
    return np.clip(out, 0.0, 255.0).astype(np.uint8)


def _bayer_cv2_code(pattern_key: str, quality: str) -> int:
    q = (quality or "fast").lower()
    if q == "fast":
        return _BAYER_CODES[pattern_key]
    if q not in ("ea", "vng"):
        return _BAYER_CODES[pattern_key]
    const_name = f"COLOR_Bayer{pattern_key.upper()}2BGR_{q.upper()}"
    code = getattr(cv2, const_name, None)
    if code is not None:
        return int(code)
    return _BAYER_CODES[pattern_key]


def _next_good_photo_path() -> str:
    os.makedirs(_GOOD_OUTPUT_DIR, exist_ok=True)
    prefix, suffix = "good_photo_", ".jpg"
    pat = re.compile(rf"^{re.escape(prefix)}(\d{{3}}){re.escape(suffix)}$")
    highest = -1
    for name in os.listdir(_GOOD_OUTPUT_DIR):
        m = pat.match(name)
        if m:
            highest = max(highest, int(m.group(1)))
    n = highest + 1
    return os.path.join(_GOOD_OUTPUT_DIR, f"{prefix}{n:03d}{suffix}")


def _load_raw10_planar_slice(
    path: str,
    width: int,
    height: int,
    bytes_per_line: Optional[int],
    row_offset: int,
) -> Tuple[np.ndarray, dict]:
    file_bytes = os.path.getsize(path)
    if height <= 0:
        raise ValueError("invalid height")

    bpl = bytes_per_line if bytes_per_line and bytes_per_line > 0 else None
    if bpl is not None and file_bytes % bpl == 0:
        pass
    elif file_bytes % (width * 2) == 0:
        bpl = width * 2
    elif file_bytes % height == 0:
        bpl = file_bytes // height
    else:
        raise ValueError(
            f"Cannot infer bytes_per_line: file {file_bytes} bytes, width {width}, "
            f"height hint {height}, driver bpl {bytes_per_line}"
        )

    if bpl % 2 != 0:
        raise ValueError(f"bytes_per_line {bpl} is not 16-bit aligned")

    total_lines = file_bytes // bpl
    stride_px = bpl // 2

    data = np.fromfile(path, dtype="<u2")
    expected = total_lines * stride_px
    if data.size != expected:
        data = data[:expected]

    raw_full = data.reshape((total_lines, stride_px))
    if width > stride_px:
        raise ValueError(f"width {width} > stride pixels {stride_px}")

    if row_offset < 0 or row_offset + height > total_lines:
        raise ValueError(
            f"row window [{row_offset}:{row_offset + height}) out of range "
            f"(buffer has {total_lines} lines of {bpl} bytes each). "
            f"Need total_lines >= {row_offset + height}. "
            "If you only want to shift the picture by a few rows, set "
            "RAW_ROW_WINDOW_START=0 and tune RAW_VERTICAL_ROLL_ROWS instead."
        )
    raw = raw_full[row_offset : row_offset + height, :width]
    dbg = {
        "file_bytes": file_bytes,
        "bytes_per_line": bpl,
        "stride_px": stride_px,
        "total_lines": total_lines,
        "row_offset": row_offset,
        "crop_width": width,
    }
    return raw, dbg


def _apply_vertical_geometry_2d(plane: np.ndarray, swap_halves: bool, roll_rows: int) -> np.ndarray:
    """swap_halves then integer roll along axis=0 (same convention for raw Bayer or BGR)."""
    out = plane
    h = out.shape[0]
    if swap_halves and h >= 2:
        mid = h // 2
        out = np.vstack((out[mid:], out[:mid]))
    if roll_rows:
        out = np.roll(out, roll_rows, axis=0)
    return out


def _capture_one_frame(video: str, subdev: str, raw_path: str) -> bool:
    subprocess.run(
        [
            "v4l2-ctl",
            "-d",
            subdev,
            "-c",
            f"exposure={SENSOR_EXPOSURE},analogue_gain={SENSOR_ANALOGUE_GAIN},digital_gain={SENSOR_DIGITAL_GAIN}",
        ],
        check=False,
    )
    subprocess.run(
        [
            "v4l2-ctl",
            "-d",
            video,
            "--set-fmt-video=width=1280,height=720,pixelformat=RG10",
            "--stream-mmap",
            f"--stream-count={max(1, int(V4L2_STREAM_COUNT))}",
            f"--stream-to={raw_path}",
        ],
        check=False,
    )
    return os.path.exists(raw_path)


def _load_raw_for_processing(
    raw_path: str, video: str
) -> Tuple[np.ndarray, dict, int, int]:
    """Return (raw slice, dbg, width, height)."""
    qw, qh, qbpl = _query_v4l2_video_format(video)
    width, height = qw or 1280, qh or 720
    raw, dbg = _load_raw10_planar_slice(raw_path, width, height, qbpl, RAW_ROW_WINDOW_START)
    if (
        V4L2_STREAM_COUNT > 1
        and V4L2_PREFER_LAST_FRAME_IF_DOUBLED
        and RAW_ROW_WINDOW_START == 0
        and dbg["total_lines"] >= 2 * height
        and dbg["total_lines"] % height == 0
    ):
        last_off = dbg["total_lines"] - height
        raw, dbg = _load_raw10_planar_slice(raw_path, width, height, qbpl, last_off)
    return raw, dbg, width, height


def _raw_to_bgr(raw: np.ndarray, raw_vertical_roll_rows: int) -> np.ndarray:
    """Apply geometry (with given raw roll), debayer, and BGR-stage geometry."""
    raw = _apply_vertical_geometry_2d(
        raw, SWAP_RAW_TOP_BOTTOM_HALVES, raw_vertical_roll_rows
    )
    img8 = ((raw & 0x3FF) >> 2).astype(np.uint8)
    if RAW_COLUMN_SHIFT_COLS:
        img8 = np.roll(img8, RAW_COLUMN_SHIFT_COLS, axis=1)
    pattern_key = _resolved_bayer_pattern(raw_vertical_roll_rows)
    bayer_code = _bayer_cv2_code(pattern_key, DEMOSAIC_QUALITY)
    out = _debayer_and_grade_tuned(img8, bayer_code)
    out = _apply_vertical_geometry_2d(
        out, SWAP_BGR_TOP_BOTTOM_HALVES, BGR_VERTICAL_ROLL_ROWS
    )
    if SEAM_REPAIR_AFTER_RAW_ROLL:
        out = _repair_roll_seam_bgr(out, raw_vertical_roll_rows, SEAM_REPAIR_HALF_WIDTH)
    return out


def sweep_raw_vertical_roll_rows(
    start: int = 550,
    end: int = 600,
    step: int = 1,
    *,
    video: str = "/dev/video0",
    subdev: str = "/dev/v4l-subdev0",
    raw_out: str = "test_raw_auto.bin",
    fresh_capture_per_frame: bool = False,
    verbose: bool = False,
) -> list[str]:
    """
    Save one JPEG per roll value, varying only RAW_VERTICAL_ROLL_ROWS (same raw buffer
    unless fresh_capture_per_frame is True). Inclusive ``start``..``end`` with ``step``.

    Files go to ``good_photo_captures/roll_sweep/roll_XXXX.jpg`` (XXXX = roll value).
    """
    if start > end:
        raise ValueError("start must be <= end")
    if step <= 0:
        raise ValueError("step must be positive")

    if BAYER_PATTERN not in _BAYER_CODES:
        raise ValueError(f"Invalid BAYER_PATTERN {BAYER_PATTERN!r}")

    os.makedirs(_ROLL_SWEEP_DIR, exist_ok=True)

    saved: list[str] = []
    raw: Optional[np.ndarray] = None
    dbg: Optional[dict] = None

    rolls = range(start, end + 1, step)

    for roll in rolls:
        if fresh_capture_per_frame or raw is None:
            if not _capture_one_frame(video, subdev, raw_out):
                raise RuntimeError(f"Capture failed (missing {raw_out})")
            raw, dbg, _w, _h = _load_raw_for_processing(raw_out, video)
            if verbose and dbg is not None:
                print(f"roll={roll} buffer:", dbg)
        assert raw is not None
        out = _raw_to_bgr(raw, roll)
        path = os.path.join(_ROLL_SWEEP_DIR, f"roll_{roll:04d}.jpg")
        cv2.imwrite(path, out)
        saved.append(path)
        print(path)

    return saved


def _debayer_and_grade_tuned(img8: np.ndarray, bayer_code: int) -> np.ndarray:
    color = cv2.cvtColor(img8, bayer_code)

    if ENABLE_SIMPLE_AWB:
        mean_b = float(np.mean(color[:, :, 0]))
        mean_g = float(np.mean(color[:, :, 1]))
        mean_r = float(np.mean(color[:, :, 2]))
        current_mean = (mean_b + mean_g + mean_r) / 3.0
        scale = TARGET_LUMA_MEAN / max(current_mean, 1.0)
        b = np.clip(
            color[:, :, 0] * scale * (mean_g / max(mean_b, 1.0)) * MANUAL_B_GAIN,
            0,
            255,
        ).astype(np.uint8)
        g = np.clip(color[:, :, 1] * scale * MANUAL_G_GAIN, 0, 255).astype(np.uint8)
        r = np.clip(
            color[:, :, 2] * scale * (mean_g / max(mean_r, 1.0)) * MANUAL_R_GAIN,
            0,
            255,
        ).astype(np.uint8)
    else:
        mean = float(np.mean(color))
        scale = TARGET_LUMA_MEAN / max(mean, 1.0)
        b = np.clip(color[:, :, 0] * scale * MANUAL_B_GAIN, 0, 255).astype(np.uint8)
        g = np.clip(color[:, :, 1] * scale * MANUAL_G_GAIN, 0, 255).astype(np.uint8)
        r = np.clip(color[:, :, 2] * scale * MANUAL_R_GAIN, 0, 255).astype(np.uint8)

    final_img = cv2.merge([b, g, r])

    hsv = cv2.cvtColor(final_img, cv2.COLOR_BGR2HSV)
    hsv[:, :, 1] = np.clip(hsv[:, :, 1] * SATURATION_MULTIPLIER, 0, 255)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Capture RG10, apply TUNING constants in this file, save good_photo_*.jpg"
    )
    parser.add_argument("--video", default="/dev/video0")
    parser.add_argument("--subdev", default="/dev/v4l-subdev0")
    parser.add_argument(
        "--raw-out",
        default="test_raw_auto.bin",
        help="Temporary raw path (same default as take_raw_photo for reuse).",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Print inferred buffer geometry.",
    )
    parser.add_argument(
        "--sweep-vertical-roll",
        nargs=2,
        type=int,
        metavar=("START", "END"),
        help="Save a JPEG per roll value (inclusive) to good_photo_captures/roll_sweep/.",
    )
    parser.add_argument(
        "--sweep-vertical-roll-step",
        type=int,
        default=1,
        help="Step between roll values (default 1).",
    )
    parser.add_argument(
        "--sweep-fresh-capture",
        action="store_true",
        help="With --sweep-vertical-roll, capture a new frame for every roll (default: one capture).",
    )
    args = parser.parse_args(argv)

    if BAYER_PATTERN not in _BAYER_CODES:
        print(f"Invalid BAYER_PATTERN {BAYER_PATTERN!r}", file=sys.stderr)
        return 1

    if args.sweep_vertical_roll is not None:
        lo, hi = args.sweep_vertical_roll
        try:
            sweep_raw_vertical_roll_rows(
                lo,
                hi,
                args.sweep_vertical_roll_step,
                video=args.video,
                subdev=args.subdev,
                raw_out=args.raw_out,
                fresh_capture_per_frame=args.sweep_fresh_capture,
                verbose=args.verbose,
            )
        except (ValueError, RuntimeError) as e:
            print(e, file=sys.stderr)
            return 1
        return 0

    if not _capture_one_frame(args.video, args.subdev, args.raw_out):
        print("Capture failed (raw file missing)", file=sys.stderr)
        return 1

    try:
        raw, dbg, _width, height = _load_raw_for_processing(args.raw_out, args.video)
    except ValueError as e:
        print(f"Raw layout error: {e}", file=sys.stderr)
        return 1

    if args.verbose:
        print("Buffer:", dbg)
    elif dbg["total_lines"] != height:
        print(
            f"Note: buffer has {dbg['total_lines']} lines (nominal height {height}). "
            f"Try increasing RAW_ROW_WINDOW_START to {height} in the TUNING block.",
            file=sys.stderr,
        )

    out = _raw_to_bgr(raw, RAW_VERTICAL_ROLL_ROWS)

    out_path = _next_good_photo_path()
    cv2.imwrite(out_path, out)
    print(f"Saved {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
