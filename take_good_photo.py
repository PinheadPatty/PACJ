"""
Capture RG10 via v4l2, repair common buffer layout issues, demosaic, save JPEG.

Edit the TUNING block below; command-line flags only override paths and verbosity.
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

# --- Bayer demosaic (OpenCV): "rg" | "gr" | "bg" | "gb" ---
BAYER_PATTERN = "rg"

# First row of the DMA buffer to use as the top of the frame. Must satisfy:
#   RAW_ROW_WINDOW_START + frame_height <= total_lines_in_file
# For a normal 720-line capture, only 0 is valid. Use 720 when the buffer has
# 1440 lines (two stacked frames). For a 1-row *alignment* nudge, use
# RAW_VERTICAL_ROLL_ROWS instead — do not use this constant for that.
RAW_ROW_WINDOW_START = 0

# --- Vertical buffer / “wrap” (all in whole rows) ---
# Half-frame reversal (DMA often delivers bottom half first): swap top/bottom halves
# of the Bayer plane before any roll.
SWAP_RAW_TOP_BOTTOM_HALVES = True

# Fine alignment: shifts content along the vertical axis after the optional swap.
# Positive = np.roll(..., +k, axis=0) moves row i to row i-k (content appears to
# shift down). Tune in ±1 steps, or jump by ~height//2 (e.g. 360 @ 720p) to explore.
RAW_VERTICAL_ROLL_ROWS = 200

# Same two controls on the final BGR image (after demosaic). Usually leave off if
# the raw-stage correction is enough.
SWAP_BGR_TOP_BOTTOM_HALVES = False
BGR_VERTICAL_ROLL_ROWS = 0

# --- Color / toning (after demosaic) ---
TARGET_LUMA_MEAN = 120.0  # higher → brighter overall
SATURATION_MULTIPLIER = 1.2  # 1.0 = no change; <1 mutes, >1 boosts color

# Simple gray-world style balance; set False to only scale brightness uniformly.
ENABLE_SIMPLE_AWB = True

# Extra per-channel multipliers after AWB (tint): 1.0 = neutral
MANUAL_B_GAIN = 1.0
MANUAL_G_GAIN = 1.0
MANUAL_R_GAIN = 1.0

# =============================================================================

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_GOOD_OUTPUT_DIR = os.path.join(_SCRIPT_DIR, "good_photo_captures")


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
    args = parser.parse_args(argv)

    subprocess.run(
        [
            "v4l2-ctl",
            "-d",
            args.subdev,
            "-c",
            f"exposure={SENSOR_EXPOSURE},analogue_gain={SENSOR_ANALOGUE_GAIN},digital_gain={SENSOR_DIGITAL_GAIN}",
        ],
        check=False,
    )
    subprocess.run(
        [
            "v4l2-ctl",
            "-d",
            args.video,
            "--set-fmt-video=width=1280,height=720,pixelformat=RG10",
            "--stream-mmap",
            "--stream-count=1",
            f"--stream-to={args.raw_out}",
        ],
        check=False,
    )

    if not os.path.exists(args.raw_out):
        print("Capture failed (raw file missing)", file=sys.stderr)
        return 1

    if BAYER_PATTERN not in _BAYER_CODES:
        print(f"Invalid BAYER_PATTERN {BAYER_PATTERN!r}", file=sys.stderr)
        return 1

    qw, qh, qbpl = _query_v4l2_video_format(args.video)
    width, height = qw or 1280, qh or 720

    try:
        raw, dbg = _load_raw10_planar_slice(
            args.raw_out, width, height, qbpl, RAW_ROW_WINDOW_START
        )
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

    raw = _apply_vertical_geometry_2d(
        raw, SWAP_RAW_TOP_BOTTOM_HALVES, RAW_VERTICAL_ROLL_ROWS
    )
    img8 = ((raw & 0x3FF) >> 2).astype(np.uint8)
    out = _debayer_and_grade_tuned(img8, _BAYER_CODES[BAYER_PATTERN])
    out = _apply_vertical_geometry_2d(
        out, SWAP_BGR_TOP_BOTTOM_HALVES, BGR_VERTICAL_ROLL_ROWS
    )

    out_path = _next_good_photo_path()
    cv2.imwrite(out_path, out)
    print(f"Saved {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
