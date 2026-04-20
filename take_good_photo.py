"""
Capture RG10 via v4l2, repair common buffer layout issues, demosaic, save JPEG.

Typical failure modes this targets:
  - Row stride padding (handled like take_raw_photo.py)
  - Vertical \"wrap\": frame starts mid-scan so the image is continuous but the top/bottom
    halves are swapped; fix by swapping or rolling the raw frame along rows before debayer.
  - Extra rows in the buffer (nominal height 720 but file holds 1440 lines): pick a slice.
"""

import argparse
import cv2
import numpy as np
import os
import re
import subprocess
import sys
from typing import Optional, Tuple

from take_raw_photo import _BAYER_CODES, _debayer_and_grade, _query_v4l2_video_format

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
    """
    Load RG10 as uint16 rows; crop to width; take `height` rows starting at `row_offset`.

    Uses driver bytes-per-line when it divides the file size (avoids mistaking a 2×-tall
    buffer for a single frame with a doubled stride).
    """
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
            f"(buffer has {total_lines} lines of {bpl} bytes each)"
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


def _unwrap_raw(raw: np.ndarray, mode: str) -> np.ndarray:
    """Fix vertical half-buffer wrap in the Bayer plane before demosaic."""
    h = raw.shape[0]
    if h < 2 or mode == "none" or mode == "swap-bgr":
        return raw
    if mode == "swap-raw":
        mid = h // 2
        return np.vstack((raw[mid:], raw[:mid]))
    if mode == "roll-raw-up":
        mid = h // 2
        return np.roll(raw, -mid, axis=0) if mid else raw
    if mode == "roll-raw-down":
        mid = h // 2
        return np.roll(raw, mid, axis=0) if mid else raw
    raise ValueError(f"unknown unwrap mode {mode!r}")


def _unwrap_bgr(img: np.ndarray, mode: str) -> np.ndarray:
    if mode != "swap-bgr":
        return img
    h = img.shape[0]
    if h < 2:
        return img
    mid = h // 2
    return np.vstack((img[mid:], img[:mid]))


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Capture RG10, fix buffer wrap / row slice, demosaic, save good_photo_*.jpg"
    )
    parser.add_argument("--video", default="/dev/video0")
    parser.add_argument("--subdev", default="/dev/v4l-subdev0")
    parser.add_argument(
        "--bayer",
        choices=list(_BAYER_CODES.keys()),
        default="gb",
        help="Bayer layout (you found gb closest).",
    )
    parser.add_argument(
        "--unwrap",
        choices=("none", "swap-raw", "roll-raw-up", "roll-raw-down", "swap-bgr"),
        default="swap-raw",
        help="Repair vertical wrap: swap-raw/roll-* on Bayer plane; swap-bgr after demosaic.",
    )
    parser.add_argument(
        "--raw-row-offset",
        type=int,
        default=0,
        help="If the driver delivers multiple stacked frames, start reading raw at this row "
        "(e.g. 720 when the file has 1440 lines of 1280-wide RG10).",
    )
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
            "exposure=1200,analogue_gain=150,digital_gain=2000",
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

    qw, qh, qbpl = _query_v4l2_video_format(args.video)
    width, height = qw or 1280, qh or 720

    try:
        raw, dbg = _load_raw10_planar_slice(
            args.raw_out, width, height, qbpl, args.raw_row_offset
        )
    except ValueError as e:
        print(f"Raw layout error: {e}", file=sys.stderr)
        return 1

    if args.verbose:
        print("Buffer:", dbg)
    elif dbg["total_lines"] != height:
        print(
            f"Note: buffer has {dbg['total_lines']} lines (nominal height {height}). "
            f"If the image looks wrong, try --raw-row-offset {height} or run with -v.",
            file=sys.stderr,
        )

    raw = _unwrap_raw(raw, args.unwrap)
    img8 = ((raw & 0x3FF) >> 2).astype(np.uint8)
    out = _debayer_and_grade(img8, _BAYER_CODES[args.bayer])
    out = _unwrap_bgr(out, args.unwrap)

    out_path = _next_good_photo_path()
    cv2.imwrite(out_path, out)
    print(f"Saved {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
