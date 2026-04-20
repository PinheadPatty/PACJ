import argparse
import cv2
import numpy as np
import os
import re
import subprocess
import sys
from typing import Optional

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_OUTPUT_DIR = os.path.join(_SCRIPT_DIR, "normal_photo_captures")

_BAYER_CODES = {
    "rg": cv2.COLOR_BayerRG2BGR,
    "gr": cv2.COLOR_BayerGR2BGR,
    "bg": cv2.COLOR_BayerBG2BGR,
    "gb": cv2.COLOR_BayerGB2BGR,
}


def _next_normal_photo_path() -> str:
    os.makedirs(_OUTPUT_DIR, exist_ok=True)
    prefix, suffix = "normal_photo_", ".jpg"
    pat = re.compile(rf"^{re.escape(prefix)}(\d{{3}})(?:_bayer_[a-z]{{2}})?{re.escape(suffix)}$")
    highest = -1
    for name in os.listdir(_OUTPUT_DIR):
        m = pat.match(name)
        if m:
            highest = max(highest, int(m.group(1)))
    n = highest + 1
    return os.path.join(_OUTPUT_DIR, f"{prefix}{n:03d}{suffix}")


def _query_v4l2_video_format(device: str):
    """Parse width, height, bytes-per-line from v4l2-ctl (bytes-per-line may exceed width*2)."""
    cp = subprocess.run(
        ["v4l2-ctl", "-d", device, "--get-fmt-video"],
        capture_output=True,
        text=True,
    )
    text = (cp.stdout or "") + (cp.stderr or "")
    w = h = bpl = None
    for line in text.splitlines():
        if "Width/Height" in line:
            m = re.search(r"(\d+)\s*/\s*(\d+)", line)
            if m:
                w, h = int(m.group(1)), int(m.group(2))
        if "Bytes Per Line" in line:
            m = re.search(r":\s*(\d+)", line)
            if m:
                bpl = int(m.group(1))
    return w, h, bpl


def _load_raw10_planar(path: str, width: int, height: int, bytes_per_line: Optional[int]):
    """
    Load RG10 (10-bit Bayer in 16-bit little-endian words). Rows may include padding;
    crop to `width` active pixels per row.
    """
    data = np.fromfile(path, dtype="<u2")
    file_bytes = os.path.getsize(path)
    if height <= 0:
        raise ValueError("invalid height")

    if file_bytes % height != 0:
        raise ValueError(
            f"Cannot infer row stride: file is {file_bytes} bytes, height {height}"
        )
    inferred_bpl = file_bytes // height
    if bytes_per_line is None or bytes_per_line <= 0 or file_bytes != height * bytes_per_line:
        bytes_per_line = inferred_bpl

    if bytes_per_line % 2 != 0:
        raise ValueError(f"bytes_per_line {bytes_per_line} is not 16-bit aligned")

    stride_px = bytes_per_line // 2
    expected = height * bytes_per_line
    if file_bytes < expected:
        raise ValueError(
            f"Raw file too small: got {file_bytes} bytes, expected at least {expected} "
            f"(height={height}, bytes_per_line={bytes_per_line})."
        )
    if file_bytes != expected:
        data = data[: height * stride_px]

    raw = data.reshape((height, stride_px))
    if width > stride_px:
        raise ValueError(f"width {width} > stride pixels {stride_px}")
    raw = raw[:, :width]
    return raw


def _debayer_and_grade(img8: np.ndarray, bayer_code: int) -> np.ndarray:
    color = cv2.cvtColor(img8, bayer_code)

    mean_b = np.mean(color[:, :, 0])
    mean_g = np.mean(color[:, :, 1])
    mean_r = np.mean(color[:, :, 2])

    target_mean = 120.0
    current_mean = (mean_b + mean_g + mean_r) / 3.0
    scale = target_mean / max(current_mean, 1.0)

    b = np.clip(color[:, :, 0] * scale * (mean_g / max(mean_b, 1.0)), 0, 255).astype(np.uint8)
    g = np.clip(color[:, :, 1] * scale, 0, 255).astype(np.uint8)
    r = np.clip(color[:, :, 2] * scale * (mean_g / max(mean_r, 1.0)), 0, 255).astype(np.uint8)

    final_img = cv2.merge([b, g, r])

    hsv = cv2.cvtColor(final_img, cv2.COLOR_BGR2HSV)
    hsv[:, :, 1] = np.clip(hsv[:, :, 1] * 1.2, 0, 255)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Capture one RG10 frame via v4l2, demosaic, save JPEG(s)."
    )
    parser.add_argument("--video", default="/dev/video0", help="V4L2 capture device")
    parser.add_argument("--subdev", default="/dev/v4l-subdev0", help="Sensor subdev for exposure/gain")
    parser.add_argument(
        "--bayer",
        choices=list(_BAYER_CODES.keys()),
        default="rg",
        help="Bayer layout for demosaic (try --try-all-bayers if colors look wrong).",
    )
    parser.add_argument(
        "--try-all-bayers",
        action="store_true",
        help="Write four JPEGs (_rg/_gr/_bg/_gb) with the same index so you can pick the best pattern.",
    )
    parser.add_argument(
        "--raw-out",
        default="test_raw_auto.bin",
        help="Temporary raw capture path (cwd unless absolute).",
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
    bytes_per_line = qbpl

    try:
        raw = _load_raw10_planar(args.raw_out, width, height, bytes_per_line)
    except ValueError as e:
        print(f"Raw layout error: {e}", file=sys.stderr)
        return 1

    img8 = ((raw & 0x3FF) >> 2).astype(np.uint8)

    base_path = _next_normal_photo_path()
    root, ext = os.path.splitext(base_path)

    if args.try_all_bayers:
        for key, code in _BAYER_CODES.items():
            out = _debayer_and_grade(img8, code)
            path = f"{root}_bayer_{key}{ext}"
            cv2.imwrite(path, out)
            print(f"Saved {path}")
        return 0

    out = _debayer_and_grade(img8, _BAYER_CODES[args.bayer])
    cv2.imwrite(base_path, out)
    print(f"Saved {base_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
