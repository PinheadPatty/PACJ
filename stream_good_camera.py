#!/usr/bin/env python3
"""
Live preview for the Raspberry Pi CSI / V4L2 RG10 camera (take_good_photo pipeline), not Orbbec.

Uses whatever TUNING you saved in take_good_photo.py (roll, Bayer, AWB, seam repair, v4l2
stream count, etc.) — no duplicate parameters here.

Run on the Pi from the repo root (same folder as take_good_photo.py):
  python3 stream_good_camera.py
Quit: press q or Esc in the window.

If the Pi camera is not /dev/video0, pass --video and --subdev (see v4l2-ctl --list-devices).
"""

import argparse
import sys
import time

import cv2

import take_good_photo as tgp


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", default="/dev/video0")
    parser.add_argument("--subdev", default="/dev/v4l-subdev0")
    parser.add_argument("--raw-out", default="test_raw_auto.bin")
    parser.add_argument(
        "--fps",
        type=float,
        default=5.0,
        help="Target max rate (Hz). Actual rate may be lower if processing is slow.",
    )
    parser.add_argument(
        "--window",
        default="PACJ camera (take_good_photo TUNING)",
        help="OpenCV window title.",
    )
    args = parser.parse_args()

    if tgp.BAYER_PATTERN not in tgp._BAYER_CODES:
        print(f"Invalid BAYER_PATTERN {tgp.BAYER_PATTERN!r}", file=sys.stderr)
        return 1

    min_period = 1.0 / max(args.fps, 0.25)
    print(f"Pipeline module: {tgp.__file__}", flush=True)
    print(
        f"TUNING from take_good_photo.py: roll={tgp.RAW_VERTICAL_ROLL_ROWS}, "
        f"bayer={tgp.BAYER_PATTERN}, swap_raw_halves={tgp.SWAP_RAW_TOP_BOTTOM_HALVES}, "
        f"col_shift={tgp.RAW_COLUMN_SHIFT_COLS}, demosaic={tgp.DEMOSAIC_QUALITY}, "
        f"v4l2_stream_count={tgp.V4L2_STREAM_COUNT}",
        flush=True,
    )
    print(
        f"Devices: video={args.video} subdev={args.subdev} | target <= {args.fps:.1f} Hz | q/Esc quit",
        flush=True,
    )

    try:
        cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)
    except cv2.error as e:
        print(f"No display for OpenCV window: {e}", file=sys.stderr)
        print("Use SSH with X11 forwarding, a local desktop, or run on the Pi with a monitor.", file=sys.stderr)
        return 1

    while True:
        t0 = time.perf_counter()
        if not tgp._capture_one_frame(args.video, args.subdev, args.raw_out):
            print("Capture failed (raw file missing).", file=sys.stderr)
            return 1
        try:
            raw, dbg, _w, height = tgp._load_raw_for_processing(args.raw_out, args.video)
            bgr = tgp._raw_to_bgr(raw, int(tgp.RAW_VERTICAL_ROLL_ROWS))
        except ValueError as e:
            print(f"Pipeline error: {e}", file=sys.stderr)
            return 1

        cv2.imshow(args.window, bgr)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), ord("Q"), 27):
            break

        elapsed = time.perf_counter() - t0
        sleep_s = min_period - elapsed
        if sleep_s > 0:
            time.sleep(sleep_s)

    cv2.destroyWindow(args.window)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
