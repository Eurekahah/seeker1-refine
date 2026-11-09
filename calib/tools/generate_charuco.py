#!/usr/bin/env python3
"""Generate a ChArUco calibration board as a 1:1 printable PNG.

Run inside the ROS container (OpenCV 4.2). The PNG is later wrapped into a
PDF at an exact physical size by make_board_pdf.py on the Windows host.
"""
import argparse
import json
import os

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--squares-x", type=int, default=7)
    ap.add_argument("--squares-y", type=int, default=5)
    ap.add_argument("--square-mm", type=float, default=80.0)
    ap.add_argument("--marker-mm", type=float, default=60.0)
    ap.add_argument("--margin-mm", type=float, default=0.0)
    ap.add_argument("--dpi", type=float, default=300.0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    if not (0 < args.marker_mm < args.square_mm):
        raise SystemExit("marker-mm must be between 0 and square-mm")

    square_m = args.square_mm / 1000.0
    marker_m = args.marker_mm / 1000.0
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard_create(
        args.squares_x, args.squares_y, square_m, marker_m, dictionary
    )

    board_w_px = int(round(args.squares_x * args.square_mm / 25.4 * args.dpi))
    board_h_px = int(round(args.squares_y * args.square_mm / 25.4 * args.dpi))
    img = board.draw((board_w_px, board_h_px), marginSize=0, borderBits=1)

    margin_px = int(round(args.margin_mm / 25.4 * args.dpi))
    if margin_px > 0:
        img = cv2.copyMakeBorder(
            img, margin_px, margin_px, margin_px, margin_px,
            cv2.BORDER_CONSTANT, value=255
        )

    out_dir = os.path.dirname(os.path.abspath(args.out))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    if not cv2.imwrite(args.out, img):
        raise SystemExit("failed to write %s" % args.out)

    spec = {
        "dictionary": "DICT_4X4_50",
        "squares_x": args.squares_x,
        "squares_y": args.squares_y,
        "square_length_m": square_m,
        "marker_length_m": marker_m,
        "margin_m": args.margin_mm / 1000.0,
        "dpi": args.dpi,
        "image_px": [int(img.shape[1]), int(img.shape[0])],
        "physical_size_m": [
            round(img.shape[1] / args.dpi * 0.0254, 5),
            round(img.shape[0] / args.dpi * 0.0254, 5),
        ],
    }
    spec_path = os.path.splitext(args.out)[0] + ".json"
    with open(spec_path, "w", encoding="utf-8") as f:
        json.dump(spec, f, indent=2, ensure_ascii=False)
    print("wrote", args.out, img.shape[1], "x", img.shape[0], "px")
    print("physical", spec["physical_size_m"], "m at", args.dpi, "dpi")
    print("wrote", spec_path)


if __name__ == "__main__":
    main()
