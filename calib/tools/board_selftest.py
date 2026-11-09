#!/usr/bin/env python3
"""Sanity-check that a generated ChArUco board is detectable at camera scale."""
import json
import sys

import cv2
import numpy as np


def main():
    png, spec_path = sys.argv[1], sys.argv[2]
    spec = json.load(open(spec_path))
    board_img = cv2.imread(png, cv2.IMREAD_GRAYSCALE)
    if board_img is None:
        raise SystemExit("cannot read " + png)

    canvas = np.full((1200, 1920), 90, np.uint8)
    scale = 0.60 * canvas.shape[1] / board_img.shape[1]
    resized = cv2.resize(board_img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    h, w = resized.shape
    src = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    dst = np.float32([
        [560, 260], [1360, 200], [1400, 950], [520, 1010],
    ])
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(resized, M, (canvas.shape[1], canvas.shape[0]),
                                 dst=canvas, borderMode=cv2.BORDER_TRANSPARENT)
    warped = cv2.GaussianBlur(warped, (3, 3), 0.8)

    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec["dictionary"]))
    board = cv2.aruco.CharucoBoard_create(
        spec["squares_x"], spec["squares_y"],
        spec["square_length_m"], spec["marker_length_m"], d)
    corners, ids, _ = cv2.aruco.detectMarkers(warped, d)
    n_mark = 0 if ids is None else len(ids)
    n_corner = 0
    if ids is not None and len(ids):
        _, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, warped, board)
        n_corner = 0 if ch_ids is None else len(ch_ids)
    print("opencv", cv2.__version__)
    print("markers", n_mark, "charuco_corners", n_corner,
          "expected_corners", (spec["squares_x"] - 1) * (spec["squares_y"] - 1))
    if n_corner < 12:
        raise SystemExit("FAILED: board not detectable")


if __name__ == "__main__":
    main()
