import json, sys
import cv2, numpy as np

png, spec_path = sys.argv[1], sys.argv[2]
spec = json.load(open(spec_path))
board_img = cv2.imread(png, cv2.IMREAD_GRAYSCALE)
d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec["dictionary"]))
board = cv2.aruco.CharucoBoard_create(spec["squares_x"], spec["squares_y"], spec["square_length_m"], spec["marker_length_m"], d)
print("board_mm", round(spec["physical_size_m"][0]*1000,1), "x", round(spec["physical_size_m"][1]*1000,1))
for W in (120, 150, 180, 220, 260, 320, 400):
    scale = W / board_img.shape[1]
    r = cv2.resize(board_img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    h, w = r.shape
    canvas = np.full((1200, 1920), 80, np.uint8)
    src = np.float32([[0,0],[w,0],[w,h],[0,h]])
    cx, cy = 960, 600
    dst = np.float32([[cx-w//2, cy-h//2-20],[cx+w//2, cy-h//2-30],[cx+w//2+20, cy+h//2],[cx-w//2-10, cy+h//2+10]])
    M = cv2.getPerspectiveTransform(src, dst)
    img = cv2.warpPerspective(r, M, (1920,1200), dst=canvas, borderMode=cv2.BORDER_TRANSPARENT)
    img = cv2.GaussianBlur(img, (3,3), 0.6)
    corners, ids, _ = cv2.aruco.detectMarkers(img, d)
    n_mark = 0 if ids is None else len(ids)
    n_corner = 0
    if ids is not None and len(ids):
        _, cc, ci = cv2.aruco.interpolateCornersCharuco(corners, ids, img, board)
        n_corner = 0 if ci is None else len(ci)
    print("width_px=%3d markers=%2d corners=%2d/24" % (W, n_mark, n_corner))
