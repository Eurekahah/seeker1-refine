#!/usr/bin/env python3
"""Live 2x2 preview of the four fisheye cameras with ChArUco detection.

Runs on the Seeker board. Serves:
  /            simple HTML page (MJPEG)
  /stream.mjpg MJPEG stream
  /snapshot.jpg current grid
  /status.json per-camera detection counts and capture progress
"""
import argparse
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage

CAMERAS = ("left", "right", "bright", "bleft")
TILE_W, TILE_H = 480, 300
GRID_W, GRID_H = TILE_W * 2, TILE_H * 2

lock = threading.Lock()
latest = {c: None for c in CAMERAS}
det = {c: {"count": 0, "pts": None, "stamp": 0.0} for c in CAMERAS}
grid_jpg = b""
board = None
dictionary = None
args = None


def load_board(path):
    spec = json.load(open(path))
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec["dictionary"]))
    b = cv2.aruco.CharucoBoard_create(
        spec["squares_x"], spec["squares_y"],
        spec["square_length_m"], spec["marker_length_m"], d)
    return b, d


def detect(gray, min_markers):
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    if ids is None or len(ids) < min_markers:
        return 0, None
    n, cc, ci = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if ci is None or n < 4:
        return int(n or 0), None
    return int(n), np.asarray(cc, dtype=np.float32).reshape(-1, 2)


def capture_log_line(path):
    try:
        size = os.path.getsize(path)
        with open(path, "rb") as f:
            f.seek(max(0, size - 4096))
            lines = f.read().decode("utf-8", "ignore").splitlines()
        for line in reversed(lines):
            if line.startswith(("PROG", "SAVED", "DONE")):
                return line
    except Exception:
        pass
    return ""


def render_loop():
    global grid_jpg
    last_det = {c: 0.0 for c in CAMERAS}
    period = 1.0 / max(args.fps, 0.5)
    det_period = 1.0 / max(args.detect_hz, 0.1) if board is not None else 1e9
    while not rospy.is_shutdown():
        start = time.time()
        grid = np.zeros((GRID_H, GRID_W, 3), dtype=np.uint8)
        for idx, cam in enumerate(CAMERAS):
            x = (idx % 2) * TILE_W
            y = (idx // 2) * TILE_H
            tile = np.full((TILE_H, TILE_W, 3), 28, dtype=np.uint8)
            with lock:
                item = latest[cam]
            if item is not None:
                raw, stamp = item
                img = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    tile = cv2.resize(img, (TILE_W, TILE_H), interpolation=cv2.INTER_AREA)
                    now = time.time()
                    if board is not None and now - last_det[cam] >= det_period:
                        last_det[cam] = now
                        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                        n, pts = detect(gray, args.min_markers)
                        with lock:
                            det[cam] = {"count": n, "pts": pts, "stamp": now}
                    with lock:
                        info = det[cam]
                    if info["pts"] is not None:
                        sx = TILE_W / float(img.shape[1])
                        sy = TILE_H / float(img.shape[0])
                        for px, py in info["pts"]:
                            cv2.circle(tile, (int(px * sx), int(py * sy)), 3, (0, 255, 0), -1)
            with lock:
                info = det[cam]
            count = info["count"]
            if count >= args.min_corners:
                color, label = (0, 220, 0), "OK"
            elif count > 0:
                color, label = (0, 200, 255), "PARTIAL"
            else:
                color, label = (0, 0, 220), "NO BOARD"
            cv2.rectangle(tile, (0, 0), (TILE_W, 30), (0, 0, 0), -1)
            cv2.putText(tile, "%s  corners=%d  %s" % (cam, count, label),
                        (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
            grid[y:y + TILE_H, x:x + TILE_W] = tile
        ok, jpg = cv2.imencode(".jpg", grid, [int(cv2.IMWRITE_JPEG_QUALITY), 78])
        if ok:
            with lock:
                grid_jpg = jpg.tobytes()
        elapsed = time.time() - start
        time.sleep(max(0.0, period - elapsed))


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *a):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = ("""<!doctype html><html><head><meta charset="utf-8">
<title>Seeker1 preview</title>
<style>body{background:#111;color:#eee;font-family:Segoe UI,Arial,sans-serif;margin:16px}
img{width:100%;max-width:1280px;border:1px solid #333}
pre{background:#1b1b1b;padding:10px;white-space:pre-wrap}</style></head>
<body><h3>Seeker1 4-camera preview</h3>
<img src="/stream.mjpg">
<pre id="s">loading...</pre>
<script>setInterval(async()=>{try{const r=await fetch('/status.json');const j=await r.json();
document.getElementById('s').textContent=JSON.stringify(j,null,2);}catch(e){}},1000)</script>
</body></html>""").encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/status.json":
            with lock:
                data = {
                    "cameras": {c: {"corners": det[c]["count"]} for c in CAMERAS},
                    "capture": capture_log_line(args.capture_log),
                    "min_corners": args.min_corners,
                }
            body = json.dumps(data, ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path in ("/snapshot.jpg", "/stream.jpg"):
            with lock:
                jpg = grid_jpg
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.end_headers()
            self.wfile.write(jpg)
        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            while True:
                with lock:
                    jpg = grid_jpg
                if jpg:
                    try:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ")
                        self.wfile.write(str(len(jpg)).encode())
                        self.wfile.write(b"\r\n\r\n")
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                    except Exception:
                        break
                time.sleep(0.2)
        else:
            self.send_error(404)


def main():
    global args, board, dictionary
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--board-spec", default="/data/local/tmp/board_A3_50mm.json")
    ap.add_argument("--fps", type=float, default=4.0)
    ap.add_argument("--detect-hz", type=float, default=1.0)
    ap.add_argument("--min-corners", type=int, default=10)
    ap.add_argument("--min-markers", type=int, default=4)
    ap.add_argument("--capture-log", default="/tmp/capture.log")
    args = ap.parse_args()

    if os.path.exists(args.board_spec):
        board, dictionary = load_board(args.board_spec)
        print("board spec:", args.board_spec, flush=True)
    else:
        print("board spec not found, detection disabled:", args.board_spec, flush=True)

    rospy.init_node("seeker_preview", anonymous=True)
    for cam in CAMERAS:
        topic = "/fisheye/%s/image_raw/compressed" % cam

        def cb(msg, cam=cam):
            with lock:
                latest[cam] = (bytes(msg.data), msg.header.stamp.to_sec())

        rospy.Subscriber(topic, CompressedImage, cb, queue_size=1, buff_size=2 ** 24)
        print("subscribed", topic, flush=True)

    threading.Thread(target=render_loop, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print("preview on http://0.0.0.0:%d/" % args.port, flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
