#!/usr/bin/env python3
"""Serve the live /panorama/image topic as an MJPEG web page."""
import argparse
import json
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image

lock = threading.Lock()
latest_jpg = b""
latest_stamp = 0.0
frame_count = 0
frame_times = deque(maxlen=60)
info = {"width": 0, "height": 0, "encoding": "", "topic": ""}


def decode_image(msg):
    h, w = int(msg.height), int(msg.width)
    enc = msg.encoding.lower()
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    channels = 3 if enc in ("bgr8", "rgb8") else 4 if enc in ("bgra8", "rgba8") else 1
    row_bytes = w * channels
    if msg.step < row_bytes or raw.size < msg.step * h:
        return None
    rows = raw.reshape(h, msg.step)[:, :row_bytes]
    if channels == 3:
        img = rows.reshape(h, w, 3)
        if enc == "rgb8":
            img = img[:, :, ::-1].copy()
        return img
    if channels == 4:
        img = rows.reshape(h, w, 4)
        if enc == "rgba8":
            img = img[:, :, [2, 1, 0, 3]].copy()
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return cv2.cvtColor(rows.reshape(h, w), cv2.COLOR_GRAY2BGR)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def send_bytes(self, code, content_type, body):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = """<!doctype html><html><head><meta charset="utf-8">
<title>Seeker1 panorama</title>
<style>body{background:#111;color:#eee;font-family:Segoe UI,Arial,sans-serif;margin:16px}
img{width:100%;max-width:1280px;background:#000;border:1px solid #333}
pre{background:#1b1b1b;padding:10px;white-space:pre-wrap}</style></head>
<body><h3>Seeker1 实时全景拼接</h3><img src="/stream.mjpg">
<pre id="s">loading...</pre><script>
setInterval(async()=>{try{const r=await fetch('/status.json');const j=await r.json();
document.getElementById('s').textContent=JSON.stringify(j,null,2);}catch(e){}},1000)
</script></body></html>""".encode("utf-8")
            self.send_bytes(200, "text/html; charset=utf-8", body)
        elif self.path == "/status.json":
            with lock:
                age = (rospy.get_time() - latest_stamp) if latest_stamp else -1.0
                now = time.time()
                recent = [t for t in frame_times if now - t <= 2.0]
                data = {
                    "topic": info["topic"], "width": info["width"],
                    "height": info["height"], "encoding": info["encoding"],
                    "frames": frame_count, "last_frame_age_s": round(age, 3),
                    "fps": round(len(recent) / 2.0, 2),
                }
            self.send_bytes(200, "application/json", json.dumps(data).encode("utf-8"))
        elif self.path in ("/snapshot.jpg", "/stream.jpg"):
            with lock:
                jpg = latest_jpg
            if not jpg:
                self.send_error(503, "no panorama frame yet")
                return
            self.send_bytes(200, "image/jpeg", jpg)
        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            while not rospy.is_shutdown():
                with lock:
                    jpg = latest_jpg
                if jpg:
                    try:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ")
                        self.wfile.write(str(len(jpg)).encode())
                        self.wfile.write(b"\r\n\r\n")
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                    except Exception:
                        break
                time.sleep(0.1)
        else:
            self.send_error(404)


def main():
    global latest_jpg, latest_stamp, frame_count, info
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8091)
    ap.add_argument("--topic", default="/panorama/image")
    ap.add_argument("--encode-hz", type=float, default=10.0)
    ap.add_argument("--jpeg-quality", type=int, default=80)
    ap.add_argument("--max-width", type=int, default=1280)
    args = ap.parse_args()

    rospy.init_node("seeker_panorama_preview", anonymous=True)
    info["topic"] = args.topic
    last_encode = [0.0]
    period = 1.0 / max(args.encode_hz, 1.0)

    def cb(msg):
        nonlocal last_encode
        now = time.time()
        if now - last_encode[0] < period:
            return
        last_encode[0] = now
        try:
            img = decode_image(msg)
            if img is None:
                return
            if args.max_width and img.shape[1] > args.max_width:
                scale = float(args.max_width) / img.shape[1]
                img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            ok, enc = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality])
            if not ok:
                return
            with lock:
                global latest_jpg, latest_stamp, frame_count
                latest_jpg = enc.tobytes()
                latest_stamp = rospy.get_time()
                frame_count += 1
                frame_times.append(now)
                info["width"] = img.shape[1]
                info["height"] = img.shape[0]
                info["encoding"] = msg.encoding
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "panorama decode failed: %s", exc)

    rospy.Subscriber(args.topic, Image, cb, queue_size=1, buff_size=2 ** 24)
    print("subscribed %s" % args.topic, flush=True)
    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print("panorama preview on http://0.0.0.0:%d/" % args.port, flush=True)
    rospy.spin()


if __name__ == "__main__":
    main()
