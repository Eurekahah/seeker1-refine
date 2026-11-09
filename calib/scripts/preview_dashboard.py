#!/usr/bin/env python3
"""Seeker1 live dashboard: panorama, four raw views, latency and RSS."""
import argparse
import json
import os
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rospy
from sensor_msgs.msg import CompressedImage

RAW_CAMS = [
    ("left", "/fisheye/left/image_raw/compressed"),
    ("right", "/fisheye/right/image_raw/compressed"),
    ("bright", "/fisheye/bright/image_raw/compressed"),
    ("bleft", "/fisheye/bleft/image_raw/compressed"),
]
PANO_TOPIC = "/panorama/image/compressed"


class Feed:
    def __init__(self, name, topic):
        self.name = name
        self.topic = topic
        self.lock = threading.Lock()
        self.jpg = None
        self.seq = 0
        self.frames = 0
        self.stamp = 0.0
        self.width = 0
        self.height = 0
        self.ages = deque(maxlen=180)
        self.times = deque(maxlen=400)
        self.size_probe = 0

    def update(self, msg):
        now = rospy.get_time()
        stamp = msg.header.stamp.to_sec()
        age = (now - stamp) * 1000.0 if stamp > 0 else -1.0
        data = bytes(msg.data)
        with self.lock:
            self.jpg = data
            self.seq += 1
            self.frames += 1
            self.stamp = stamp
            if age >= 0:
                self.ages.append(age)
            self.times.append(now)
            self.size_probe += 1
            if self.width == 0 or self.size_probe % 120 == 0:
                self._probe_size(data)

    def _probe_size(self, data):
        try:
            import cv2
            import numpy as np
            img = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
            if img is not None:
                self.height, self.width = img.shape[0], img.shape[1]
        except Exception:
            pass

    def snapshot(self):
        with self.lock:
            return self.jpg, self.seq

    def stats(self):
        now = rospy.get_time()
        with self.lock:
            ages = list(self.ages)
            times = list(self.times)
            jpg = self.jpg is not None
            width, height = self.width, self.height
            frames = self.frames
            stamp = self.stamp
            seq = self.seq
        recent = [t for t in times if now - t <= 2.0]
        return {
            "topic": self.topic,
            "has_frame": jpg,
            "seq": seq,
            "frames": frames,
            "fps": round(len(recent) / 2.0, 2),
            "width": width,
            "height": height,
            "age_ms": summarize(ages),
            "stamp": round(stamp, 3),
        }


def summarize(values):
    if not values:
        return {"last": None, "avg": None, "min": None, "max": None}
    return {
        "last": round(values[-1], 1),
        "avg": round(sum(values) / len(values), 1),
        "min": round(min(values), 1),
        "max": round(max(values), 1),
    }


feeds = {}
pano = None
pipeline = deque(maxlen=180)
raw_spread = deque(maxlen=180)
server_lock = threading.Lock()


def find_nodelet_rss():
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            with open("/proc/%s/cmdline" % name, "rb") as handle:
                cmd = handle.read().replace(b"\0", b" ").decode("utf-8", "ignore")
            if "nodelet" not in cmd or "pano_manager" not in cmd:
                continue
            with open("/proc/%s/status" % name) as handle:
                for line in handle:
                    if line.startswith("VmRSS:"):
                        return int(name), round(int(line.split()[1]) / 1024.0, 1)
        except Exception:
            continue
    return None, None


def board_mem():
    info = {}
    try:
        with open("/proc/meminfo") as handle:
            for line in handle:
                key, value = line.split(":", 1)
                if key in ("MemTotal", "MemAvailable", "MemFree"):
                    info[key] = round(int(value.split()[0]) / 1024.0, 1)
        info["load1"] = round(os.getloadavg()[0], 2)
    except Exception:
        pass
    return info


def pano_callback(msg):
    now = rospy.get_time()
    stamp = msg.header.stamp.to_sec()
    age = (now - stamp) * 1000.0 if stamp > 0 else -1.0
    data = bytes(msg.data)
    with pano.lock:
        pano.jpg = data
        pano.seq += 1
        pano.frames += 1
        pano.stamp = stamp
        if age >= 0:
            pano.ages.append(age)
        pano.times.append(now)
        if pano.width == 0:
            pano._probe_size(data)

    with server_lock:
        raw_stamps = [feed.stamp for feed in feeds.values() if feed.stamp > 0]
        if raw_stamps and stamp > 0:
            pipeline.append((stamp - max(raw_stamps)) * 1000.0)
            raw_spread.append((max(raw_stamps) - min(raw_stamps)) * 1000.0)


PAGE = r'''<!doctype html><html><head><meta charset="utf-8">
<title>Seeker1 实时全景</title>
<style>
:root{--bg:#0b0f14;--card:#151b23;--line:#26303b;--fg:#e6edf3;--dim:#8b98a5;--ok:#3fb950;--warn:#d29922;--bad:#f85149}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.45 "Segoe UI",Arial,sans-serif}
.wrap{max-width:1680px;margin:0 auto;padding:14px}
h1{font-size:18px;margin:0 0 10px;font-weight:600}h1 span{color:var(--dim);font-weight:400;margin-left:10px}
.cards{display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:10px;margin-bottom:12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:10px 12px}
.card .k{color:var(--dim);font-size:12px}.card .v{font-size:22px;font-weight:600;margin-top:2px}.card .v small{font-size:12px;color:var(--dim);font-weight:400}
.pano{background:#000;border:1px solid var(--line);border-radius:8px;overflow:hidden;margin-bottom:12px}
.pano img{display:block;width:100%;max-height:44vh;object-fit:contain;background:#000}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
figure{margin:0;background:#000;border:1px solid var(--line);border-radius:8px;overflow:hidden}
figure img{display:block;width:100%;height:28vh;object-fit:contain;background:#000}
figcaption{padding:6px 10px;font-size:12px;color:var(--dim);border-top:1px solid var(--line);background:var(--card)}
.table{margin-top:12px;background:var(--card);border:1px solid var(--line);border-radius:8px;padding:10px 12px}
table{width:100%;border-collapse:collapse;font-size:12px}th,td{text-align:right;padding:5px 6px;border-bottom:1px solid var(--line)}
th:first-child,td:first-child{text-align:left}th{color:var(--dim);font-weight:500}
.ok{color:var(--ok)}.warn{color:var(--warn)}.bad{color:var(--bad)}
.note{color:var(--dim);font-size:12px;margin-top:8px}
@media(max-width:900px){.cards{grid-template-columns:1fr 1fr}.grid{grid-template-columns:1fr}}
</style></head><body><div class="wrap">
<h1>Seeker1 实时全景拼接<span id="state">连接中…</span></h1>
<div class="cards">
<div class="card"><div class="k">全景延迟 (采样→收到)</div><div class="v"><span id="panoAge">--</span><small> ms</small></div></div>
<div class="card"><div class="k">处理滞后 (相对最新原始帧)</div><div class="v"><span id="pipe">--</span><small> ms</small></div></div>
<div class="card"><div class="k">全景刷新率</div><div class="v"><span id="panoFps">--</span><small> fps</small></div></div>
<div class="card"><div class="k">全景节点内存</div><div class="v"><span id="rss">--</span><small> MB</small></div></div>
<div class="card"><div class="k">板端可用内存</div><div class="v"><span id="mem">--</span><small> MB</small></div></div>
</div>
<div class="pano"><img src="/pano.mjpg" alt="panorama"></div>
<div class="grid">
<figure><img src="/cam0.mjpg"><figcaption id="cap0">cam0 left</figcaption></figure>
<figure><img src="/cam1.mjpg"><figcaption id="cap1">cam1 right</figcaption></figure>
<figure><img src="/cam2.mjpg"><figcaption id="cap2">cam2 bright</figcaption></figure>
<figure><img src="/cam3.mjpg"><figcaption id="cap3">cam3 bleft</figcaption></figure>
</div>
<div class="table"><table><thead><tr><th>流</th><th>当前延迟</th><th>平均</th><th>最小</th><th>最大</th><th>帧率</th></tr></thead>
<tbody id="rows"></tbody></table>
<div class="note">延迟在板端用同一个 ROS 时钟计算：全景延迟 = 收到压缩帧时刻 − 帧 header 时间戳；处理滞后 = |全景时间戳 − 四路原始帧中最新的时间戳|。当前为 method 4 等距柱面全景（360°×90°），上下极窄黑边是四目水平环的天顶/天底盲区。原始四路为压缩话题直传，浏览器端不再解码。</div>
</div></div>
<script>
const $=id=>document.getElementById(id);
function cls(ms){if(ms==null)return'';if(ms<250)return'ok';if(ms<500)return'warn';return'bad'}
function set(id,v,unit){$(id).textContent=(v==null?'--':v)+(unit||'')}
function row(name,s){const a=s.age_ms;
 return `<tr><td>${name}</td><td class="${cls(a.last)}">${a.last==null?'--':a.last}</td><td>${a.avg==null?'--':a.avg}</td><td>${a.min==null?'--':a.min}</td><td>${a.max==null?'--':a.max}</td><td>${s.fps}</td></tr>`}
async function tick(){
 try{
  const r=await fetch('/status.json',{cache:'no-store'});const j=await r.json();
  set('panoAge',j.pano.age_ms.last); set('pipe',j.pipeline.last==null?null:Math.abs(j.pipeline.last));
  set('panoFps',j.pano.fps); set('rss',j.nodelet_rss_mb); set('mem',j.mem.MemAvailable);
  $('state').textContent=j.pano.has_frame?'● 实时':'● 等待全景帧';
  $('state').className=j.pano.has_frame?'ok':'warn';
  const names=['left','right','bright','bleft'];
  $('rows').innerHTML=row('全景',j.pano)+names.map(n=>row('原始 '+n,j.raw[n])).join('');
  names.forEach((n,i)=>{const s=j.raw[n];$('cap'+i).textContent=`cam${i} ${n} · ${s.fps}fps · ${s.age_ms.last==null?'--':s.age_ms.last}ms`});
 }catch(e){$('state').textContent='● 连接断开';$('state').className='bad'}
}
setInterval(tick,500);tick();
</script></body></html>'''


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def send_bytes(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path, _, query = self.path.partition("?")
        if path in ("/", "/index.html"):
            self.send_bytes(200, "text/html; charset=utf-8", PAGE.encode("utf-8"))
            return
        if path == "/status.json":
            with server_lock:
                pipeline_stats = summarize(list(pipeline))
                spread_stats = summarize(list(raw_spread))
            pid, rss = find_nodelet_rss()
            data = {
                "board_time": round(rospy.get_time(), 3),
                "pano": pano.stats(),
                "raw": {name: feed.stats() for name, feed in feeds.items()},
                "pipeline": pipeline_stats,
                "raw_spread_ms": spread_stats,
                "nodelet_pid": pid,
                "nodelet_rss_mb": rss,
                "mem": board_mem(),
            }
            self.send_bytes(200, "application/json", json.dumps(data).encode("utf-8"))
            return
        if path == "/snapshot.jpg":
            feed = pano
            for i, (name, _) in enumerate(RAW_CAMS):
                if ("id=cam%d" % i) in query:
                    feed = feeds[name]
            jpg, _ = feed.snapshot()
            if jpg is None:
                self.send_error(503, "no frame yet")
                return
            self.send_bytes(200, "image/jpeg", jpg)
            return
        for i, (name, _) in enumerate(RAW_CAMS):
            if path == "/cam%d.mjpg" % i:
                self.serve_mjpg(feeds[name], 10.0)
                return
        if path == "/pano.mjpg":
            self.serve_mjpg(pano, 12.0)
            return
        self.send_error(404)

    def serve_mjpg(self, feed, max_fps):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last_seq = -1
        last_send = 0.0
        interval = 1.0 / max(max_fps, 1.0)
        try:
            while True:
                jpg, seq = feed.snapshot()
                now = time.time()
                if jpg is None or seq == last_seq or (now - last_send) < interval:
                    time.sleep(0.01)
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ")
                self.wfile.write(str(len(jpg)).encode("ascii"))
                self.wfile.write(b"\r\n\r\n")
                self.wfile.write(jpg)
                self.wfile.write(b"\r\n")
                last_seq = seq
                last_send = now
        except Exception:
            return


def main():
    global pano
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8092)
    ap.add_argument("--pano-topic", default=PANO_TOPIC)
    args = ap.parse_args()

    rospy.init_node("seeker_dashboard", anonymous=True)
    pano = Feed("pano", args.pano_topic)
    for name, topic in RAW_CAMS:
        feed = Feed(name, topic)
        feeds[name] = feed
        rospy.Subscriber(topic, CompressedImage, lambda msg, f=feed: f.update(msg),
                         queue_size=1, buff_size=2 ** 24)
        print("subscribe %s" % topic, flush=True)
    rospy.Subscriber(args.pano_topic, CompressedImage, pano_callback,
                     queue_size=1, buff_size=2 ** 24)
    print("subscribe %s" % args.pano_topic, flush=True)

    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print("dashboard on http://0.0.0.0:%d/" % args.port, flush=True)
    rospy.spin()


if __name__ == "__main__":
    main()
