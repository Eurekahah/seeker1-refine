#!/usr/bin/env python3
"""Compose a live dashboard PNG from the Seeker1 preview server."""
import io
import json
import sys
import time

import requests
from PIL import Image, ImageDraw, ImageFont

BASE = "http://127.0.0.1:8092"
OUT = r"F:\Code\Seeker1\calib\dashboard_live.png"


def font(size, bold=False):
    candidates = [
        r"C:\Windows\Fonts\msyhbd.ttc" if bold else r"C:\Windows\Fonts\msyh.ttc",
        r"C:\Windows\Fonts\simhei.ttf",
        r"C:\Windows\Fonts\arialbd.ttf" if bold else r"C:\Windows\Fonts\arial.ttf",
    ]
    for path in candidates:
        try:
            return ImageFont.truetype(path, size)
        except Exception:
            pass
    return ImageFont.load_default()


def fetch(url):
    r = requests.get(url, timeout=5)
    r.raise_for_status()
    return r.content


def main():
    status = json.loads(fetch(BASE + "/status.json?t=%f" % time.time()))
    pano = Image.open(io.BytesIO(fetch(BASE + "/snapshot.jpg?t=%f" % time.time()))).convert("RGB")
    cams = []
    for i in range(4):
        img = Image.open(io.BytesIO(fetch(BASE + "/snapshot.jpg?id=cam%d&t=%f" % (i, time.time())))).convert("RGB")
        cams.append(img)

    W, H = 1500, 1470
    bg = (12, 14, 18)
    card = (25, 29, 36)
    fg = (232, 236, 242)
    muted = (150, 160, 175)
    ok = (60, 220, 130)
    accent = (80, 170, 255)
    canvas = Image.new("RGB", (W, H), bg)
    d = ImageDraw.Draw(canvas)

    f_title = font(30, True)
    f_body = font(20)
    f_small = font(17)
    f_mono = font(19)

    # header
    d.rectangle([0, 0, W, 62], fill=card)
    d.text((24, 15), "Seeker1  实时全景拼接", font=f_title, fill=fg)
    state = "● 实时" if status["pano"]["has_frame"] else "● 等待全景帧"
    d.text((1240, 22), state, font=f_body, fill=ok if status["pano"]["has_frame"] else (255, 190, 60))

    def panel(x, y, w, h, title, stats_lines):
        d.rectangle([x, y, x + w, y + h], fill=card)
        d.text((x + 14, y + 8), title, font=f_body, fill=fg)
        tx = x + 14
        ty = y + 38
        for line in stats_lines:
            d.text((tx, ty), line, font=f_small, fill=muted)
            ty += 22

    # panorama
    px, py, pw = 24, 78, 1452
    ph = int(pw * pano.height / pano.width)
    panel(px, py, pw, ph + 76, "全景  /panorama/image", [])
    canvas.paste(pano.resize((pw, ph), Image.LANCZOS), (px, py + 66))
    pa = status["pano"]["age_ms"]
    pstats = "960x240 · %.1f fps · 延迟 last %.0f ms / avg %.0f ms · seq %d" % (
        status["pano"]["fps"], pa["last"] or 0, pa["avg"] or 0, status["pano"]["seq"])
    d.text((px + 14, py + 36), pstats, font=f_small, fill=accent)

    # cameras 2x2
    names = ["left", "right", "bright", "bleft"]
    gap = 18
    tile_w = (W - 24 * 2 - gap) // 2
    tile_h = int(tile_w * cams[0].height / cams[0].width)
    y0 = py + ph + 96
    for i, name in enumerate(names):
        r = status["raw"][name]
        x = 24 + (i % 2) * (tile_w + gap)
        y = y0 + (i // 2) * (tile_h + 62 + gap)
        panel(x, y, tile_w, tile_h + 56, "原始 cam%d  %s" % (i, name), [])
        stats = "%.1f fps · 延迟 last %.0f ms / avg %.0f ms · %dx%d" % (
            r["fps"], r["age_ms"]["last"] or 0, r["age_ms"]["avg"] or 0, r["width"], r["height"])
        d.text((x + 14, y + 30), stats, font=f_small, fill=muted)
        canvas.paste(cams[i].resize((tile_w, tile_h), Image.LANCZOS), (x, y + 54))

    # footer / health
    fy = y0 + 2 * (tile_h + 62 + gap) + 8
    mem = status["mem"]
    pipe = status["pipeline"]
    rss = status["nodelet_rss_mb"]
    foot = "节点 RSS %.1f MB   ·   板端可用内存 %.0f MB   ·   处理滞后 avg %.0f ms   ·   四路时间差 avg %.0f ms   ·   %s" % (
        rss, mem.get("MemAvailable", 0), abs(pipe.get("avg") or 0),
        status["raw_spread_ms"].get("avg") or 0, time.strftime("%Y-%m-%d %H:%M:%S"))
    d.rectangle([0, H - 56, W, H], fill=card)
    d.text((24, H - 40), foot, font=f_mono, fill=fg)

    canvas.save(OUT, quality=95)
    print(OUT)
    print(foot)
    return 0


if __name__ == "__main__":
    sys.exit(main())
