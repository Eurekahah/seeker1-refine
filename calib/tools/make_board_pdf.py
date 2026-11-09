#!/usr/bin/env python3
"""Wrap a 1:1 ChArUco PNG into an exact-size A2/A3/A4 PDF."""
import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

PAPERS_MM = {
    "a2": (594.0, 420.0),
    "a3": (420.0, 297.0),
    "a4": (210.0, 297.0),
}


def mm_to_px(mm, dpi):
    return int(round(mm / 25.4 * dpi))


def load_font(size):
    for path in (r"C:\Windows\Fonts\arial.ttf", r"C:\Windows\Fonts\segoeui.ttf"):
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            pass
    return ImageFont.load_default()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--png", required=True)
    ap.add_argument("--spec", required=True)
    ap.add_argument("--paper", required=True, choices=sorted(PAPERS_MM))
    ap.add_argument("--dpi", type=float, default=300.0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default="Seeker1 4-camera fisheye calibration board")
    args = ap.parse_args()

    spec = json.loads(Path(args.spec).read_text(encoding="utf-8"))
    board = Image.open(args.png).convert("RGB")
    paper_w_mm, paper_h_mm = PAPERS_MM[args.paper]
    paper_w = mm_to_px(paper_w_mm, args.dpi)
    paper_h = mm_to_px(paper_h_mm, args.dpi)
    page = Image.new("RGB", (paper_w, paper_h), "white")

    board_w_mm = spec["physical_size_m"][0] * 1000.0
    board_h_mm = spec["physical_size_m"][1] * 1000.0
    if board_w_mm > paper_w_mm or board_h_mm > paper_h_mm:
        raise SystemExit("board does not fit on selected paper")
    x = (paper_w - board.width) // 2
    y = (paper_h - board.height) // 2
    page.paste(board, (x, y))

    draw = ImageDraw.Draw(page)
    font = load_font(mm_to_px(3.2, args.dpi))
    small = load_font(mm_to_px(2.4, args.dpi))
    ink = (0, 0, 0)

    # Top label (only where there is room).
    top_mm = (paper_h_mm - board_h_mm) / 2.0
    if top_mm >= 18:
        label = "%s  |  %dx%d squares  |  %.1f mm square  |  print at 100%%" % (
            "ChArUco DICT_4X4_50", spec["squares_x"], spec["squares_y"],
            spec["square_length_m"] * 1000.0,
        )
        tw = draw.textlength(label, font=font)
        draw.text(((paper_w - tw) / 2, mm_to_px(7, args.dpi)), label, font=font, fill=ink)
        note = "Do not use Fit to page / Scale to fit. Verify the 100 mm bar with a ruler."
        tw = draw.textlength(note, font=small)
        draw.text(((paper_w - tw) / 2, mm_to_px(13, args.dpi)), note, font=small, fill=ink)

    # 100 mm verification bar in the bottom margin.
    bar_y = paper_h - mm_to_px(6, args.dpi)
    bar_x0 = mm_to_px(10, args.dpi)
    bar_x1 = bar_x0 + mm_to_px(100, args.dpi)
    draw.line([(bar_x0, bar_y), (bar_x1, bar_y)], fill=ink, width=max(2, mm_to_px(0.5, args.dpi)))
    tick = mm_to_px(1.5, args.dpi)
    draw.line([(bar_x0, bar_y - tick), (bar_x0, bar_y + tick)], fill=ink, width=max(2, mm_to_px(0.5, args.dpi)))
    draw.line([(bar_x1, bar_y - tick), (bar_x1, bar_y + tick)], fill=ink, width=max(2, mm_to_px(0.5, args.dpi)))
    draw.text((bar_x1 + mm_to_px(3, args.dpi), bar_y - mm_to_px(1.5, args.dpi)),
              "100 mm", font=small, fill=ink)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    page.save(out, "PDF", resolution=args.dpi)
    print("wrote", out, "page_mm", paper_w_mm, "x", paper_h_mm,
          "board_mm", round(board_w_mm, 2), "x", round(board_h_mm, 2))


if __name__ == "__main__":
    main()
