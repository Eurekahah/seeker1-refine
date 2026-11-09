#!/usr/bin/env python3
"""Assisted 4-camera capture: save only synchronized sets where the ChArUco
board is actually detected, and track per-camera / per-pair coverage.

Runs on the Seeker board (OpenCV 4.2, Python 3.8).
"""
import argparse
import json
import os
import time

import cv2
import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage

CAMERAS = ("left", "right", "bright", "bleft")
PAIRS = (("left", "right"), ("right", "bright"), ("bright", "bleft"))


def load_board(spec_path):
    spec = json.load(open(spec_path))
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec["dictionary"]))
    board = cv2.aruco.CharucoBoard_create(
        spec["squares_x"], spec["squares_y"],
        spec["square_length_m"], spec["marker_length_m"], dictionary)
    return spec, dictionary, board


def detect(gray, dictionary, board, min_markers):
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    if ids is None or len(ids) < min_markers:
        return 0, None
    n, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, gray, board)
    if ch_ids is None or n < 4:
        return int(n or 0), None
    pts = np.asarray(ch_corners, dtype=np.float32).reshape(-1, 2)
    return int(n), pts


def signature(pts, width, height):
    if pts is None or len(pts) < 4:
        return None
    center = pts.mean(axis=0) / np.array([width, height], dtype=np.float32)
    spread = float(np.mean(np.linalg.norm(pts - pts.mean(axis=0), axis=1))) / width
    return center, spread


def sig_changed(prev, cur, min_shift, min_spread):
    if prev is None or cur is None:
        return True
    dc = float(np.linalg.norm(cur[0] - prev[0]))
    ds = abs(cur[1] - prev[1]) / max(prev[1], 1e-6)
    return dc > min_shift or ds > min_spread


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/data/local/tmp/calib_data_assisted")
    ap.add_argument("--board-spec", default="/data/local/tmp/board_A3_50mm.json")
    ap.add_argument("--process-hz", type=float, default=4.0)
    ap.add_argument("--min-corners", type=int, default=10)
    ap.add_argument("--min-markers", type=int, default=4)
    ap.add_argument("--min-save-period", type=float, default=0.6)
    ap.add_argument("--min-shift", type=float, default=0.025)
    ap.add_argument("--min-spread", type=float, default=0.10)
    ap.add_argument("--target-cam", type=int, default=30)
    ap.add_argument("--target-pair", type=int, default=12)
    ap.add_argument("--slop", type=float, default=0.04)
    ap.add_argument("--status-period", type=float, default=2.0)
    ap.add_argument("--max-seconds", type=float, default=0.0)
    ap.add_argument("--stop-file", default="/data/local/tmp/stop_capture")
    args = ap.parse_args()

    spec, dictionary, board = load_board(args.board_spec)
    rospy.init_node("seeker_calib_assisted", anonymous=True)
    out_dir = os.path.abspath(args.out)
    for cam in CAMERAS:
        os.makedirs(os.path.join(out_dir, cam), exist_ok=True)
    meta = open(os.path.join(out_dir, "sessions.jsonl"), "a")

    subs = [message_filters.Subscriber(
        "/fisheye/%s/image_raw/compressed" % cam, CompressedImage) for cam in CAMERAS]
    sync = message_filters.ApproximateTimeSynchronizer(subs, queue_size=5, slop=args.slop)

    state = {
        "count": 0, "last_proc": 0.0, "last_save": 0.0, "last_status": 0.0,
        "last_sig": None, "started": time.time(), "cam_views": {c: 0 for c in CAMERAS},
        "pair_views": {"%s-%s" % p: 0 for p in PAIRS}, "closed": False,
        "cur": {c: 0 for c in CAMERAS},
    }

    def status(force=False):
        now = time.time()
        if not force and now - state["last_status"] < args.status_period:
            return
        state["last_status"] = now
        cams = " ".join("%s:%d/%d" % (c, state["cam_views"][c], args.target_cam) for c in CAMERAS)
        pairs = " ".join("%s:%d/%d" % (k, v, args.target_pair)
                         for k, v in state["pair_views"].items())
        print("PROG saved=%d %s %s | now %s" % (
            state["count"], cams, pairs,
            " ".join("%s:%d" % (c, state["cur"][c]) for c in CAMERAS)), flush=True)

    def close():
        if state["closed"]:
            return
        state["closed"] = True
        rec = {
            "started": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(state["started"])),
            "ended": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "sets_saved": state["count"],
            "cam_views": state["cam_views"],
            "pair_views": state["pair_views"],
            "board_spec": args.board_spec,
        }
        meta.write(json.dumps(rec) + "\n")
        meta.flush()
        meta.close()
        print("DONE saved=%d %s" % (state["count"], json.dumps(rec)), flush=True)

    def targets_met():
        return (all(state["cam_views"][c] >= args.target_cam for c in CAMERAS)
                and all(v >= args.target_pair for v in state["pair_views"].values()))

    def callback(*msgs):
        now = time.time()
        if args.max_seconds and now - state["started"] > args.max_seconds:
            close()
            rospy.signal_shutdown("time limit")
            return
        if os.path.exists(args.stop_file):
            close()
            rospy.signal_shutdown("stop file")
            return
        period = 1.0 / args.process_hz if args.process_hz > 0 else 0.0
        if period and now - state["last_proc"] < period:
            return
        state["last_proc"] = now

        stamps = [m.header.stamp.to_sec() for m in msgs]
        spread = max(stamps) - min(stamps)
        if spread > args.slop:
            return

        counts, sigs = {}, {}
        for cam, msg in zip(CAMERAS, msgs):
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            gray = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
            if gray is None:
                counts[cam], sigs[cam] = 0, None
                continue
            h, w = gray.shape[:2]
            small = cv2.resize(gray, (w // 2, h // 2), interpolation=cv2.INTER_AREA)
            n, pts = detect(small, dictionary, board, args.min_markers)
            counts[cam] = n
            sigs[cam] = signature(pts, w // 2, h // 2)
        state["cur"] = counts
        status()

        best = max(CAMERAS, key=lambda c: counts[c])
        if counts[best] < args.min_corners:
            return
        if now - state["last_save"] < args.min_save_period:
            return
        if not sig_changed(state["last_sig"], sigs[best], args.min_shift, args.min_spread):
            return

        seq = state["count"]
        stamp_ns = int(round(stamps[0] * 1e9))
        for cam, msg in zip(CAMERAS, msgs):
            path = os.path.join(out_dir, cam, "%06d_%d.jpg" % (seq, stamp_ns))
            with open(path, "wb") as f:
                f.write(msg.data)
        good_cams = [c for c in CAMERAS if counts[c] >= args.min_corners]
        for cam in good_cams:
            state["cam_views"][cam] += 1
        for a, b in PAIRS:
            if counts[a] >= args.min_corners and counts[b] >= args.min_corners:
                state["pair_views"]["%s-%s" % (a, b)] += 1
        record = {
            "seq": seq, "wall": now, "spread": spread,
            "stamps": {c: s for c, s in zip(CAMERAS, stamps)},
            "corners": counts,
            "bytes": {c: len(m.data) for c, m in zip(CAMERAS, msgs)},
        }
        meta.write(json.dumps(record) + "\n")
        meta.flush()
        state["last_save"] = now
        state["last_sig"] = sigs[best]
        state["count"] += 1
        print("SAVED %d good=%s corners=%s" % (seq, ",".join(good_cams), counts), flush=True)
        if targets_met():
            close()
            rospy.signal_shutdown("targets reached")

    sync.registerCallback(callback)
    rospy.on_shutdown(close)
    print("assisted capture: board=%s out=%s targets cam=%d pair=%d" % (
        args.board_spec, out_dir, args.target_cam, args.target_pair), flush=True)
    rospy.spin()


if __name__ == "__main__":
    main()
