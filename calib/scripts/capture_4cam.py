#!/usr/bin/env python3
"""Capture synchronized 4-camera compressed images for fisheye calibration.

Runs on the Seeker board. Saves the JPEG payload of CompressedImage messages,
so no decode/encode is needed on the ARM CPU.
"""
import argparse
import json
import os
import time

import message_filters
import rospy
from sensor_msgs.msg import CompressedImage

CAMERAS = ("left", "right", "bright", "bleft")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/data/local/tmp/calib_data")
    ap.add_argument("--sets", type=int, default=120)
    ap.add_argument("--hz", type=float, default=1.0)
    ap.add_argument("--slop", type=float, default=0.04)
    ap.add_argument("--queue", type=int, default=30)
    args = ap.parse_args()

    rospy.init_node("seeker_calib_capture", anonymous=True)
    out_dir = os.path.abspath(args.out)
    for cam in CAMERAS:
        os.makedirs(os.path.join(out_dir, cam), exist_ok=True)
    meta_path = os.path.join(out_dir, "sessions.jsonl")
    meta = open(meta_path, "a")
    session = {
        "started": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "sets_target": args.sets,
        "hz": args.hz,
        "slop": args.slop,
        "cameras": list(CAMERAS),
    }

    subs = []
    for cam in CAMERAS:
        topic = "/fisheye/%s/image_raw/compressed" % cam
        subs.append(message_filters.Subscriber(topic, CompressedImage))
        rospy.loginfo("subscribed %s", topic)

    sync = message_filters.ApproximateTimeSynchronizer(
        subs, queue_size=args.queue, slop=args.slop
    )

    state = {"count": 0, "last": 0.0, "stamps": [], "closed": False}
    min_period = 1.0 / args.hz if args.hz > 0 else 0.0

    def close():
        if state["closed"]:
            return
        state["closed"] = True
        session["ended"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        session["sets_saved"] = state["count"]
        meta.write(json.dumps(session, ensure_ascii=False) + "\n")
        meta.flush()
        meta.close()

    def callback(*msgs):
        now = time.time()
        if min_period and now - state["last"] < min_period:
            return
        stamps = [m.header.stamp.to_sec() for m in msgs]
        spread = max(stamps) - min(stamps)
        if spread > args.slop:
            return
        state["last"] = now
        seq = state["count"]
        stamp_ns = int(round(stamps[0] * 1e9))
        for cam, msg in zip(CAMERAS, msgs):
            path = os.path.join(out_dir, cam, "%06d_%d.jpg" % (seq, stamp_ns))
            with open(path, "wb") as f:
                f.write(msg.data)
        record = {
            "seq": seq,
            "wall": now,
            "stamps": {cam: s for cam, s in zip(CAMERAS, stamps)},
            "spread": spread,
            "bytes": {cam: len(m.data) for cam, m in zip(CAMERAS, msgs)},
        }
        meta.write(json.dumps(record) + "\n")
        meta.flush()
        state["count"] += 1
        print("SAVED %d  spread=%.4fs  bytes=%s" % (
            seq, spread, ",".join(str(len(m.data)) for m in msgs)), flush=True)
        if state["count"] >= args.sets:
            rospy.loginfo("finished %d sets", state["count"])
            close()
            rospy.signal_shutdown("capture complete")

    sync.registerCallback(callback)
    rospy.on_shutdown(close)
    print("capture started: out=%s sets=%d hz=%.2f slop=%.3f" % (
        out_dir, args.sets, args.hz, args.slop), flush=True)
    rospy.spin()


if __name__ == "__main__":
    main()
