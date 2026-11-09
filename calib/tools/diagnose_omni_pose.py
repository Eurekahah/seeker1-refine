#!/usr/bin/env python3
"""Diagnose omni board-pose and pairwise-extrinsic errors on captured data."""
import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, "/tmp")
import calibrate_fisheye as cf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="/tmp/calib_data_assisted")
    ap.add_argument("--board-spec", default="/tmp/board_A4_26mm.json")
    ap.add_argument("--cams", default="left,right")
    args = ap.parse_args()

    cams = args.cams.split(",")
    spec = json.load(open(args.board_spec))
    board, dictionary = cf.build_board(spec)
    params = cf.detector_params()
    dets = {}
    size = None
    for cam in cams:
        dets[cam] = {}
        for path in sorted(glob.glob(os.path.join(args.data, cam, "*.jpg"))):
            m = cf.SEQ_RE.match(os.path.basename(path))
            if not m:
                continue
            img, rec = cf.detect_one(path, board, dictionary, params, 8)
            if img is not None and size is None:
                size = (img.shape[1], img.shape[0])
            if rec is not None:
                dets[cam][int(m.group(1))] = rec
        print(cam, "detected", len(dets[cam]))

    cals = {}
    for cam in cams:
        recs = [dets[cam][s] for s in sorted(dets[cam])]
        recs = cf.dedupe(recs, size, 100, 0.97)
        obj = [r["obj"] for r in recs]
        img = [r["img"].reshape(-1, 1, 2) for r in recs]
        cals[cam] = cf.run_omni(obj, img, size)
        print(cam, "omni", cals[cam]["score"], cals[cam]["K"].diagonal())

    for cam in cams:
        errs = []
        bad = 0
        for seq, a in dets[cam].items():
            pose = cf.pose_from_charuco(a["obj"], a["img"], cals[cam])
            if pose is None:
                bad += 1
                continue
            R, t = pose
            rv, _ = cv2.Rodrigues(R)
            pred = cf.project_points_with_cal(a["obj"], rv, t.reshape(3, 1), cals[cam])
            errs.append(float(np.sqrt(np.mean(np.sum((pred - a["img"]) ** 2, axis=1)))))
        errs = np.asarray(errs)
        print(cam, "pose px median=%.3f p90=%.3f max=%.3f bad=%d" % (
            np.median(errs), np.percentile(errs, 90), np.max(errs), bad))

    seqs = sorted(set(dets[cams[0]]) & set(dets[cams[1]]))
    rows = []
    for seq in seqs:
        a, b = dets[cams[0]][seq], dets[cams[1]][seq]
        common = np.intersect1d(a["ids"], b["ids"])
        if len(common) < 8:
            continue
        ia = {int(v): k for k, v in enumerate(a["ids"])}
        ib = {int(v): k for k, v in enumerate(b["ids"])}
        o = a["obj"][[ia[int(v)] for v in common]]
        pa = a["img"][[ia[int(v)] for v in common]]
        pb = b["img"][[ib[int(v)] for v in common]]
        pose_a = cf.pose_from_charuco(o, pa, cals[cams[0]])
        pose_b = cf.pose_from_charuco(o, pb, cals[cams[1]])
        if pose_a is None or pose_b is None:
            continue
        Ra, ta = pose_a
        Rb, tb = pose_b
        R21 = Rb @ Ra.T
        T21 = tb - R21 @ ta
        pred_b = cf.project_points_with_cal(o, cv2.Rodrigues(R21 @ Ra)[0],
                                            (R21 @ ta.reshape(3, 1) + T21.reshape(3, 1)), cals[cams[1]])
        err = float(np.sqrt(np.mean(np.sum((pred_b - pb) ** 2, axis=1))))
        rows.append((seq, err, cf.rotation_deg(R21), np.linalg.norm(T21)))
    rows.sort(key=lambda r: r[1])
    print("pair common", len(rows))
    print("best:")
    for r in rows[:10]:
        print("  seq=%d err=%.3f rot=%.2f |T|=%.1fmm" % (r[0], r[1], r[2], r[3] * 1000))
    print("median err=%.3f px" % np.median([r[1] for r in rows]))


if __name__ == "__main__":
    main()
