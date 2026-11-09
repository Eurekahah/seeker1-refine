#!/usr/bin/env python3
"""Calibrate four synchronized fisheye cameras from ChArUco images.

Outputs:
  kalibr_cam_chain.yaml       camera_model=omni / distortion_model=radtan
  kalibr_cam_chain_fisheye.yaml camera_model=pinhole / equidistant
  calibration_report.json

The omni+radtan layout matches image_undistort's ImageConcatNodelet:
  intrinsics=[fx,fy,cx,cy], distortion_coeffs=[xi,k1,k2,p1,p2]
The fisheye layout is useful for per-camera undistortion and validation.
"""
import argparse
import glob
import json
import math
import os
import re
import subprocess
import tempfile

import cv2
import numpy as np

CAMERAS_DEFAULT = ("left", "right", "bright", "bleft")
SEQ_RE = re.compile(r"^(\d+)_")


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--board-spec", required=True)
    ap.add_argument("--out-dir", default="/tmp/calib_out")
    ap.add_argument("--order", default=",".join(CAMERAS_DEFAULT))
    ap.add_argument("--min-corners", type=int, default=8)
    ap.add_argument("--min-pair-corners", type=int, default=8)
    ap.add_argument("--max-views", type=int, default=100)
    ap.add_argument("--dedupe-sim", type=float, default=0.97)
    ap.add_argument("--skip-fisheye", action="store_true",
                    help="skip the equidistant fisheye fit and only use omni")
    return ap.parse_args()


def build_board(spec):
    dict_id = getattr(cv2.aruco, spec["dictionary"])
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    board = cv2.aruco.CharucoBoard_create(
        int(spec["squares_x"]), int(spec["squares_y"]),
        float(spec["square_length_m"]), float(spec["marker_length_m"]),
        dictionary,
    )
    return board, dictionary


def detector_params():
    p = cv2.aruco.DetectorParameters_create()
    p.adaptiveThreshWinSizeMin = 5
    p.adaptiveThreshWinSizeMax = 35
    p.adaptiveThreshWinSizeStep = 6
    p.minMarkerPerimeterRate = 0.008
    p.maxMarkerPerimeterRate = 0.6
    p.polygonalApproxAccuracyRate = 0.05
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.cornerRefinementWinSize = 7
    p.cornerRefinementMaxIterations = 50
    p.cornerRefinementMinAccuracy = 0.01
    return p


def detect_one(path, board, dictionary, params, min_corners):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        return None
    corners, ids, _ = cv2.aruco.detectMarkers(
        img, dictionary, parameters=params
    )
    if ids is None or len(ids) < 4:
        return img, None
    ret, img_pts, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, img, board
    )
    if charuco_ids is None or len(charuco_ids) < min_corners:
        return img, None
    ids_flat = charuco_ids.reshape(-1).astype(int)
    pts = img_pts.reshape(-1, 2).astype(np.float64)
    obj = board.chessboardCorners[ids_flat].astype(np.float64)
    return img, {"ids": ids_flat, "img": pts, "obj": obj, "path": path}


def signature(pts, size):
    h, _, _ = np.histogram2d(
        pts[:, 0], pts[:, 1], bins=[10, 8],
        range=[[0, size[0]], [0, size[1]]],
    )
    h = h.reshape(-1)
    n = np.linalg.norm(h)
    return h / n if n > 0 else h


def dedupe(records, size, max_views, sim_threshold):
    accepted = []
    sigs = []
    for rec in records:
        sig = signature(rec["img"], size)
        if sigs and max(float(np.dot(sig, s)) for s in sigs) >= sim_threshold:
            continue
        accepted.append(rec)
        sigs.append(sig)
        if len(accepted) >= max_views:
            break
    return accepted


def initial_k(size):
    w, h = size
    f = 0.29 * w
    return np.array([[f, 0.0, (w - 1) / 2.0],
                     [0.0, f, (h - 1) / 2.0],
                     [0.0, 0.0, 1.0]], dtype=np.float64)


def fisheye_view_errors(obj, img, K, D, rvecs, tvecs):
    errs = []
    for o, im, rv, tv in zip(obj, img, rvecs, tvecs):
        proj, _ = cv2.fisheye.projectPoints(
            np.asarray(o, dtype=np.float64).reshape(1, -1, 3),
            rv.reshape(3, 1), tv.reshape(3, 1), K, D
        )
        proj = proj.reshape(-1, 2)
        im = np.asarray(im, dtype=np.float64).reshape(-1, 2)
        n = min(len(proj), len(im))
        errs.append(float(np.sqrt(np.mean(np.sum((proj[:n] - im[:n]) ** 2, axis=1)))))
    return np.array(errs)


def run_fisheye(obj, img, size):
    K = initial_k(size)
    D = np.zeros((4, 1), dtype=np.float64)
    criteria = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 100, 1e-7)
    flag_sets = [
        cv2.fisheye.CALIB_USE_INTRINSIC_GUESS | cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW,
        cv2.fisheye.CALIB_USE_INTRINSIC_GUESS | cv2.fisheye.CALIB_FIX_SKEW,
        cv2.fisheye.CALIB_USE_INTRINSIC_GUESS,
    ]
    obj3 = [np.asarray(o, dtype=np.float64).reshape(-1, 1, 3) for o in obj]
    img2 = [np.asarray(i, dtype=np.float64).reshape(-1, 1, 2) for i in img]
    best = None
    last_err = None
    for flags in flag_sets:
        try:
            rms, Kc, Dc, rvecs, tvecs = cv2.fisheye.calibrate(
                obj3, img2, size, K.copy(), D.copy(), flags=flags, criteria=criteria
            )
        except cv2.error as exc:
            last_err = str(exc).strip().splitlines()[-1]
            continue
        if not np.all(np.isfinite(Kc)) or Kc[0, 0] <= 0 or Kc[1, 1] <= 0:
            continue
        errs = fisheye_view_errors(obj, img, Kc, Dc, rvecs, tvecs)
        score = float(np.median(errs))
        if best is None or score < best["score"]:
            best = {"rms": float(rms), "score": score, "K": Kc, "D": Dc,
                    "rvecs": rvecs, "tvecs": tvecs, "errs": errs,
                    "flags": int(flags)}
    if best is None:
        raise RuntimeError("fisheye calibration failed: %s" % last_err)

    # One outlier-rejection pass.
    keep = best["errs"] <= max(3.0, 3.0 * np.median(best["errs"]))
    if keep.sum() >= max(8, int(0.7 * len(obj))) and keep.sum() < len(obj):
        obj2 = [o for o, k in zip(obj3, keep) if k]
        img3 = [i for i, k in zip(img2, keep) if k]
        try:
            rms, Kc, Dc, rvecs, tvecs = cv2.fisheye.calibrate(
                obj2, img3, size, best["K"].copy(), best["D"].copy(),
                flags=best["flags"], criteria=criteria
            )
            errs = fisheye_view_errors(obj2, img3, Kc, Dc, rvecs, tvecs)
            if float(np.median(errs)) <= best["score"] * 1.05:
                best = {"rms": float(rms), "score": float(np.median(errs)),
                        "K": Kc, "D": Dc, "rvecs": rvecs, "tvecs": tvecs,
                        "errs": errs, "flags": best["flags"],
                        "used": int(keep.sum()), "total": int(len(obj))}
        except cv2.error:
            pass
    best.setdefault("used", len(obj))
    best.setdefault("total", len(obj))
    return best


def omni_view_errors(obj, img, K, xi, D, rvecs, tvecs):
    errs = []
    for o, im, rv, tv in zip(obj, img, rvecs, tvecs):
        proj, _ = cv2.omnidir.projectPoints(
            np.asarray(o, dtype=np.float64).reshape(-1, 1, 3),
            rv.reshape(3, 1), tv.reshape(3, 1), K, float(np.asarray(xi).reshape(-1)[0]), D
        )
        proj = proj.reshape(-1, 2)
        im = np.asarray(im, dtype=np.float64).reshape(-1, 2)
        n = min(len(proj), len(im))
        errs.append(float(np.sqrt(np.mean(np.sum((proj[:n] - im[:n]) ** 2, axis=1)))))
    return np.array(errs)


def run_omni(obj, img, size, K_init=None):
    """Call the C++ helper; OpenCV 4.2 Python cannot pass CV_64FC3 arrays."""
    K0 = initial_k(size) if K_init is None else np.asarray(K_init, dtype=np.float64)
    fd, path = tempfile.mkstemp(prefix="omni_points_", suffix=".txt")
    try:
        with os.fdopen(fd, "w") as f:
            f.write("%d\n" % len(obj))
            for o, p in zip(obj, img):
                oo = np.asarray(o, dtype=np.float64).reshape(-1, 3)
                pp = np.asarray(p, dtype=np.float64).reshape(-1, 2)
                f.write("%d\n" % len(oo))
                for a, b in zip(oo, pp):
                    f.write("%.17g %.17g %.17g %.17g %.17g\n" % (
                        a[0], a[1], a[2], b[0], b[1]))
        cmd = ["omni_calib", path, str(size[0]), str(size[1]),
               "%.17g" % K0[0, 0], "%.17g" % K0[1, 1],
               "%.17g" % K0[0, 2], "%.17g" % K0[1, 2]]
        run = subprocess.run(cmd, capture_output=True, text=True)
        if run.returncode != 0:
            raise RuntimeError("omni_calib failed: %s" % (run.stderr or run.stdout).strip())
        d = json.loads(run.stdout)
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass
    Kc = np.asarray(d["K"], dtype=np.float64).reshape(3, 3)
    xi_c = float(d["xi"])
    Dc = np.asarray(d["D"], dtype=np.float64).reshape(4, 1)
    rvecs = [np.asarray(r, dtype=np.float64).reshape(3, 1) for r in d["rvecs"]]
    tvecs = [np.asarray(t, dtype=np.float64).reshape(3, 1) for t in d["tvecs"]]
    idx = d.get("idx") or list(range(len(obj)))
    if len(rvecs) != len(idx):
        raise RuntimeError("omni_calib returned %d poses for %d indices" % (len(rvecs), len(idx)))
    used_obj = [obj[i] for i in idx]
    used_img = [img[i] for i in idx]
    errs = omni_view_errors(used_obj, used_img, Kc, xi_c, Dc, rvecs, tvecs)
    return {"rms": float(d["rms"]), "score": float(np.median(errs)), "K": Kc,
            "xi": xi_c, "D": Dc, "rvecs": rvecs, "tvecs": tvecs,
            "errs": errs, "used": len(idx), "total": int(len(obj))}


def make_pair_data(det_a, det_b, seqs, min_corners):
    obj, ip1, ip2, used = [], [], [], []
    for seq in seqs:
        a, b = det_a.get(seq), det_b.get(seq)
        if a is None or b is None:
            continue
        common = np.intersect1d(a["ids"], b["ids"])
        if len(common) < min_corners:
            continue
        ia = {int(v): k for k, v in enumerate(a["ids"])}
        ib = {int(v): k for k, v in enumerate(b["ids"])}
        o = a["obj"][[ia[int(v)] for v in common]].astype(np.float64)
        p1 = a["img"][[ia[int(v)] for v in common]].astype(np.float64)
        p2 = b["img"][[ib[int(v)] for v in common]].astype(np.float64)
        obj.append(o.reshape(-1, 1, 3))
        ip1.append(p1.reshape(-1, 1, 2))
        ip2.append(p2.reshape(-1, 1, 2))
        used.append(seq)
    return obj, ip1, ip2, used


def run_stereo(obj, ip1, ip2, size, cal_a, cal_b):
    """Call the C++ helper; Python bindings fail with variable point counts."""
    fd, path = tempfile.mkstemp(prefix="stereo_points_", suffix=".txt")
    try:
        with os.fdopen(fd, "w") as f:
            f.write("%d\n" % len(obj))
            for o, p1, p2 in zip(obj, ip1, ip2):
                oo = np.asarray(o, dtype=np.float64).reshape(-1, 3)
                aa = np.asarray(p1, dtype=np.float64).reshape(-1, 2)
                bb = np.asarray(p2, dtype=np.float64).reshape(-1, 2)
                f.write("%d\n" % len(oo))
                for a, b, c in zip(oo, aa, bb):
                    f.write("%.17g %.17g %.17g %.17g %.17g %.17g %.17g\n" % (
                        a[0], a[1], a[2], b[0], b[1], c[0], c[1]))
        Ka, Da = np.asarray(cal_a["K"]), np.asarray(cal_a["D"]).reshape(-1)
        Kb, Db = np.asarray(cal_b["K"]), np.asarray(cal_b["D"]).reshape(-1)
        cmd = ["fisheye_stereo", path, str(size[0]), str(size[1]),
               "%.17g" % Ka[0, 0], "%.17g" % Ka[1, 1],
               "%.17g" % Ka[0, 2], "%.17g" % Ka[1, 2]]
        cmd += ["%.17g" % x for x in Da[:4]]
        cmd += ["%.17g" % Kb[0, 0], "%.17g" % Kb[1, 1],
                "%.17g" % Kb[0, 2], "%.17g" % Kb[1, 2]]
        cmd += ["%.17g" % x for x in Db[:4]]
        run = subprocess.run(cmd, capture_output=True, text=True)
        if run.returncode != 0:
            raise RuntimeError("fisheye_stereo failed: %s" % (run.stderr or run.stdout).strip())
        d = json.loads(run.stdout)
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass
    R = np.asarray(d["R"], dtype=np.float64).reshape(3, 3)
    T = np.asarray(d["T"], dtype=np.float64).reshape(3, 1)
    return {"rms": float(d["rms"]), "R": R, "T": T, "views": len(obj)}


def rotation_deg(R):
    rv, _ = cv2.Rodrigues(R)
    return float(np.degrees(np.linalg.norm(rv)))


def pinhole_points_from_cal(img, cal):
    """Convert image points to pinhole-normalized coordinates for solvePnP."""
    try:
        pts = np.asarray(img, dtype=np.float64).reshape(-1, 1, 2)
        if "xi" in cal:
            # OpenCV's omnidir.undistortPoints returns u = m / (1 + xi*|m|),
            # not the pinhole-normalized m expected by solvePnP.
            u = cv2.omnidir.undistortPoints(
                pts, cal["K"], cal["D"], np.array([cal["xi"]], dtype=np.float64),
                np.eye(3, dtype=np.float64)
            ).reshape(-1, 2)
            xi = float(cal["xi"])
            rho = np.linalg.norm(u, axis=1)
            den = 1.0 - rho * rho * xi * xi
            if np.any(np.abs(den) < 1e-8):
                return None
            s = (rho * rho * xi + np.sqrt(1.0 + rho * rho * (1.0 - xi * xi))) / den
            norm = u * (1.0 + xi * s[:, None])
        else:
            norm = cv2.fisheye.undistortPoints(pts, cal["K"], cal["D"])
        return np.asarray(norm, dtype=np.float64).reshape(-1, 1, 2)
    except (cv2.error, KeyError, ValueError):
        return None


def project_points_with_cal(obj, rvec, tvec, cal):
    """Project board points through the calibration model stored in cal."""
    obj = np.asarray(obj, dtype=np.float64).reshape(-1, 1, 3)
    rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
    tvec = np.asarray(tvec, dtype=np.float64).reshape(3, 1)
    if "xi" in cal:
        proj, _ = cv2.omnidir.projectPoints(
            obj, rvec, tvec, cal["K"], float(cal["xi"]), cal["D"]
        )
    else:
        proj, _ = cv2.fisheye.projectPoints(
            obj, rvec, tvec, cal["K"], cal["D"]
        )
    return proj.reshape(-1, 2)


def pose_from_charuco(obj, img, cal):
    """Board pose in camera frame from normalized calibration points."""
    try:
        norm = pinhole_points_from_cal(img, cal)
        if norm is None:
            return None
        ok, rvec, tvec = cv2.solvePnP(
            np.asarray(obj, dtype=np.float64).reshape(-1, 1, 3),
            norm, np.eye(3), np.zeros(4),
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not ok:
            return None
        rvec, tvec = cv2.solvePnPRefineLM(
            np.asarray(obj, dtype=np.float64).reshape(-1, 1, 3),
            norm, np.eye(3), np.zeros(4), rvec, tvec
        )
        R, _ = cv2.Rodrigues(rvec)
        return R, np.asarray(tvec, dtype=np.float64).reshape(3)
    except cv2.error:
        return None


def average_rotations(rots):
    ref = cv2.Rodrigues(rots[0])[0].reshape(3)
    rvs = []
    for R in rots:
        rv = cv2.Rodrigues(R)[0].reshape(3)
        if float(np.dot(rv, ref)) < 0:
            rv = -rv
        rvs.append(rv)
    rvs = np.asarray(rvs)
    med = np.median(rvs, axis=0)
    ang = np.linalg.norm(rvs - med, axis=1)
    keep = ang <= max(0.03, 3.0 * float(np.median(ang)))
    mean_rv = np.mean(rvs[keep], axis=0)
    R, _ = cv2.Rodrigues(mean_rv)
    return R, keep


def pose_reproj_rms(obj, img, R, t, cal):
    rvec, _ = cv2.Rodrigues(np.asarray(R, dtype=np.float64))
    pred = project_points_with_cal(obj, rvec, np.asarray(t).reshape(3, 1), cal)
    img = np.asarray(img, dtype=np.float64).reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum((pred - img) ** 2, axis=1))))


def pair_view_rms(estimate, R, T, cal_b):
    _, obj, pb, _, _, Ra, ta = estimate
    Rb = np.asarray(R, dtype=np.float64) @ Ra
    tb = np.asarray(R, dtype=np.float64) @ ta.reshape(3, 1) + np.asarray(T).reshape(3, 1)
    pred = project_points_with_cal(obj, cv2.Rodrigues(Rb)[0], tb, cal_b)
    return float(np.sqrt(np.mean(np.sum((pred - pb) ** 2, axis=1))))


def estimate_pair_extrinsics(det_a, det_b, seqs, cal_a, cal_b, min_corners):
    """Per-view board poses -> robust relative camera transform."""
    estimates = []
    for seq in seqs:
        a, b = det_a.get(seq), det_b.get(seq)
        if a is None or b is None:
            continue
        common = np.intersect1d(a["ids"], b["ids"])
        if len(common) < min_corners:
            continue
        ia = {int(v): k for k, v in enumerate(a["ids"])}
        ib = {int(v): k for k, v in enumerate(b["ids"])}
        o = a["obj"][[ia[int(v)] for v in common]].astype(np.float64)
        pa = a["img"][[ia[int(v)] for v in common]].astype(np.float64)
        pb = b["img"][[ib[int(v)] for v in common]].astype(np.float64)
        pose_a = pose_from_charuco(o, pa, cal_a)
        pose_b = pose_from_charuco(o, pb, cal_b)
        if pose_a is None or pose_b is None:
            continue
        Ra, ta = pose_a
        Rb, tb = pose_b
        if (pose_reproj_rms(o, pa, Ra, ta, cal_a) > 8.0 or
                pose_reproj_rms(o, pb, Rb, tb, cal_b) > 8.0):
            continue
        R21 = Rb @ Ra.T
        T21 = tb - R21 @ ta
        estimates.append((seq, o, pb, R21, T21, Ra, ta))
    if len(estimates) < 3:
        raise RuntimeError("pair has only %d usable board poses" % len(estimates))

    # Planar PnP can return a mirrored branch with low per-camera error. Pick
    # the relative transform supported by the most views, then refine only
    # that consensus set.
    best = None
    for cand in estimates:
        cand_errs = np.asarray([pair_view_rms(e, cand[3], cand[4], cal_b)
                                for e in estimates])
        inliers = cand_errs <= 8.0
        score = (int(inliers.sum()), -float(np.median(cand_errs[inliers])))
        if best is None or score > best[0]:
            best = (score, inliers)
    keep = best[1]
    if keep.sum() < 3:
        raise RuntimeError("pair has only %d consensus board poses" % int(keep.sum()))

    selected = [e for e, k in zip(estimates, keep) if k]
    R, _ = average_rotations([e[3] for e in selected])
    Ts = np.asarray([e[4] for e in selected])
    T = np.median(Ts, axis=0)
    R, T, refine_rms = refine_extrinsics(selected, cal_b, R, T.reshape(3, 1))

    errs_after = np.asarray([pair_view_rms(e, R, T, cal_b) for e in estimates])
    inliers_after = errs_after <= max(3.0, 3.0 * float(np.median(errs_after[keep])))
    if inliers_after.sum() >= 3 and inliers_after.sum() < keep.sum():
        selected = [e for e, k in zip(estimates, inliers_after) if k]
        R, _ = average_rotations([e[3] for e in selected])
        T = np.median(np.asarray([e[4] for e in selected]), axis=0)
        R, T, refine_rms = refine_extrinsics(selected, cal_b, R, T.reshape(3, 1))
        keep = inliers_after

    errs = []
    for e in estimates:
        errs.append(pair_view_rms(e, R, T, cal_b))
    errs = np.asarray(errs)
    return {"R": R, "T": T.reshape(3, 1), "rms": float(np.sqrt(np.mean(errs ** 2))),
            "refine_rms": refine_rms,
            "median": float(np.median(errs)), "views": len(estimates),
            "used": int(keep.sum()), "seqs": [e[0] for e in estimates],
            "inlier_seqs": [e[0] for e, k in zip(estimates, keep) if k],
            "inlier_rms": float(np.sqrt(np.mean(errs[keep] ** 2))),
            "errs": errs.tolist()}

def refine_extrinsics(estimates, cal_b, R0, T0, iters=30):
    """LM refinement of R,T minimizing camera-B reprojection error."""
    rv0, _ = cv2.Rodrigues(np.asarray(R0, dtype=np.float64))
    p = np.concatenate([rv0.reshape(3), np.asarray(T0, dtype=np.float64).reshape(3)])

    def residual(params):
        R, _ = cv2.Rodrigues(params[:3].reshape(3, 1))
        T = params[3:].reshape(3, 1)
        out = []
        for seq, o, pb, _, _, Ra, ta in estimates:
            Rb = R @ Ra
            tb = R @ ta.reshape(3, 1) + T
            rvb, _ = cv2.Rodrigues(Rb)
            proj = project_points_with_cal(o, rvb, tb, cal_b)
            out.append((proj - pb).reshape(-1))
        return np.concatenate(out)

    r = residual(p)
    cost = float(np.dot(r, r))
    lam = 1e-3
    eps_rot, eps_t = 1e-7, 1e-7
    for _ in range(iters):
        J = np.zeros((len(r), 6), dtype=np.float64)
        for j in range(6):
            dp = p.copy()
            step = eps_rot if j < 3 else eps_t
            dp[j] += step
            J[:, j] = (residual(dp) - r) / step
        A = J.T @ J
        g = J.T @ r
        improved = False
        for _try in range(10):
            try:
                delta = np.linalg.solve(A + lam * np.eye(6), -g)
            except np.linalg.LinAlgError:
                lam *= 10.0
                continue
            p_new = p + delta
            r_new = residual(p_new)
            cost_new = float(np.dot(r_new, r_new))
            if cost_new < cost:
                p, r, cost = p_new, r_new, cost_new
                lam = max(lam / 3.0, 1e-10)
                improved = True
                break
            lam *= 10.0
        if not improved or float(np.linalg.norm(delta)) < 1e-11:
            break
    R, _ = cv2.Rodrigues(p[:3].reshape(3, 1))
    T = p[3:].reshape(3, 1)
    return R, T, float(np.sqrt(np.mean(r ** 2)))

def even_indices(n, m):
    if m >= n:
        return list(range(n))
    idx = sorted(set(int(round(x)) for x in np.linspace(0, n - 1, m)))
    extra = [i for i in range(n) if i not in idx]
    while len(idx) < m and extra:
        idx.append(extra.pop(0))
    return sorted(idx)


def build_equal_pair_data(det_a, det_b, seqs, min_corners):
    rows = []
    for seq in seqs:
        a, b = det_a.get(seq), det_b.get(seq)
        if a is None or b is None:
            continue
        common = np.intersect1d(a["ids"], b["ids"])
        if len(common) >= min_corners:
            rows.append((seq, common, a, b))
    if len(rows) < 3:
        return None
    lengths = [len(r[1]) for r in rows]
    m = max(min_corners, int(np.percentile(lengths, 25)))
    rows = [r for r in rows if len(r[1]) >= m]
    if len(rows) < 3:
        return None
    obj, ip1, ip2, used = [], [], [], []
    for seq, common, a, b in rows:
        ia = {int(v): k for k, v in enumerate(a["ids"])}
        ib = {int(v): k for k, v in enumerate(b["ids"])}
        sel = [common[i] for i in even_indices(len(common), m)]
        o = a["obj"][[ia[int(v)] for v in sel]].astype(np.float64)
        p1 = a["img"][[ia[int(v)] for v in sel]].astype(np.float64)
        p2 = b["img"][[ib[int(v)] for v in sel]].astype(np.float64)
        obj.append(o.reshape(-1, 1, 3))
        ip1.append(p1.reshape(-1, 1, 2))
        ip2.append(p2.reshape(-1, 1, 2))
        used.append(seq)
    return obj, ip1, ip2, used, m


def pair_reproj_rms(det_a, det_b, seqs, cal_a, cal_b, R, T, min_corners):
    errs = []
    for seq in seqs:
        a, b = det_a.get(seq), det_b.get(seq)
        if a is None or b is None:
            continue
        common = np.intersect1d(a["ids"], b["ids"])
        if len(common) < min_corners:
            continue
        ia = {int(v): k for k, v in enumerate(a["ids"])}
        ib = {int(v): k for k, v in enumerate(b["ids"])}
        o = a["obj"][[ia[int(v)] for v in common]].astype(np.float64)
        pa = a["img"][[ia[int(v)] for v in common]].astype(np.float64)
        pb = b["img"][[ib[int(v)] for v in common]].astype(np.float64)
        pose_a = pose_from_charuco(o, pa, cal_a)
        if pose_a is None:
            continue
        Ra, ta = pose_a
        R = np.asarray(R, dtype=np.float64)
        T = np.asarray(T, dtype=np.float64).reshape(3, 1)
        Rb = R @ Ra
        tb = R @ ta.reshape(3, 1) + T
        rv_b, _ = cv2.Rodrigues(Rb)
        pred_b = project_points_with_cal(o, rv_b, tb.reshape(3, 1), cal_b)
        errs.append(float(np.sqrt(np.mean(np.sum((pred_b - pb) ** 2, axis=1)))))
        rv_a, _ = cv2.Rodrigues(Ra)
        pred_a = project_points_with_cal(o, rv_a, ta.reshape(3, 1), cal_a)
        errs.append(float(np.sqrt(np.mean(np.sum((pred_a - pa) ** 2, axis=1)))))
    return float(np.sqrt(np.mean(np.square(errs)))) if errs else float("inf")

def mat4(R, T):
    M = np.eye(4, dtype=np.float64)
    M[:3, :3] = R
    M[:3, 3] = np.asarray(T).reshape(3)
    return M


def fmt_num(x):
    s = "%.10g" % float(x)
    if "." not in s and "e" not in s and "E" not in s:
        s += ".0"
    return s


def fmt_vec(v):
    return "[" + ", ".join(fmt_num(x) for x in np.asarray(v).reshape(-1)) + "]"


def fmt_mat(M):
    rows = []
    for row in np.asarray(M):
        rows.append("    [" + ", ".join(fmt_num(x) for x in row) + "]")
    return "[\n" + ",\n".join(rows) + "\n  ]"


def write_yaml(path, cameras, cals, pair_T, model, resolution):
    lines = []
    for i, cam in enumerate(cameras):
        cal = cals[cam]
        lines.append("cam%d:" % i)
        lines.append("  camera_model: %s" % model)
        if model == "omni":
            lines.append("  distortion_model: radtan")
            lines.append("  intrinsics: %s" % fmt_vec(
                [cal["K"][0, 0], cal["K"][1, 1], cal["K"][0, 2], cal["K"][1, 2]]))
            lines.append("  distortion_coeffs: %s" % fmt_vec(
                [cal["xi"], cal["D"][0, 0], cal["D"][1, 0],
                 cal["D"][2, 0], cal["D"][3, 0]]))
        else:
            lines.append("  distortion_model: equidistant")
            lines.append("  intrinsics: %s" % fmt_vec(
                [cal["K"][0, 0], cal["K"][1, 1], cal["K"][0, 2], cal["K"][1, 2]]))
            lines.append("  distortion_coeffs: %s" % fmt_vec(cal["D"].reshape(-1)))
        lines.append("  resolution: [%d, %d]" % (resolution[0], resolution[1]))
        if i == 0:
            lines.append("  T_cn_cnm1: %s" % fmt_mat(np.eye(4)))
        else:
            prev = cameras[i - 1]
            lines.append("  T_cn_cnm1: %s" % fmt_mat(pair_T[(prev, cam)]))
        lines.append("  T_cam_imu: %s" % fmt_mat(np.eye(4)))
        lines.append("  rostopic: /fisheye/%s/image_raw" % cam)
        lines.append("  timeshift_cam_imu: 0.0")
        lines.append("")
    with open(path, "w") as f:
        f.write("\n".join(lines))
    return path


def main():
    args = parse_args()
    cameras = [c.strip() for c in args.order.split(",") if c.strip()]
    spec = json.load(open(args.board_spec))
    board, dictionary = build_board(spec)
    params = detector_params()
    os.makedirs(args.out_dir, exist_ok=True)
    debug_dir = os.path.join(args.out_dir, "debug")
    os.makedirs(debug_dir, exist_ok=True)

    detections = {}
    image_size = None
    for cam in cameras:
        files = sorted(glob.glob(os.path.join(args.data, cam, "*.jpg")))
        detections[cam] = {}
        for path in files:
            m = SEQ_RE.match(os.path.basename(path))
            if not m:
                continue
            seq = int(m.group(1))
            img, rec = detect_one(path, board, dictionary, params, args.min_corners)
            if img is not None and image_size is None:
                image_size = (img.shape[1], img.shape[0])
            if rec is not None:
                detections[cam][seq] = rec
        print("%-7s images=%d detected=%d" % (cam, len(files), len(detections[cam])))

    if image_size is None:
        raise SystemExit("no images detected")

    cals_fisheye = {}
    cals_omni = {}
    report = {"board_spec": spec, "image_size": list(image_size), "cameras": {}}
    for cam in cameras:
        recs = [detections[cam][s] for s in sorted(detections[cam])]
        recs = dedupe(recs, image_size, args.max_views, args.dedupe_sim)
        if len(recs) < 8:
            raise SystemExit("%s: only %d usable views; need at least 8" % (cam, len(recs)))
        obj = [r["obj"] for r in recs]
        img = [r["img"].reshape(-1, 1, 2) for r in recs]
        print("%-7s calibrating with %d diverse views" % (cam, len(recs)))
        fisheye = None
        if not args.skip_fisheye:
            try:
                fisheye = run_fisheye(obj, img, image_size)
                cals_fisheye[cam] = fisheye
            except Exception as exc:
                print("        fisheye skipped: %s" % exc)
        omni = run_omni(obj, img, image_size, fisheye["K"] if fisheye else None)
        cals_omni[cam] = omni
        report["cameras"][cam] = {
            "detected": len(detections[cam]),
            "used": len(recs),
            "omni": {"rms": omni["rms"], "median_reproj_px": omni["score"],
                     "xi": omni["xi"], "K": omni["K"].tolist(),
                     "D": omni["D"].reshape(-1).tolist()},
        }
        if fisheye is not None:
            report["cameras"][cam]["fisheye"] = {
                "rms": fisheye["rms"], "median_reproj_px": fisheye["score"],
                "used_after_outliers": fisheye["used"],
                "K": fisheye["K"].tolist(), "D": fisheye["D"].reshape(-1).tolist()}
            print("        fisheye rms=%.3f median=%.3f px | omni xi=%.3f rms=%.3f median=%.3f px" % (
                fisheye["rms"], fisheye["score"], omni["xi"], omni["rms"], omni["score"]))
        else:
            print("        omni xi=%.3f rms=%.3f median=%.3f px" % (
                omni["xi"], omni["rms"], omni["score"]))

        # Coverage image and a few annotated samples.
        cover = np.zeros((image_size[1], image_size[0], 3), np.uint8)
        for rec in recs:
            for x, y in rec["img"]:
                cv2.circle(cover, (int(round(x)), int(round(y))), 4, (0, 255, 0), -1)
        cv2.imwrite(os.path.join(debug_dir, "coverage_%s.png" % cam), cover)
        for rec in recs[:12]:
            im = cv2.imread(rec["path"], cv2.IMREAD_COLOR)
            for (x, y), cid in zip(rec["img"], rec["ids"]):
                cv2.circle(im, (int(round(x)), int(round(y))), 8, (0, 255, 0), 2)
                cv2.putText(im, str(int(cid)), (int(x) + 8, int(y) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.imwrite(os.path.join(debug_dir, "%s_%s" % (
                cam, os.path.basename(rec["path"]))), im)

    pair_T = {}
    report["pairs"] = {}
    for a, b in zip(cameras[:-1], cameras[1:]):
        seqs = sorted(set(detections[a]) & set(detections[b]))
        obj, ip1, ip2, used = make_pair_data(
            detections[a], detections[b], seqs, args.min_pair_corners
        )
        print("pair %-7s %-7s common sets=%d" % (a, b, len(obj)))
        if len(obj) < 3:
            raise SystemExit("pair %s-%s: only %d synchronized views" % (a, b, len(obj)))
        st = estimate_pair_extrinsics(
            detections[a], detections[b], seqs, cals_omni[a], cals_omni[b],
            args.min_pair_corners
        )
        rms = pair_reproj_rms(
            detections[a], detections[b], st["inlier_seqs"], cals_omni[a], cals_omni[b],
            st["R"], st["T"], args.min_pair_corners
        )
        T = mat4(st["R"], st["T"])
        tnorm = float(np.linalg.norm(st["T"]))
        angle = rotation_deg(st["R"])
        print("        extrinsics rms=%.3f px |T|=%.1f mm rot=%.2f deg views=%d poses=%d" % (
            rms, tnorm * 1000.0, angle, len(seqs), st["views"]))
        if not (0.005 <= tnorm <= 0.5):
            raise SystemExit("pair %s-%s translation |T|=%.4f m looks invalid" % (a, b, tnorm))
        pair_T[(a, b)] = T
        report["pairs"]["%s-%s" % (a, b)] = {
            "method": "per_view_board_pose_lm", "rms": rms,
            "refine_rms": st.get("refine_rms"),
            "views": st["views"], "poses_used": st["used"],
            "all_view_rms": st["rms"], "inlier_rms": st["inlier_rms"],
            "translation_m": st["T"].reshape(-1).tolist(),
            "translation_norm_m": tnorm, "rotation_deg": angle,
            "T_cn_cnm1": T.tolist(), "seqs": st["seqs"],
            "inlier_seqs": st["inlier_seqs"],
        }

    fisheye_path = None
    if len(cals_fisheye) == len(cameras):
        fisheye_path = write_yaml(os.path.join(args.out_dir, "kalibr_cam_chain_fisheye.yaml"),
                                  cameras, cals_fisheye, pair_T, "pinhole", image_size)
    omni_path = write_yaml(os.path.join(args.out_dir, "kalibr_cam_chain.yaml"),
                           cameras, cals_omni, pair_T, "omni", image_size)
    report_path = os.path.join(args.out_dir, "calibration_report.json")
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2)
    print("wrote", omni_path)
    if fisheye_path is not None:
        print("wrote", fisheye_path)
    else:
        print("fisheye YAML skipped (not all cameras calibrated)")
    print("wrote", report_path)


if __name__ == "__main__":
    main()
