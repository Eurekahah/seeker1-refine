import cv2, glob, itertools, os
base = "/tmp/calib_test1"
cams = ["left", "right", "bright", "bleft"]
files = {c: sorted(glob.glob(os.path.join(base, c, "*.jpg"))) for c in cams}
orb = cv2.ORB_create(nfeatures=4000, fastThreshold=10)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
for a, b in itertools.combinations(cams, 2):
    inl = []
    for fa, fb in zip(files[a], files[b]):
        ia = cv2.imread(fa, 0); ib = cv2.imread(fb, 0)
        ia = cv2.resize(ia, None, fx=0.5, fy=0.5); ib = cv2.resize(ib, None, fx=0.5, fy=0.5)
        ka, da = orb.detectAndCompute(ia, None); kb, db = orb.detectAndCompute(ib, None)
        if da is None or db is None: inl.append(0); continue
        m = bf.knnMatch(da, db, k=2)
        good = [x[0] for x in m if len(x) == 2 and x[0].distance < 0.75 * x[1].distance]
        if len(good) < 8: inl.append(0); continue
        pa = np.float32([ka[g.queryIdx].pt for g in good]).reshape(-1,1,2)
        pb = np.float32([kb[g.trainIdx].pt for g in good]).reshape(-1,1,2)
        H, mask = cv2.findHomography(pa, pb, cv2.RANSAC, 4.0)
        inl.append(int(mask.sum()) if mask is not None else 0)
    print("%-14s %s  inliers=%s" % (a+"-"+b, "", inl))
