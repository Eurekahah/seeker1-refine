import cv2, numpy as np, importlib.util
spec = importlib.util.spec_from_file_location("cal", "/tmp/calibrate_fisheye.py")
cal = importlib.util.module_from_spec(spec); spec.loader.exec_module(cal)
size=(1920,1200); board=cv2.aruco.CharucoBoard_create(7,5,0.08,0.06,cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)); o0=board.chessboardCorners.astype(np.float64)
K=np.array([[530.,0,960.],[0,530.,600.],[0,0,1.]]); D=np.array([[-0.02],[0.003],[-0.001],[0.0002]])
rng=np.random.RandomState(1); obj=[]; img=[]
for i in range(24):
    rvec=rng.uniform(-0.5,0.5,(3,1)); tvec=np.array([[rng.uniform(-.2,.2)],[rng.uniform(-.15,.15)],[rng.uniform(.35,.8)]])
    p,_=cv2.fisheye.projectPoints(o0.reshape(1,-1,3),rvec,tvec,K,D); p=p.reshape(-1,2)+rng.normal(0,0.15,(len(o0),2))
    obj.append(o0.reshape(-1,1,3)); img.append(p.reshape(-1,1,2))
for flags in [cv2.fisheye.CALIB_FIX_SKEW|cv2.fisheye.CALIB_USE_INTRINSIC_GUESS|cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC, cv2.fisheye.CALIB_FIX_SKEW|cv2.fisheye.CALIB_USE_INTRINSIC_GUESS, cv2.fisheye.CALIB_FIX_SKEW|cv2.fisheye.CALIB_USE_INTRINSIC_GUESS|cv2.fisheye.CALIB_CHECK_COND]:
    try:
        rms,Kc,Dc,rv,tv=cv2.fisheye.calibrate(obj,img,size,K.copy(),D.copy(),flags=flags,criteria=(3,200,1e-9))
        errs=cal.fisheye_view_errors([x.reshape(-1,3) for x in obj],[x.reshape(-1,2) for x in img],Kc,Dc,rv,tv)
        print("flags",flags,"rms",round(rms,3),"K",np.round([Kc[0,0],Kc[1,1],Kc[0,2],Kc[1,2]],2),"D",np.round(Dc.reshape(-1),5),"med/err",round(np.median(errs),3),round(errs.max(),3))
    except Exception as e: print("flags",flags,"ERR",str(e).splitlines()[-1])
