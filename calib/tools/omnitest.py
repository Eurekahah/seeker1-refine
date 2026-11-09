import cv2, numpy as np
size=(1920,1200); board=cv2.aruco.CharucoBoard_create(7,5,0.08,0.06,cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)); o0=board.chessboardCorners.astype(np.float32)
K=np.array([[530.,0,960.],[0,530.,600.],[0,0,1.]]); xi=np.array([[1.05]],np.float64); D=np.array([[0.01],[-0.002],[0.0003],[-0.0001]],np.float64)
rng=np.random.RandomState(1); oo=[]; ii=[]
for i in range(24):
    rvec=rng.uniform(-0.5,0.5,(3,1)); tvec=np.array([[rng.uniform(-.2,.2)],[rng.uniform(-.15,.15)],[rng.uniform(.35,.8)]],np.float64)
    p,_=cv2.omnidir.projectPoints(o0.reshape(-1,1,3),rvec,tvec,K,xi,D)
    oo.append(o0.copy()); ii.append(p.reshape(-1,2).astype(np.float32)+rng.normal(0,0.1,(len(o0),2)).astype(np.float32))
flags=cv2.omnidir.CALIB_USE_GUESS|cv2.omnidir.CALIB_FIX_SKEW; crit=(3,200,1e-9)
shapes=[("Nx3",[o.reshape(-1,3) for o in oo],[p.reshape(-1,2) for p in ii]),("Nx1x3",[o.reshape(-1,1,3) for o in oo],[p.reshape(-1,1,2) for p in ii]),("1xNx3",[o.reshape(1,-1,3) for o in oo],[p.reshape(1,-1,2) for p in ii])]
for name,ob,im in shapes:
    try:
        r=cv2.omnidir.calibrate(ob,im,size,K.copy(),xi.copy(),D.copy(),flags,crit)
        print(name,"OK rms",r[0],"K",np.round([r[1][0,0],r[1][1,1],r[1][0,2],r[1][1,2]],2),"xi",r[2],"D",np.round(r[3].reshape(-1),5))
    except Exception as e: print(name,"ERR",str(e).splitlines()[-1])
