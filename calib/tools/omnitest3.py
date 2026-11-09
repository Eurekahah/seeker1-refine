import cv2, numpy as np, itertools
size=(1920,1200); board=cv2.aruco.CharucoBoard_create(7,5,0.08,0.06,cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)); o0=board.chessboardCorners.astype(np.float32)
K=np.array([[530.,0,960.],[0,530.,600.],[0,0,1.]]); xi=np.array([[1.05]],np.float64); D=np.array([[0.01],[-0.002],[0.0003],[-0.0001]],np.float64)
rng=np.random.RandomState(1); oo=[]; ii=[]
for i in range(12):
    rvec=rng.uniform(-0.5,0.5,(3,1)); tvec=np.array([[rng.uniform(-.2,.2)],[rng.uniform(-.15,.15)],[rng.uniform(.35,.8)]],np.float64)
    p,_=cv2.omnidir.projectPoints(o0.reshape(-1,1,3),rvec,tvec,K,xi,D)
    oo.append(o0.copy()); ii.append(p.reshape(-1,2).astype(np.float32))
flags=cv2.omnidir.CALIB_USE_GUESS|cv2.omnidir.CALIB_FIX_SKEW; crit=(3,200,1e-9)
def sh(a, kind):
    n=len(a)
    if kind=="Nx3": return a
    if kind=="Nx1x3": return a.reshape(n,1,3)
    if kind=="1xNx3": return a.reshape(1,n,3)
    if kind=="Nx3x1": return a.reshape(n,3,1)
def shi(a, kind):
    n=len(a)
    if kind=="Nx2": return a
    if kind=="Nx1x2": return a.reshape(n,1,2)
    if kind=="1xNx2": return a.reshape(1,n,2)
    if kind=="Nx2x1": return a.reshape(n,2,1)
for ko,ki in itertools.product(["Nx3","Nx1x3","1xNx3","Nx3x1"],["Nx2","Nx1x2","1xNx2","Nx2x1"]):
    ob=[sh(o,ko).copy() for o in oo]; im=[shi(p,ki).copy() for p in ii]
    try:
        r=cv2.omnidir.calibrate(ob,im,size,K.copy(),xi.copy(),D.copy(),flags,crit)
        print(ko,ki,"OK")
    except Exception as e:
        print(ko,ki,"ERR",str(e).splitlines()[-1].split("error: ")[-1][:80])
