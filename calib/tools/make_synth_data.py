import cv2, numpy as np, os, json, random
OUT='/tmp/synth_data'; os.makedirs(OUT, exist_ok=True)
size=(960,600); K=np.array([[270.,0,480.],[0,270.,300.],[0,0,1.]]); D=np.array([[-0.02],[0.003],[-0.001],[0.0002]])
tex=cv2.imread('/tmp/board_A3_50mm.png',0)
tex=cv2.resize(tex,(1000,714),interpolation=cv2.INTER_AREA)
board=cv2.aruco.CharucoBoard_create(7,5,0.05,0.0375,cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
H,W=size[1],size[0]
def proj(R,t,pts):
    pc=(R@pts.T).T+t.reshape(1,3)
    z=pc[:,2]; ok=z>1e-6
    p=np.zeros((len(pts),2),np.float64)
    if ok.any():
        pp,_=cv2.fisheye.projectPoints(pts[ok].reshape(1,-1,3),cv2.Rodrigues(R)[0],t.reshape(3,1),K,D)
        p[ok]=pp.reshape(-1,2)
    return p,ok
def visible(R,t):
    pts=np.array([[0,0,0],[.35,0,0],[.35,.25,0],[0,.25,0]],np.float64)
    p,ok=proj(R,t,pts)
    if not ok.all(): return False
    # at least 3 outer corners inside a generous image region
    inside=((p[:,0]>-0.2*W)&(p[:,0]<1.2*W)&(p[:,1]>-0.2*H)&(p[:,1]<1.2*H))
    return inside.sum()>=3
def render(R,t):
    pts=np.array([[0,0,0],[.35,0,0],[.35,.25,0],[0,.25,0]],np.float64)
    p,_=proj(R,t,pts)
    src=np.array([[0,0],[999,0],[999,713],[0,713]],np.float32)
    Hm=cv2.getPerspectiveTransform(src,p.astype(np.float32))
    canvas=np.full((H,W),255,np.uint8)
    warped=cv2.warpPerspective(tex,Hm,(W,H),dst=canvas,flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=255)
    return warped
R12=cv2.Rodrigues(np.array([0.02,0.6,-0.01]))[0]; T12=np.array([-0.08,0.005,0.01])
rng=np.random.RandomState(7); made=0; attempts=0
for cam in ['left','right']: os.makedirs(os.path.join(OUT,cam),exist_ok=True)
while made<24 and attempts<2000:
    attempts+=1
    rvec=rng.uniform(-0.45,0.45,(3,1)); R1=cv2.Rodrigues(rvec)[0]
    t1=np.array([rng.uniform(-.28,.28),rng.uniform(-.2,.2),rng.uniform(.38,.8)])
    R2=R12@R1; t2=R12@t1+T12
    if not (visible(R1,t1) and visible(R2,t2)): continue
    i1=render(R1,t1); i2=render(R2,t2)
    # reject if the board occupies almost nothing
    if (i1<128).sum()<3000 or (i2<128).sum()<3000: continue
    stamp=str(1760600000000000000+made*1000000000)
    cv2.imwrite(os.path.join(OUT,'left','%06d_%s.jpg'%(made,stamp)),i1,[cv2.IMWRITE_JPEG_QUALITY,95])
    cv2.imwrite(os.path.join(OUT,'right','%06d_%s.jpg'%(made,stamp)),i2,[cv2.IMWRITE_JPEG_QUALITY,95])
    made+=1
print('made',made,'attempts',attempts)
