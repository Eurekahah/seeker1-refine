import cv2, numpy as np, subprocess, json, tempfile, os
size=(1920,1200); board=cv2.aruco.CharucoBoard_create(7,5,0.08,0.06,cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)); o0=board.chessboardCorners.astype(np.float64)
K=np.array([[530.,0,960.],[0,530.,600.],[0,0,1.]]); xi=np.array([[1.05]]); D=np.array([[0.01],[-0.002],[0.0003],[-0.0001]])
rng=np.random.RandomState(1); oo=[]; ii=[]
for i in range(24):
    rvec=rng.uniform(-0.5,0.5,(3,1)); tvec=np.array([[rng.uniform(-.2,.2)],[rng.uniform(-.15,.15)],[rng.uniform(.35,.8)]])
    p,_=cv2.omnidir.projectPoints(o0.reshape(-1,1,3),rvec,tvec,K,xi,D); p=p.reshape(-1,2)+rng.normal(0,0.15,(len(o0),2))
    oo.append(o0); ii.append(p)
fd,path=tempfile.mkstemp(suffix='.txt'); os.close(fd)
with open(path,'w') as f:
    f.write(str(len(oo))+'\n')
    for o,p in zip(oo,ii):
        f.write(str(len(o))+'\n')
        for a,b in zip(o,p): f.write('%.12g %.12g %.12g %.12g %.12g\n'%(a[0],a[1],a[2],b[0],b[1]))
r=subprocess.run(['omni_calib',path,str(size[0]),str(size[1]),str(K[0,0]),str(K[1,1]),str(K[0,2]),str(K[1,2])],capture_output=True,text=True)
print("rc",r.returncode,r.stderr[:300]); print(r.stdout[:500])
if r.returncode==0:
    d=json.loads(r.stdout); print("K",np.round([d['K'][0],d['K'][4],d['K'][2],d['K'][5]],3),"xi",round(d['xi'],4),"D",np.round(d['D'],5),"rms",round(d['rms'],4))
