import cv2, numpy as np, importlib.util, sys
spec = importlib.util.spec_from_file_location("cal", "/tmp/calibrate_fisheye.py")
cal = importlib.util.module_from_spec(spec); spec.loader.exec_module(cal)
size = (1920, 1200)
board = cv2.aruco.CharucoBoard_create(7,5,0.08,0.06,cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
obj0 = board.chessboardCorners.astype(np.float64)
K = np.array([[530.,0,960.],[0,530.,600.],[0,0,1.]])
D = np.array([[-0.02],[0.003],[-0.001],[0.0002]])
rng = np.random.RandomState(1)
obj, img = [], []
for i in range(24):
    rvec = rng.uniform(-0.5,0.5,(3,1)); tvec = np.array([[rng.uniform(-.2,.2)],[rng.uniform(-.15,.15)],[rng.uniform(.35,.8)]])
    p,_ = cv2.fisheye.projectPoints(obj0.reshape(1,-1,3), rvec, tvec, K, D)
    p = p.reshape(-1,2) + rng.normal(0,0.15,(len(obj0),2))
    obj.append(obj0); img.append(p.reshape(-1,1,2))
r = cal.run_fisheye(obj,img,size)
print("FISHEYE K", np.round(r["K"],3).tolist(), "D", np.round(r["D"].reshape(-1),5).tolist(), "rms", r["rms"])
# omni synthetic
xi = 1.05; Do = np.array([[0.01],[-0.002],[0.0003],[-0.0001]])
obj2, img2 = [], []
for i in range(24):
    rvec = rng.uniform(-0.5,0.5,(3,1)); tvec = np.array([[rng.uniform(-.2,.2)],[rng.uniform(-.15,.15)],[rng.uniform(.35,.8)]])
    p,_ = cv2.omnidir.projectPoints(obj0.reshape(-1,1,3), rvec, tvec, K, xi, Do)
    p = p.reshape(-1,2) + rng.normal(0,0.15,(len(obj0),2))
    obj2.append(obj0); img2.append(p.reshape(-1,1,2))
o = cal.run_omni(obj2,img2,size)
print("OMNI K", np.round(o["K"],3).tolist(), "xi", o["xi"], "D", np.round(o["D"].reshape(-1),5).tolist(), "rms", o["rms"])
# stereo synthetic
R = cv2.Rodrigues(np.array([0.02, 0.9, -0.01]))[0]; T = np.array([[-0.09],[0.005],[0.01]])
so, si1, si2 = [], [], []
for i in range(16):
    rvec = rng.uniform(-0.5,0.5,(3,1)); tvec = np.array([[rng.uniform(-.2,.2)],[rng.uniform(-.15,.15)],[rng.uniform(.35,.8)]])
    p1,_ = cv2.fisheye.projectPoints(obj0.reshape(1,-1,3), rvec, tvec, K, D)
    Rv,_ = cv2.Rodrigues(R @ cv2.Rodrigues(rvec)[0]); Tv = (R @ tvec.reshape(3,1) + T)
    p2,_ = cv2.fisheye.projectPoints(obj0.reshape(1,-1,3), Rv, Tv, K, D)
    so.append(obj0.reshape(-1,1,3)); si1.append(p1.reshape(-1,1,2)); si2.append(p2.reshape(-1,1,2))
st = cal.run_stereo(so,si1,si2,size,{"K":K,"D":D},{"K":K,"D":D})
print("STEREO rms", st["rms"], "T", np.round(st["T"].reshape(-1),4).tolist(), "rot", round(cal.rotation_deg(st["R"]),2))
