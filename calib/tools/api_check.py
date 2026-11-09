import cv2, numpy as np
print("board attrs:", [x for x in dir(cv2.aruco.CharucoBoard_create(7,5,0.08,0.06,cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))) if "orn" in x or "chess" in x.lower()])
b = cv2.aruco.CharucoBoard_create(7,5,0.08,0.06,cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
print("chessboardCorners", b.chessboardCorners.shape, b.chessboardCorners[:2])
print("stereo doc:", cv2.fisheye.stereoCalibrate.__doc__)
print("calibrate doc:", cv2.fisheye.calibrate.__doc__[:1000])
