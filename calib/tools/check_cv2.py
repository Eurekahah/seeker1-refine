import cv2, numpy as np
print("cv2", cv2.__version__)
print("numpy", np.__version__)
print("has_aruco", hasattr(cv2, "aruco"))
if hasattr(cv2, "aruco"):
    names = [x for x in dir(cv2.aruco) if "Charuco" in x or "Detector" in x or "Dictionary" in x]
    print(names)
print("has_fisheye", hasattr(cv2, "fisheye"))
