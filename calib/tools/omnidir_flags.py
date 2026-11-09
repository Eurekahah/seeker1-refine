import cv2
print([x for x in dir(cv2.omnidir) if x.startswith("CALIB")])
print(vars(cv2.omnidir).get("CALIB_FIX_INTRINSIC"))
