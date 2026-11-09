import cv2
print("omnidir", hasattr(cv2, "omnidir"))
if hasattr(cv2, "omnidir"):
    print([x for x in dir(cv2.omnidir) if not x.startswith("_")][:40])
