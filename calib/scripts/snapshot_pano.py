#!/usr/bin/env python3
import sys
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

out = sys.argv[1] if len(sys.argv) > 1 else "/data/local/tmp/pano.jpg"
rospy.init_node("pano_snapshot", anonymous=True)
bridge = CvBridge()

def cb(msg):
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    cv2.imwrite(out, img)
    print("saved %s %dx%d" % (out, img.shape[1], img.shape[0]), flush=True)
    rospy.signal_shutdown("done")

rospy.Subscriber("/panorama/image", Image, cb, queue_size=1)
try:
    rospy.spin()
except KeyboardInterrupt:
    pass
