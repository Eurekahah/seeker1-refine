import rospy, sys, time
from sensor_msgs.msg import Image
rospy.init_node('count_img', anonymous=True)
counts={}
def make_cb(t):
    def cb(msg): counts[t]=counts.get(t,0)+1
    return cb
for t in ['/fisheye/left/image_raw','/fisheye/right/image_raw','/fisheye/bright/image_raw','/fisheye/bleft/image_raw']:
    rospy.Subscriber(t, Image, make_cb(t), queue_size=20)
time.sleep(5)
print(counts)
print({k:v/5.0 for k,v in counts.items()})
