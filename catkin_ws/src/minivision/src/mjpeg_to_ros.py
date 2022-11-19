#!/usr/bin/env python3
import cv2
from urllib.request import urlopen # for py3/noetic
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

rospy.init_node('mjpeg_to_ros', anonymous=True)
bridge = CvBridge()
stream = urlopen(rospy.get_param("~url"))
pub = rospy.Publisher(rospy.get_param("~ns")+"/image_rect", Image, queue_size=10)
pub2 = rospy.Publisher(rospy.get_param("~ns")+"/camera_info", CameraInfo, queue_size=10)
bytes = b''
while not rospy.is_shutdown():
    bytes += stream.read(1024)
    a = bytes.find(b'\xff\xd8')
    b = bytes.find(b'\xff\xd9')
    if a != -1 and b != -1:
        t = rospy.Time.now()
        jpg = bytes[a:b+2]
        bytes = bytes[b+2:]
        i = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        image_message = bridge.cv2_to_imgmsg(i, encoding="bgr8")
        image_message.header.stamp = t
        image_message.header.frame_id = "pizero"
        pub.publish(image_message)
        msg = CameraInfo()
        msg.K = [495.01673169074536, 0.0, 302.0656161138883, 0.0, 495.0247902026783, 246.53001027037237, 0.0, 0.0, 1.0]
        msg.D = [0.14705480718852015, -0.21096730519503384, 0.0015540604275837222, -0.01703127002318487, 0.0]
        msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.P = [504.3512878417969, 0.0, 291.02988349884254, 0.0, 0.0, 514.5285034179688, 247.23254714502218, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.header.frame_id = "pizero"
        msg.header.stamp = t
        msg.distortion_model = "plumb_bob"
        pub2.publish(msg)

