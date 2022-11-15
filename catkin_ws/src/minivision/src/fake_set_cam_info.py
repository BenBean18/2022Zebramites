#!/usr/bin/env python3

from __future__ import print_function

from sensor_msgs.srv import SetCameraInfo
from sensor_msgs.msg import CameraInfo
import rospy

def handle_add_two_ints(req):
    print(req)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('/set_camera_info', SetCameraInfo, handle_add_two_ints)
    pub = rospy.Publisher("/camera_info", CameraInfo, queue_size=5)
    print("Ready to add two ints.")
    while not rospy.is_shutdown():
        msg = CameraInfo()
        msg.K = [495.01673169074536, 0.0, 302.0656161138883, 0.0, 495.0247902026783, 246.53001027037237, 0.0, 0.0, 1.0]
        msg.D = [0.14705480718852015, -0.21096730519503384, 0.0015540604275837222, -0.01703127002318487, 0.0]
        msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.P = [504.3512878417969, 0.0, 291.02988349884254, 0.0, 0.0, 514.5285034179688, 247.23254714502218, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.header.frame_id = "pizero"
        msg.header.stamp = rospy.Time.now()
        msg.distortion_model = "plumb_bob"
        pub.publish(msg)

if __name__ == "__main__":
    add_two_ints_server()
