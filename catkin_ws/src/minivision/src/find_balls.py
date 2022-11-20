#!/usr/bin/env python3

# (written with the help of GitHub Copilot)
# Use OpenCV to detect a yellow sphere and draw a rectangle around it
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64
from image_geometry import PinholeCameraModel
import tf_conversions

import tf2_ros
import geometry_msgs.msg

# Import the OpenCV bridge
from cv_bridge import CvBridge
bridge = CvBridge()

# Initialize the ROS node
rospy.init_node('ball_finder')

caminfo = None

def distance(px):
    try:
        return 23.3 * pow(px, -0.988)
    except:
        # div by 0, maybe
        return -1

# Callback function
def callback(msg):
    # Convert the image from a ROS message to a numpy array
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # Convert to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)

    # Define the range of yellow
    lower_yellow = np.array([45/2, 0.2*255, 0.15*255]) # HLS not HSL
    upper_yellow = np.array([55/2, 0.6*255, 1.0*255])

    # Threshold the HSV image to get only yellow colors
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Find blobs in the mask
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    try:
        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)

        # Draw a blue rectangle around the found contour
        x, y, w, h = cv2.boundingRect(largest_contour)
        cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)

        mess = Float64()
        mess.data = distance(w)
        pub2.publish(mess)

        pcm = PinholeCameraModel()
        pcm.fromCameraInfo(caminfo)
        tf = pcm.projectPixelTo3dRay(pcm.rectifyPoint((x, y)))
        ntf = (tf[0] * distance(w), tf[1] * distance(w), tf[2] * distance(w))
        rospy.loginfo_throttle(0.5, ntf)

        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "pizero"
        t.child_frame_id = "ball"
        t.transform.translation.x = ntf[0]
        t.transform.translation.y = ntf[1]
        t.transform.translation.z = ntf[2]

        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1

        br.sendTransform(t)
    except Exception as e:
        rospy.logwarn_throttle(1, e)

    # Publish img on the ROS topic /detected_balls
    pub.publish(bridge.cv2_to_imgmsg(img, encoding='bgr8'))

# Create a ROS subscriber to the topic /pizero/image_rect with the type sensor_msgs/Image
# The callback function is called when a new message is received
# The queue_size is the number of messages to buffer before throwing away old ones
rospy.Subscriber('/pizero/image_rect', Image, callback, queue_size=1)

def infoCb(msg):
    global caminfo
    caminfo = msg

rospy.Subscriber('/pizero/camera_info', CameraInfo, infoCb, queue_size=1)


# Create a ROS publisher on the topic /detected_balls with the type sensor_msgs/Image
pub = rospy.Publisher('/detected_balls', Image, queue_size=1)

pub2 = rospy.Publisher('/ball_distance', Float64, queue_size=1)

# Spin the ROS node
rospy.spin()