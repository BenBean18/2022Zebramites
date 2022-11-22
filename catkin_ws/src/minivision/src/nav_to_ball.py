#!/usr/bin/env python3

# Credit to http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29
import rospy

import math
import tf2_ros
import geometry_msgs.msg

rospy.init_node('nav_to_ball')

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

cmd_vel = rospy.Publisher('/minibot/diffbot_controller/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

print("Hi!")

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    try:
        trans = tfBuffer.lookup_transform("base_link", "ball", rospy.Time.now(), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()
        continue

    msg = geometry_msgs.msg.Twist()

    a = math.atan2(trans.transform.translation.y, trans.transform.translation.x)

    print(a)

    if (a < 0.05):
        msg.angular.z = 0
        cmd_vel.publish(msg)
        continue

    if a > 0:
        msg.angular.z = 6
    else:
        msg.angular.z = -6

    cmd_vel.publish(msg)

    rate.sleep()

msg = geometry_msgs.msg.Twist()
msg.angular.z = 0
cmd_vel.publish(msg)