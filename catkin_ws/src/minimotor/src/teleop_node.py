#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

max_linear_speed = 1.23
max_angular_speed = 55.0
intake_radians_per_second = 37.0

rospy.init_node('teleop_node', anonymous=True)

pub = rospy.Publisher("/minibot/diffbot_controller/cmd_vel", Twist)
pub_intake = rospy.Publisher("/minibot/intake_controller/command", Float64)

def callback(msg):
    t = Twist()
    t.linear.x = msg.axes[1] * max_linear_speed
    t.angular.z = msg.axes[3] * max_angular_speed
    pub.publish(t)
    if (msg.axes[2] == 1 and msg.axes[5] == 1):
        pub_intake.publish(0)
    else:
        if (msg.axes[2] != 0 and msg.axes[2] != 1):
            intake_control = (msg.axes[2] - 1.0) / -2.0 * intake_radians_per_second # -1 = fully pressed, 1 = not pressed. left trigger
            pub_intake.publish(intake_control)
        if (msg.axes[5] != 0 and msg.axes[5] != 1):
            outtake_control = -1 * ((msg.axes[5] - 1.0) / -2.0 * intake_radians_per_second) # -1 = fully pressed, 1 = not pressed. right trigger
            pub_intake.publish(outtake_control)

rospy.Subscriber("/minibot/joy", Joy, callback)

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()