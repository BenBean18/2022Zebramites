#!/usr/bin/env python3
# license removed for brevity
import rospy, time
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('/minibot/diffbot_controller/cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    s = time.time()
    msg = Twist()
    msg.linear.x = 0.5
    pub.publish(msg)
    while time.time() - s < 1:
        rate.sleep()
        pub.publish(msg)
    msg.linear.x = 0
    pub.publish(msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass