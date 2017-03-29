#!/usr/bin/env python
import sys
sys.dont_write_bytecode = True
import rospy
from geometry_msgs.msg import Twist
import numpy as np
import roslib
import time


def rockabyebaby():
    vel_max = 0.3
    t0 = rospy.get_rostime().to_sec()
    T = 2
    while not rospy.is_shutdown():
        output_command = Twist()
        t = rospy.get_rostime().to_sec() - t0
        vel = vel_max * np.sin(np.pi*t/T)
        output_command.linear.x = vel
        # output_command.angular.z = dh.w
        # rospy.loginfo(output_command)
        pub.publish(output_command)

        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('jackal_control')
    rate = rospy.Rate(100)
    pub = rospy.Publisher('/cmd_vel/keyboard', Twist, queue_size=5)
    try:
        rockabyebaby()
    except rospy.ROSInterruptException:
        pass
