#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Point, Pose2D, Quaternion
from nav_msgs.msg import Odometry


class Pose2DtoOdometry:
    def __init__(self):
        self.pub = rospy.Publisher('LaserScan/odom', Odometry, queue_size=5)
        self.sub = rospy.Subscriber('LaserScan/pose2D', Pose2D, self.pose_cb)
        self.output = Odometry()

    def pose_cb(self, data):
        self.output.header.stamp = rospy.get_rostime()
        self.output.header.frame_id = 'odometry_corrected'

        self.output.pose.pose.position = Point(data.x, data.y, 0.0)

        q = tf.transformations.quaternion_from_euler(0.0, 0.0, data.theta)
        self.output.pose.pose.orientation = Quaternion(*q)

        self.output.pose.covariance[0] = 0.008 # X, eyeballed from simulation + fudge factor for real system
        self.output.pose.covariance[7] = 0.008 # Y, eyeballed from simulation + fudge factor for real system
        # self.output.pose.covariance[14] = 99.9 # no z information
        # self.output.pose.covariance[21] = 99.9 # no roll information
        # self.output.pose.covariance[28] = 99.9 # no pitch information
        self.output.pose.covariance[35] = 0.1 # yaw, complete guess, roughly 5.7 degrees

        self.pub.publish(self.output)


if __name__ == '__main__':
    try:
        rospy.init_node('Pose2D_to_Odometry')
        converter = Pose2DtoOdometry()
    except KeyboardInterrupt:
        print("Shutting down")

    rospy.spin()
