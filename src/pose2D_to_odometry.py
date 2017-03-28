#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Point, Pose2D, Quaternion
from nav_msgs.msg import Odometry


class Pose2DtoOdometry:
    def __init__(self):
        # subscriber/publisher setup
        self.sub = rospy.Subscriber('LaserScan/pose2D', Pose2D, self.pose_cb)
        self.pub = rospy.Publisher('LaserScan/odom', Odometry, queue_size=5)

        # create output nav_msgs/Odometry object and populate covariances
        # not too important, since they don't feed into robot_localization anyway
        self.output = Odometry()
        self.output.pose.covariance[0] = 0.008 # X, eyeballed from simulation * FoS for real system
        self.output.pose.covariance[7] = 0.008 # Y, eyeballed from simulation * FoS for real system
        self.output.pose.covariance[14] = 99.9 # no z information
        self.output.pose.covariance[21] = 99.9 # no roll information
        self.output.pose.covariance[28] = 99.9 # no pitch information
        self.output.pose.covariance[35] = 0.1 # yaw, complete guess, roughly 5.7 degrees

    def pose_cb(self, data):
        # fill in header info:
        self.output.header.stamp = rospy.get_rostime()
        self.output.header.frame_id = 'odometry_corrected'

        # fill in position data:
        self.output.pose.pose.position = Point(data.x, data.y, 0.0)

        # fill in orientation data:
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, data.theta)
        self.output.pose.pose.orientation = Quaternion(*q)

        # publish:
        self.pub.publish(self.output)


if __name__ == '__main__':
    rospy.init_node('Pose2D_to_Odometry')
    converter = Pose2DtoOdometry()
    rospy.spin()
