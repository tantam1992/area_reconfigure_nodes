#!/usr/bin/env python3

import rospy
from time import sleep
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped

class PoseReinitializer:
    def __init__(self):
        rospy.init_node('pose_reinitializer')
        rospy.loginfo("Pose Reinitializer node started")
        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.last_angular_z = 0.0
        # self.rate = rospy.Rate(0.016)  # Set the rate to 0.1 Hz

    def odom_callback(self, msg):
        angular_z = msg.twist.twist.angular.z
        if abs(angular_z) > 0.05 and abs(angular_z - self.last_angular_z) > 0.05:
            self.last_angular_z = angular_z
            self.publish_initial_pose()

    def pose_callback(self, msg):
        self.current_pose = msg

    def publish_initial_pose(self):
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.stamp = rospy.Time.now()
        initial_pose_msg.header.frame_id = "map"  # Adjust frame_id if necessary
        initial_pose_msg.pose.pose.position = self.current_pose.pose.pose.position
        initial_pose_msg.pose.pose.orientation = self.current_pose.pose.pose.orientation
        initial_pose_msg.pose.covariance = self.current_pose.pose.covariance
        self.initialpose_pub.publish(initial_pose_msg)
        rospy.loginfo("Pose Reinitialized")
        sleep(10)

    def run(self):
        while not rospy.is_shutdown():
            # self.rate.sleep()
            rospy.spin()

if __name__ == '__main__':
    try:
        pose_reinitializer = PoseReinitializer()
        pose_reinitializer.run()
    except rospy.ROSInterruptException:
        pass
