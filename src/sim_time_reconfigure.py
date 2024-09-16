#!/usr/bin/env python3

import rospy
import dynamic_reconfigure.client
from geometry_msgs.msg import Pose, PoseStamped
import math  # Importing math for distance calculation
from std_msgs.msg import Bool

class SimTimeReconfigureNode:
    def __init__(self):
        rospy.init_node('sim_time_reconfigure_node')

        self.current_pose = None
        self.current_goal = None
        self.near_goal = False
        self.reconfiguration_done = False  # Track if reconfiguration has been done

        rospy.loginfo("sim time reconfigure node started")

        # Subscribe to robot's pose
        rospy.Subscriber('robot_pose', Pose, self.pose_callback)
        rospy.Subscriber('/move_base/current_goal', PoseStamped, self.goal_callback)

        # Dynamic Reconfigure client
        self.reconfigure_client = dynamic_reconfigure.client.Client('move_base/DWAPlannerROS')

    def wait_for_reconfigure_services(self):
        try:
            rospy.loginfo("Waiting for dynamic reconfigure services...")
            rospy.wait_for_service('/move_base/DWAPlannerROS/set_parameters')
            self.global_reconfigure_client = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS')
            rospy.loginfo("Dynamic reconfigure services are ready.")
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to connect to dynamic reconfigure services: {e}")
            rospy.signal_shutdown("Shutting down due to service connection failure.")

    def goal_callback(self, goal_msg):
        self.current_goal = goal_msg

    def pose_callback(self, pose_msg):
        self.current_pose = pose_msg
        if self.current_goal is not None:  # Check if current_goal is set
            near_goal = self.check_is_near_goal(pose_msg, self.current_goal.pose)

            if near_goal:
                if not self.reconfiguration_done:  # Perform reconfiguration only once
                    rospy.loginfo("Robot is near goal.")
                    self.reconfigure_sim_time(1.1)  # Adjust the sim_time parameter
                    self.reconfiguration_done = True  # Set reconfiguration status
            else:
                if self.reconfiguration_done:
                    rospy.loginfo("Robot is not near goal.")
                    self.reconfigure_sim_time(3.0)  # Adjust the sim_time parameter
                    self.reconfiguration_done = False  # Reset reconfiguration status

    def enable_callback(self, enable_msg):
        self.enable_reconfiguration = enable_msg.data  # Update enable/disable status

    def reconfigure_sim_time(self, new_sim_time):
        self.wait_for_reconfigure_services()
        rospy.loginfo("Reconfiguring sim_time to: {}".format(new_sim_time))
        params = {'sim_time': new_sim_time}
        self.reconfigure_client.update_configuration(params)

    def check_is_near_goal(self, pose1, pose2):
        x_distance = pose1.position.x - pose2.position.x
        y_distance = pose1.position.y - pose2.position.y
        distance = math.sqrt((x_distance * x_distance) + (y_distance * y_distance))
        return distance < 0.4  # Return True if within 0.4 units

if __name__ == '__main__':
    try:
        node = SimTimeReconfigureNode()
        rospy.spin()  # Process incoming messages
    except rospy.ROSInterruptException:
        pass