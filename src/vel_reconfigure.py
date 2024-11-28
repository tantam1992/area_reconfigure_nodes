#!/usr/bin/env python3

import rospy
import dynamic_reconfigure.client
from geometry_msgs.msg import Pose, PoseStamped
import math
from std_msgs.msg import Bool

# List of ramp areas
ramp_areas = [
    [(-37.72, -5.84), (-29.60, -4.84), (-29.35, -8.54), (-37.05, -9.74)],
    [(-44.02, -4.97), (-47.30, -5.28), (-46.71, -7.91), (-43.92, -7.68)],
    [(0.95, -1.04), (3.00, -0.85), (3.075, 1.97), (0.87, 2.15)]
]

class VelReconfigureNode:
    def __init__(self):
        rospy.init_node('vel_reconfigure_node')

        self.current_pose = None
        self.current_goal = None
        self.enable_reconfiguration = True
        self.current_state = "NORMAL"  # Possible states: NORMAL, RAMP, NEAR_GOAL

        rospy.loginfo("Velocity reconfigure node started")

        rospy.Subscriber('robot_pose', Pose, self.pose_callback, queue_size=10)
        rospy.Subscriber('/move_base/current_goal', PoseStamped, self.goal_callback, queue_size=10)
        rospy.Subscriber('/rampreconf_enable', Bool, self.enable_callback, queue_size=10)

        try:
            self.reconfigure_client = dynamic_reconfigure.client.Client('move_base/DWAPlannerROS')
        except Exception as e:
            rospy.logerr(f"Failed to create Dynamic Reconfigure Client: {e}")
            self.reconfigure_client = None

        # Perform an initial evaluation of the state
        self.check_and_update_state()

    def pose_callback(self, pose_msg):
        self.current_pose = pose_msg
        if self.current_goal is None or not self.enable_reconfiguration:
            return

        # Check and update the robot's state
        self.check_and_update_state()

    def goal_callback(self, goal_msg):
        rospy.loginfo("New goal received. Resetting state to NORMAL.")
        self.current_goal = goal_msg

        # Reset state to NORMAL when a new goal is received
        self.current_state = "NORMAL"
        self.update_configurations("NORMAL")

    def enable_callback(self, enable_msg):
        self.enable_reconfiguration = enable_msg.data

    def check_and_update_state(self):
        if self.current_pose is None or self.current_goal is None or not self.enable_reconfiguration:
            return

        # Check if the robot is inside a ramp or near the goal
        inside_ramp = self.check_is_inside_any_area(self.current_pose.position, ramp_areas)
        near_goal = self.check_is_near_goal(self.current_pose, self.current_goal.pose)

        # State priority: RAMP > NEAR_GOAL > NORMAL
        if inside_ramp:
            new_state = "RAMP"
        elif self.current_state == "NEAR_GOAL" or near_goal:
            # If already in NEAR_GOAL, stay in NEAR_GOAL until a new goal is received
            new_state = "NEAR_GOAL"
        else:
            new_state = "NORMAL"

        # Update state only if it has changed
        if new_state != self.current_state:
            self.current_state = new_state
            self.update_configurations(new_state)

    def update_configurations(self, new_state):
        if new_state == "RAMP":
            rospy.loginfo("Robot is inside a ramp area. Setting max_vel_x=0.3 and min_vel_x=-0.15.")
            self.reconfigure_max_vel(0.3)
            self.reconfigure_min_vel(-0.15)
        elif new_state == "NEAR_GOAL":
            rospy.loginfo("Robot is near the goal. Setting max_vel_x=0.3.")
            self.reconfigure_max_vel(0.3)
        elif new_state == "NORMAL":
            rospy.loginfo("Robot is outside special areas. Resetting max_vel_x=0.5 and min_vel_x=-0.3.")
            self.reconfigure_max_vel(0.5)
            self.reconfigure_min_vel(-0.3)

    def reconfigure_max_vel(self, new_max_vel):
        if self.reconfigure_client:
            try:
                rospy.loginfo(f"Reconfiguring max_vel_x to: {new_max_vel}")
                params = {'max_vel_x': new_max_vel}
                self.reconfigure_client.update_configuration(params)
            except Exception as e:
                rospy.logerr(f"Failed to update max_vel_x: {e}")
        else:
            rospy.logwarn("Dynamic Reconfigure Client is not initialized. Cannot update max_vel_x.")

    def reconfigure_min_vel(self, new_min_vel):
        if self.reconfigure_client:
            try:
                rospy.loginfo(f"Reconfiguring min_vel_x to: {new_min_vel}")
                params = {'min_vel_x': new_min_vel}
                self.reconfigure_client.update_configuration(params)
            except Exception as e:
                rospy.logerr(f"Failed to update min_vel_x: {e}")
        else:
            rospy.logwarn("Dynamic Reconfigure Client is not initialized. Cannot update min_vel_x.")

    def check_is_near_goal(self, pose1, pose2):
        # Distance threshold for NEAR_GOAL state
        threshold = 0.5  # Stay in NEAR_GOAL if within this distance

        x_dist = pose1.position.x - pose2.position.x
        y_dist = pose1.position.y - pose2.position.y
        distance = math.sqrt(x_dist**2 + y_dist**2)

        return distance < threshold

    def check_is_inside_any_area(self, position, areas):
        for area in areas:
            if self.point_inside_polygon(position.x, position.y, area):
                return True
        return False

    @staticmethod
    def point_inside_polygon(x, y, vertices):
        n = len(vertices)
        inside = False
        p1x, p1y = vertices[0]
        for i in range(n + 1):
            p2x, p2y = vertices[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside


if __name__ == '__main__':
    try:
        node = VelReconfigureNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass