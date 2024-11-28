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
near_goal_dist = 0.8

class VelReconfigureNode:
    def __init__(self):
        rospy.init_node('vel_reconfigure_node')

        self.current_pose = None
        self.current_goal = None
        self.enable_reconfiguration = True
        self.current_state = "NORMAL"  # Possible states: NORMAL, RAMP, NEAR_GOAL

        rospy.loginfo("Velocity and sim_time reconfigure node started")

        rospy.Subscriber('robot_pose', Pose, self.pose_callback, queue_size=10)
        rospy.Subscriber('/move_base/current_goal', PoseStamped, self.goal_callback, queue_size=10)
        rospy.Subscriber('/rampreconf_enable', Bool, self.enable_callback, queue_size=10)

        try:
            self.reconfigure_client = dynamic_reconfigure.client.Client('move_base/DWAPlannerROS')
        except Exception as e:
            rospy.logerr(f"Failed to create Dynamic Reconfigure Client: {e}")
            self.reconfigure_client = None

        # self.rate = rospy.Rate(5)

    def pose_callback(self, pose_msg):
        self.current_pose = pose_msg
        if self.current_goal is None or not self.enable_reconfiguration:
            return

        inside_ramp = self.check_is_inside_any_area(pose_msg.position, ramp_areas)
        near_goal = self.check_is_near_goal(pose_msg, self.current_goal.pose)

        if inside_ramp:
            new_state = "RAMP"
        elif near_goal:
            new_state = "NEAR_GOAL"
        else:
            new_state = "NORMAL"

        if new_state != self.current_state:
            self.current_state = new_state
            self.update_configurations(new_state)

        # self.rate.sleep()

    def update_configurations(self, new_state):
        if new_state == "RAMP":
            rospy.loginfo("Robot is inside a ramp area. Setting max_vel_x=0.3 and min_vel_x=-0.15.")
            self.reconfigure_sim_time(3.0)
            self.reconfigure_max_vel(0.3)
            self.reconfigure_min_vel(-0.15)
        elif new_state == "NEAR_GOAL":
            rospy.loginfo("Robot is near the goal. Setting max_vel_x=0.3 and sim_time=1.1.")
            self.reconfigure_sim_time(1.1)
            self.reconfigure_max_vel(0.3)
        elif new_state == "NORMAL":
            rospy.loginfo("Robot is outside special areas. Resetting max_vel_x=0.5, min_vel_x=-0.3, and sim_time=3.0")
            self.reconfigure_sim_time(3.0)
            self.reconfigure_max_vel(0.5)
            self.reconfigure_min_vel(-0.3)

    def goal_callback(self, goal_msg):
        rospy.loginfo("New goal received. Checking location.")
        self.current_goal = goal_msg
        if not self.check_is_inside_any_area(self.current_pose.position, ramp_areas):
            self.current_state = "NORMAL"
            self.update_configurations("NORMAL")

    def enable_callback(self, enable_msg):
        self.enable_reconfiguration = enable_msg.data

    def reconfigure_sim_time(self, new_sim_time):
        if self.reconfigure_client:
            rospy.loginfo(f"Reconfiguring sim_time to: {new_sim_time}")
            params = {'sim_time': new_sim_time}
            self.reconfigure_client.update_configuration(params)

    def reconfigure_max_vel(self, new_max_vel):
        if self.reconfigure_client:
            rospy.loginfo(f"Reconfiguring max_vel_x to: {new_max_vel}")
            params = {'max_vel_x': new_max_vel}
            self.reconfigure_client.update_configuration(params)

    def reconfigure_min_vel(self, new_min_vel):
        if self.reconfigure_client:
            rospy.loginfo(f"Reconfiguring min_vel_x to: {new_min_vel}")
            params = {'min_vel_x': new_min_vel}
            self.reconfigure_client.update_configuration(params)

    def check_is_near_goal(self, pose1, pose2):
        x_dist = pose1.position.x - pose2.position.x
        y_dist = pose1.position.y - pose2.position.y
        distance = math.sqrt(x_dist**2 + y_dist**2)
        return distance < near_goal_dist

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
