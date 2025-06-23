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

corner_areas_LG = [
    [(-89.56, 0.81), (-85.24, 1.30), (-86.00, 6.62), (-90.27, 6.17)],
    [(-84.10, -7.33), (-83.70, -10.36), (-87.15, -10.793), (-87.42, -7.74)],
    [(-27.38, -1.77), (-24.97, -1.54), (-24.63, -4.35), (-27.08, -4.71)]
]

corner_areas_5F = [
    # [(-89.56, 0.81), (-85.24, 1.30), (-86.00, 6.62), (-90.27, 6.17)],
    # [(-84.10, -7.33), (-83.70, -10.36), (-87.15, -10.793), (-87.42, -7.74)],
    [(-4.42, -6.39), (-2.68, -6.34), (-2.55, -7.44), (-4.33, -7.92)]
]

dock_areas_LG = [
    [(-82.43, -6.69), (-76.86, -5.87), (-76.69, -7.64), (-81.72, -8.29)]

]

dock_area_5F = [
    [(2.70, -16.57), (0.99, -16.56), (0.96, -19.14), (2.76, -19.15)]
]

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
        if self.current_goal is None:
            return

        inside_ramp = self.check_is_inside_any_area(pose_msg.position, ramp_areas)
        indside_corner_LG = self.check_is_inside_any_area(pose_msg.position, corner_areas_LG)
        indside_corner_5F = self.check_is_inside_any_area(pose_msg.position, corner_areas_5F)
        near_goal_and_in_dock = self.check_is_near_goal_and_in_dock(pose_msg, self.current_goal.pose)

        if self.enable_reconfiguration:
            if inside_ramp:
                new_state = "RAMP"
            elif near_goal_and_in_dock:
                new_state = "NEAR_GOAL"
            elif indside_corner_LG:
                new_state = "CORNER"            
            else:
                new_state = "NORMAL"
        else:
            if near_goal_and_in_dock:
                new_state = "NEAR_GOAL"
            elif indside_corner_5F:
                new_state = "CORNER"            
            else:
                new_state = "NORMAL"

        if new_state != self.current_state:
            self.current_state = new_state
            self.update_configurations(new_state)

        # self.rate.sleep()

    def update_configurations(self, new_state):
        if new_state == "RAMP":
            rospy.loginfo("Robot is inside a ramp area. Setting max_vel_x=0.4 and min_vel_x=-0.3.")
            # self.reconfigure_sim_time(3.0)
            self.reconfigure_max_vel(0.4)
            self.reconfigure_min_vel(-0.3)
        elif new_state == "NEAR_GOAL":
            rospy.loginfo("Robot is near the goal. Setting max_vel_x=0.3.")
            # self.reconfigure_sim_time(1.1)
            self.reconfigure_max_vel(0.3)
        elif new_state == "CORNER":
            rospy.loginfo("Robot is inside a corner area. Setting max_vel_x=0.45 and min_vel_x=-0.4.")
            self.reconfigure_max_vel(0.45)
            self.reconfigure_min_vel(-0.4)            
        elif new_state == "NORMAL":
            rospy.loginfo("Robot is outside special areas. Resetting max_vel_x=0.6, min_vel_x=-0.5.")
            # self.reconfigure_sim_time(3.0)
            self.reconfigure_max_vel(0.6)
            self.reconfigure_min_vel(-0.5)

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
            params_x = {'max_vel_x': new_max_vel}
            params_trans = {'max_vel_trans': new_max_vel}
            self.reconfigure_client.update_configuration(params_x)
            self.reconfigure_client.update_configuration(params_trans)

    def reconfigure_min_vel(self, new_min_vel):
        if self.reconfigure_client:
            rospy.loginfo(f"Reconfiguring min_vel_x to: {new_min_vel}")
            params = {'min_vel_x': new_min_vel}
            self.reconfigure_client.update_configuration(params)

    def check_is_near_goal_and_in_dock(self, pose1, pose2):
        near_goal_dist = 0.5
        x_dist = pose1.position.x - pose2.position.x
        y_dist = pose1.position.y - pose2.position.y
        distance = math.sqrt(x_dist**2 + y_dist**2)
        
        # Check if the robot is near the goal AND inside any of the dock areas
        is_near = distance < near_goal_dist
        
        # Determine which dock areas to use based on your setup (LG or 5F or both)
        # This example checks both.  Adjust as needed.
        is_in_dock = self.check_is_inside_any_area(pose1.position, dock_areas_LG) or \
                     self.check_is_inside_any_area(pose1.position, dock_area_5F)
        
        return is_near and is_in_dock

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