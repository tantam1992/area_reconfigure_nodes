#!/usr/bin/env python3

import rospy
import dynamic_reconfigure.client
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

# List of docking areas
docking_areas = [
    # Add your docking areas here
    [(-1.54, 47.50), (-1.54, 55.62), (-4.25, 55.49), (-4.16, 47.65)]
]

class ToleranceReconfigureNode:
    def __init__(self):
        rospy.init_node('tolerance_reconfigure_node')

        self.current_pose = None
        self.inside_docking = False
        self.reconfiguration_done = False  # Track if reconfiguration has been done
        self.enable_reconfiguration = True  # Track enable/disable status
        self.original_xy_goal_tolerance = rospy.get_param('/move_base/DWAPlannerROS/xy_goal_tolerance')

        rospy.loginfo("docking reconfigure node started")

        # Subscribe to robot's pose
        rospy.Subscriber('robot_pose', Pose, self.pose_callback)

        # Subscribe to reconfiguration enable/disable topic
        rospy.Subscriber('/rampreconf_enable', Bool, self.enable_callback)

        # Dynamic Reconfigure client
        self.reconfigure_client = dynamic_reconfigure.client.Client('move_base/DWAPlannerROS')

    def pose_callback(self, pose_msg):
        self.current_pose = pose_msg

        if not self.enable_reconfiguration:  # Check if reconfiguration is enabled
            if self.reconfiguration_done:
                self.reconfigure_xy_goal_tolerance(self.original_xy_goal_tolerance)
                self.reconfiguration_done = False  # Reset reconfiguration status
        else:
            inside_docking = self.check_is_inside_any_docking_area(pose_msg.position)
            
            if inside_docking:
                if not self.reconfiguration_done:  # Perform reconfiguration only once
                    rospy.loginfo("Robot is inside a docking area.")
                    self.reconfigure_xy_goal_tolerance(0.1)
                    self.reconfiguration_done = True  # Set reconfiguration status
            else:
                if self.reconfiguration_done:
                    rospy.loginfo("Robot is outside docking areas.")
                    self.reconfigure_xy_goal_tolerance(self.original_xy_goal_tolerance)
                    self.reconfiguration_done = False  # Reset reconfiguration status

    def enable_callback(self, enable_msg):
        self.enable_reconfiguration = enable_msg.data  # Update enable/disable status

    def reconfigure_xy_goal_tolerance(self, new_tolerance):
        rospy.loginfo("Reconfiguring xy_goal_tolerance to: {}".format(new_tolerance))
        params = {'xy_goal_tolerance': new_tolerance}
        self.reconfigure_client.update_configuration(params)

    def check_is_inside_any_docking_area(self, position):
        # Check if the given position is inside any of the docking areas
        for docking_area_polygon in docking_areas:
            if self.point_inside_polygon(position.x, position.y, docking_area_polygon):
                return True
        return False

    def point_inside_polygon(self, x, y, vertices):
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
        node = ToleranceReconfigureNode()
        rospy.spin()  # Process incoming messages
    except rospy.ROSInterruptException:
        pass
