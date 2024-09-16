#!/usr/bin/env python3

import rospy
import dynamic_reconfigure.client
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

## test in simulation
# LG_dock_area_polygon = [(0.0, 0.0), (0.0, 3.0), (3.0, 3.0), (3.0, 0.0)]
# FiveF_dock_area_polygon = [(0.0, 0.0), (0.0, 3.0), (3.0, 3.0), (3.0, 0.0)]
LG_dock_area_polygon = [(-2.3715, 47.51), (-1.5262, 47.488), (-1.47, 56.148), (-2.1851, 56.092)]
FiveF_dock_area_polygon = [(1.4971, -19.281), (2.189, -19.467), (2.0321, -14.916), (1.4992, -14.899)]

## footprint
small_footprint  = [[0.15, -0.40], [0.15, -0.31], [0.15, 0.31], [0.15, 0.40], [-0.97, 0.40], [-1.27, 0.10], [-1.27, -0.10], [-0.97, -0.40]]
big_footprint = [[0.31,-0.5],[0.5,-0.31],[0.5,0.31],[0.31,0.5],[-0.97,0.5],[-1.27,0.42],[-1.27,-0.42],[-0.97,-0.5]]
# big_footprint = [[0.0,-0.6],[0.31,-0.5],[0.5,-0.31],[0.5,0.31],[0.31,0.5],[0.0,0.6],[-0.97,0.5],[-1.27,0.42],[-1.27,-0.42],[-0.97,-0.5]]

class DockReconfigureNode:
    def __init__(self):
        rospy.init_node('dock_reconfigure_node')

        self.current_pose = None
        self.inside = False
        self.reconfiguration_done = False  # Track if reconfiguration has been done
        self.enable_reconfiguration = True  # Track enable/disable status

        rospy.loginfo("Dock reconfigure node started")

        # Subscribe to robot's pose
        rospy.Subscriber('robot_pose', Pose, self.pose_callback)

        # Subscribe to reconfiguration enable/disable topic
        rospy.Subscriber('/rampreconf_enable', Bool, self.enable_callback)

        # Dynamic Reconfigure client
        self.global_reconfigure_client = dynamic_reconfigure.client.Client('/move_base/global_costmap')
        self.local_reconfigure_client = dynamic_reconfigure.client.Client('/move_base/local_costmap')

    def pose_callback(self, pose_msg):
        self.current_pose = pose_msg
        if not self.enable_reconfiguration:
            if self.check_is_inside_5f_dock_area(pose_msg.position):
                if not self.reconfiguration_done:
                    rospy.loginfo("Robot is close to 5F docking area.")
                    # Reconfigure footprint for local and global costmaps
                    self.reconfigure_footprint(small_footprint)
                    self.reconfiguration_done = True
            else:
                if self.reconfiguration_done:
                    rospy.loginfo("Robot is outside 5F docking area.")
                    # Reset the footprint to default
                    self.reconfigure_footprint(big_footprint)
                    self.reconfiguration_done = False
        else:
            if self.check_is_inside_lg_dock_area(pose_msg.position):
                if not self.reconfiguration_done:
                    rospy.loginfo("Robot is close to LG docking area.")
                    # Reconfigure footprint for local and global costmaps
                    self.reconfigure_footprint(small_footprint)
                    self.reconfiguration_done = True
            else:
                if self.reconfiguration_done:
                    rospy.loginfo("Robot is outside LG docking area.")
                    # Reset the footprint to default
                    self.reconfigure_footprint(big_footprint)
                    self.reconfiguration_done = False

    def enable_callback(self, enable_msg):
        self.enable_reconfiguration = enable_msg.data

    def reconfigure_footprint(self, new_footprint):
        rospy.loginfo("Reconfiguring footprint to: {}".format(new_footprint))
        params = {'footprint': new_footprint}
        self.global_reconfigure_client.update_configuration(params)
        self.local_reconfigure_client.update_configuration(params)

    def check_is_inside_lg_dock_area(self, position):
        x, y = position.x, position.y
        inside = self.point_inside_polygon(x, y, LG_dock_area_polygon)
        return inside
    
    def check_is_inside_5f_dock_area(self, position):
        x, y = position.x, position.y
        inside = self.point_inside_polygon(x, y, FiveF_dock_area_polygon)
        return inside
    
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
        node = DockReconfigureNode()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
