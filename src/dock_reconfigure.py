#!/usr/bin/env python3

import rospy
import dynamic_reconfigure.client
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, String

# Constants
LG_ROTATE_AREA_POLYGON = [(-88.09, -1.78), (-89.62, -1.63), (-89.63, 2.84), (-88.32, 2.91)]
LG_DOCK_AREA_POLYGON = [(-75.65, -6.98), (-75.61, -6.45), (-84.18, -7.00), (-84.13, -7.79)]
FIVEF_DOCK_AREA_POLYGON = [(1.4971, -19.281), (2.189, -19.467), (2.0321, -14.916), (1.4992, -14.899)]

ROTATE_FOOTPRINT  =  [[0.31,-0.40] , [0.31 ,-0.31], [0.31 , 0.31], [0.31, 0.40 ], [-0.97, 0.40] , [-1.07, 0.10], [-1.07,-0.10], [-0.97,-0.40] ]
DOCK_FOOTPRINT    =  [[0.15,-0.40] , [0.15 ,-0.31], [0.15 , 0.31], [0.15, 0.40 ], [-0.97, 0.40] , [-1.07, 0.10], [-1.07,-0.10], [-0.97,-0.40] ]
SMALL_FOOTPRINT   =  [[0.31,-0.45] , [0.45 ,-0.31], [0.45 , 0.31], [0.31, 0.45 ], [-0.97, 0.40] , [-1.07, 0.10], [-1.07,-0.10], [-0.97,-0.40] ]
UNLOAD_FOOTPRINT  =  [[0.31,-0.495], [0.495,-0.31], [0.495, 0.31], [0.31, 0.495], [-0.97, 0.495], [-1.07, 0.10], [-1.07,-0.10], [-0.97,-0.495]]
BIG_FOOTPRINT     =  [[0.31,-0.495], [0.495,-0.31], [0.495, 0.31], [0.31, 0.495], [-0.97, 0.495], [-1.27, 0.42], [-1.27,-0.42], [-0.97,-0.495]]

FOLD_STATE_READY = "OPERATIONAL/READY"
FOLD_STATE_READY_PICKUP = "OPERATIONAL/READY_PICKUP"

class DockReconfigureNode:
    def __init__(self):
        rospy.init_node('dock_reconfigure_node')

        self.current_pose = None
        self.fold_state = "UNKNOWN"
        self.enable_reconfiguration = True
        self.dynamic_reconfigure_services_ready = False

        # Track the last footprint sent to avoid redundant reconfigs
        self.last_footprint = None

        rospy.loginfo("Dock reconfigure node started")

        self.fold_state_sub = rospy.Subscriber('/fold_state', String, self.fold_state_callback)
        self.pose_sub = rospy.Subscriber('robot_pose', Pose, self.pose_callback)
        self.enable_sub = rospy.Subscriber('/rampreconf_enable', Bool, self.enable_callback)

        self.global_reconfigure_client = None
        self.local_reconfigure_client = None

        self.wait_for_reconfigure_services()

    def wait_for_reconfigure_services(self):
        if self.dynamic_reconfigure_services_ready:
            return

        try:
            rospy.loginfo("Waiting for dynamic reconfigure services...")
            rospy.wait_for_service('/move_base/global_costmap/set_parameters')
            rospy.wait_for_service('/move_base/local_costmap/set_parameters')
            self.global_reconfigure_client = dynamic_reconfigure.client.Client('/move_base/global_costmap')
            self.local_reconfigure_client = dynamic_reconfigure.client.Client('/move_base/local_costmap')
            rospy.loginfo("Dynamic reconfigure services are ready.")
            self.dynamic_reconfigure_services_ready = True
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to connect to dynamic reconfigure services: {e}")
            rospy.signal_shutdown("Shutting down due to service connection failure.")

    def fold_state_callback(self, msg):
        self.fold_state = msg.data
        self.update_footprint()

    def pose_callback(self, pose_msg):
        self.current_pose = pose_msg
        self.update_footprint()

    def enable_callback(self, enable_msg):
        self.enable_reconfiguration = enable_msg.data
        self.update_footprint()

    def update_footprint(self):
        """Determine and set the correct footprint according to area and fold state."""
        if not self.current_pose or not self.dynamic_reconfigure_services_ready:
            return

        # Select dock area polygon based on enable flag
        if self.enable_reconfiguration:
            dock_polygon = LG_DOCK_AREA_POLYGON
            dock_area_name = "LG"
        else:
            dock_polygon = FIVEF_DOCK_AREA_POLYGON
            dock_area_name = "5F"

        rotate_polygon = LG_ROTATE_AREA_POLYGON
        robot_pos = self.current_pose.position
        in_dock_area = self.point_inside_polygon(robot_pos.x, robot_pos.y, dock_polygon)
        in_rotate_area = self.point_inside_polygon(robot_pos.x, robot_pos.y, rotate_polygon)

        # Choose correct footprint
        if in_dock_area:
            target_footprint = DOCK_FOOTPRINT
            rospy.loginfo(f"Robot is inside {dock_area_name} dock area, using DOCK_FOOTPRINT")
        elif in_rotate_area and self.fold_state == FOLD_STATE_READY:
            target_footprint = ROTATE_FOOTPRINT
            rospy.loginfo(f"Robot is inside LG rotate area, using ROTATE_FOOTPRINT")
        else:
            if self.fold_state == FOLD_STATE_READY:
                target_footprint = SMALL_FOOTPRINT
                rospy.loginfo("Robot is outside dock area and arms are CLOSED, using SMALL_FOOTPRINT")
            elif self.fold_state == FOLD_STATE_READY_PICKUP:
                target_footprint = UNLOAD_FOOTPRINT
                rospy.loginfo("Robot is outside dock area and arms are OPENED, using UNLOAD_FOOTPRINT")
            else:
                target_footprint = BIG_FOOTPRINT
                rospy.loginfo("Robot is outside dock area and arms are OPEN, using BIG_FOOTPRINT")

        # Only reconfigure if the footprint changes
        if self.last_footprint != target_footprint:
            self.reconfigure_footprint(target_footprint)
            self.last_footprint = target_footprint

    def reconfigure_footprint(self, new_footprint):
        if not self.dynamic_reconfigure_services_ready:
            rospy.logerr("Dynamic reconfigure services not available.  Skipping reconfiguration.")
            return

        rospy.loginfo("Reconfiguring footprint to: {}".format(new_footprint))
        params = {'footprint': new_footprint}
        try:
            self.global_reconfigure_client.update_configuration(params)
            self.local_reconfigure_client.update_configuration(params)
        except dynamic_reconfigure.DynamicReconfigureCallbackException as e:
            rospy.logerr(f"Failed to reconfigure footprint: {e}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

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