#!/usr/bin/env python3

import rospy
import dynamic_reconfigure.client
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

# example
narrow_area_polygon = [(0.0, 0.0), (0.0, 3.0), (3.0, 3.0), (3.0, 0.0)]
# ramp_area_polygon = [(-8.5, 10.7), (-4.1, 10.7), (-4.1, 1.4), (-8.5, 1.4)]

class NarrowReconfigureNode:
    def __init__(self):
        rospy.init_node('narrow_reconfigure_node')

        self.current_pose = None
        self.inside = False
        self.reconfiguration_done = False  # Track if reconfiguration has been done
        self.enable_reconfiguration = True  # Track enable/disable status

        rospy.loginfo("narrow reconfigure node started")

        # Subscribe to robot's pose
        rospy.Subscriber('robot_pose', Pose, self.pose_callback)

        # Subscribe to reconfiguration enable/disable topic
        rospy.Subscriber('/rampreconf_enable', Bool, self.enable_callback)

        # Dynamic Reconfigure client
        self.obstacle_layer_reconfigure_client = dynamic_reconfigure.client.Client('move_base/global_costmap/obstacle_layer', timeout=30)
        self.rgbd_obstacle_layer_reconfigure_client = dynamic_reconfigure.client.Client('move_base/global_costmap/rgbd_obstacle_layer', timeout=30)

    def pose_callback(self, pose_msg):
        self.current_pose = pose_msg
        # rospy.loginfo("The current pose is : \n {}".format(pose_msg))
        if not self.enable_reconfiguration:  # Check if reconfiguration is enabled
            if self.reconfiguration_done:
                self.reconfigure_obstacle_layer(True)
                self.reconfiguration_done = False  # Reset reconfiguration status
        else:
            if self.check_is_inside_narrow_area(pose_msg.position):
                if not self.reconfiguration_done:  # Perform reconfiguration only once
                    rospy.loginfo("Robot is inside narrow area.")
                    self.reconfigure_obstacle_layer(False)
                    self.reconfiguration_done = True  # Set reconfiguration status
            else:
                if self.reconfiguration_done:
                    rospy.loginfo("Robot is outside narrow area.")
                    self.reconfigure_obstacle_layer(True)
                    self.reconfiguration_done = False  # Reset reconfiguration status

    def enable_callback(self, enable_msg):
        self.enable_reconfiguration = enable_msg.data  # Update enable/disable status

    def reconfigure_obstacle_layer(self, enable):
        rospy.loginfo("Reconfiguring Obstacle layer: {}".format(enable))
        params = {'enabled': enable}
        self.obstacle_layer_reconfigure_client.update_configuration(params)
        self.rgbd_obstacle_layer_reconfigure_client.update_configuration(params)

    def check_is_inside_narrow_area(self, position):
        # Check if the given position is inside the ramp area polygon
        x, y = position.x, position.y
        inside = self.point_inside_polygon(x, y, narrow_area_polygon)
        # rospy.loginfo("Inside ramp area: {}".format(inside))
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
        node = NarrowReconfigureNode()
        rospy.spin()  # Process incoming messages

    except rospy.ROSInterruptException:
        pass
