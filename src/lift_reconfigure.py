#!/usr/bin/env python3

import rospy
import dynamic_reconfigure.client
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

# example
# ramp_area_polygon = [(0.0, 0.0), (0.0, 3.0), (3.0, 3.0), (3.0, 0.0)]
ramp_area_polygon = [(-8.5, 10.7), (-4.1, 10.7), (-4.1, 1.4), (-8.5, 1.4)]
LG_lift_area_polygon = [(0.07, 1.07), (-1.98, 0.93), (-1.86, -0.88), (0.43, -0.78)]
FiveF_lift_area_polygon = [(-2.87, 1.25), (-0.73, 1.51), (-0.98, -0.42), (-2.78, -0.47)]


class LiftReconfigureNode:
    def __init__(self):
        rospy.init_node('lift_reconfigure_node')

        self.current_pose = None
        self.ramp_inside = False
        self.lg_lift_inside = False
        self.fivef_lift_inside = False

        self.enable_reconfiguration = True  # Track enable/disable status

        rospy.loginfo("lift reconfigure node started")

        # Subscribe to reconfiguration enable/disable topic
        rospy.Subscriber('rampreconf_enable', Bool, self.enable_callback)

        # Subscribe to robot's pose
        rospy.Subscriber('robot_pose', Pose, self.pose_callback)

        # Dynamic Reconfigure client
        self.reconfigure_client = dynamic_reconfigure.client.Client('move_base/DWAPlannerROS')

    def pose_callback(self, pose_msg):
        self.current_pose = pose_msg
        rospy.loginfo("The current pose is : \n {}".format(pose_msg))
        # Implement logic to determine if the robot is inside the ramp area
        if self.enable_reconfiguration:  # Check if reconfiguration is enabled
            if self.check_is_inside_LG_lift_area(pose_msg.position):
                self.reconfigure_max_vel(0.45)
                self.reconfigure_min_vel(-0.3)
            else:
                self.reconfigure_max_vel(0.4)
                self.reconfigure_min_vel(-0.3)
        if not self.enable_reconfiguration:
            if self.check_is_inside_5F_lift_area(pose_msg.position):
                self.reconfigure_max_vel(0.45)
                self.reconfigure_min_vel(-0.3)
            else:
                self.reconfigure_max_vel(0.4)
                self.reconfigure_min_vel(-0.3)


    def enable_callback(self, enable_msg):
        self.enable_reconfiguration = enable_msg.data  # Update enable/disable status

    def reconfigure_max_vel(self, new_max_vel):
        rospy.loginfo("Reconfiguring max_vel_x to: {}".format(new_max_vel))
        params = {'max_vel_x': new_max_vel}
        self.reconfigure_client.update_configuration(params)

    def reconfigure_min_vel(self, new_min_vel):
        rospy.loginfo("Reconfiguring min_vel_x to: {}".format(new_min_vel))
        params = {'min_vel_x': new_min_vel}
        self.reconfigure_client.update_configuration(params)

    def check_is_inside_ramp_area(self, position):
        # Check if the given position is inside the ramp area polygon
        x, y = position.x, position.y
        ramp_inside = self.point_inside_polygon(x, y, ramp_area_polygon)
        rospy.loginfo("Inside ramp area: {}".format(ramp_inside))
        return ramp_inside

    def check_is_inside_LG_lift_area(self, position):
        # Check if the given position is inside the ramp area polygon
        x, y = position.x, position.y
        lg_lift_inside = self.point_inside_polygon(x, y, LG_lift_area_polygon)
        rospy.loginfo("Inside LG lift area: {}".format(lg_lift_inside))
        return lg_lift_inside

    def check_is_inside_5F_lift_area(self, position):
        # Check if the given position is inside the ramp area polygon
        x, y = position.x, position.y
        fiveF_lift_inside = self.point_inside_polygon(x, y, FiveF_lift_area_polygon)
        rospy.loginfo("Inside 5F lift area: {}".format(fiveF_lift_inside))
        return fiveF_lift_inside
    
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
        node = LiftReconfigureNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass