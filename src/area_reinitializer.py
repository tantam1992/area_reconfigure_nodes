#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from std_msgs.msg import Bool

# Define the areas as polygons
LG_areas = [
    # T
    [(-1.96, 56.27), (-4.32, 56.25), (-4.30, 57.03), (-1.44, 57.08)],
    # clean
    [(-15.38, 56.38), (-16.33, 56.27), (-13.59, 59.29), (-14.84, 59.20)],
    # narrow corridor
    [(-9.55, 56.38), (-10.44, 56.27), (-10.37, 59.29), (-9.60, 59.20)],
    # corridor
    [(-4.59, 38.73), (-1.11, 38.72), (-1.15, 40.17), (-4.58, 40.39)],
    # corridor door
    [(-4.38, 19.18), (-1.00, 19.28), (-0.97, 20.33), (-4.19, 20.37)],
    # outside 
    [(-5.56, 15.84), (-4.92, 15.77), (-4.78, 19.12), (-5.55, 19.21)],
    # Ramp Button
    [(-8.06, 9.52), (-5.04, 9.44), (-4.97, 10.08), (-8.08, 10.05)],
    # Ramp Top
    [(-4.57, 1.09), (-7.82, 0.93), (-7.74, 0.48), (-4.59, 0.60)],
    # After lift door
    [(-4.16, -0.65), (-4.90, -0.49), (-4.49, -3.49), (-3.78, -3.47)]
]

FF_areas = [
    # lift door
    [(-4.64, -6.85), (-1.81, -7.00), (-1.78, -6.50), (-5.18, -6.25)],
    # DSC door
    [(-7.65, -9.51), (-7.19, -9.53), (-7.24, -6.95), (-8.01, -6.85)],
    # corridor door
    [(-0.86, -9.40), (-0.97, -6.74), (-0.68, -6.67), (-0.79, -9.27)],
    # corridor 
    [(-0.75, -11.37), (2.35, -11.35), (2.36, -10.421), (-0.75, -10.58)]
]

class AreaReinitializerNode:
    def __init__(self):
        rospy.init_node('area_reinitializer_node')

        self.current_pose = None
        self.reinitialization_done = False  # Track if reconfiguration has been done
        self.enable_reconfiguration = True  # Track enable/disable status

        rospy.loginfo("New Area reinitializer node started")

        # Subscribe to robot's pose
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        # Subscribe to reconfiguration enable/disable topic
        rospy.Subscriber('/rampreconf_enable', Bool, self.enable_callback)

        # Publish initial pose
        self.initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    def pose_callback(self, pose_msg):
        self.current_pose = pose_msg
        if not self.enable_reconfiguration:
            self.inside_areas = [False] * len(FF_areas)
            for i, area in enumerate(FF_areas):
                if self.check_is_inside_area(self.current_pose.pose.pose.position, area):
                    self.inside_areas[i] = True
                else:
                    self.inside_areas[i] = False

            if any(self.inside_areas):
                if not self.reinitialization_done:
                    rospy.loginfo("Robot is in the FF reinitialization area.")
                    self.publish_initial_pose()
                    self.reinitialization_done = True
            else:
                if self.reinitialization_done:
                    rospy.loginfo("Robot is out of the FF reinitialization area.")
                    self.reinitialization_done = False

        else:
            self.inside_areas = [False] * len(LG_areas)
            for i, area in enumerate(LG_areas):
                if self.check_is_inside_area(self.current_pose.pose.pose.position, area):
                    self.inside_areas[i] = True
                else:
                    self.inside_areas[i] = False

            if any(self.inside_areas):
                if not self.reinitialization_done:
                    rospy.loginfo("Robot is in the LG reinitialization area.")
                    self.publish_initial_pose()
                    self.reinitialization_done = True
            else:
                if self.reinitialization_done:
                    rospy.loginfo("Robot is out of the LG reinitialization area.")
                    self.reinitialization_done = False

    def enable_callback(self, enable_msg):
        self.enable_reconfiguration = enable_msg.data  # Update enable/disable status

    def publish_initial_pose(self):
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.stamp = rospy.Time.now()
        initial_pose_msg.header.frame_id = "map"  # Adjust frame_id if necessary
        initial_pose_msg.pose.pose.position = self.current_pose.pose.pose.position
        initial_pose_msg.pose.pose.orientation = self.current_pose.pose.pose.orientation
        initial_pose_msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
        self.initialpose_pub.publish(initial_pose_msg)
        rospy.loginfo("Pose Reinitialized")

    def check_is_inside_area(self, position, vertices):
        # Check if the given position is inside the area defined by vertices
        x, y = position.x, position.y
        inside = self.point_inside_polygon(x, y, vertices)
        return inside

    def point_inside_polygon(self, x, y, vertices):
        # Check if the given point is inside the polygon defined by vertices
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
        node = AreaReinitializerNode()
        rospy.spin()  # Process incoming messages

    except rospy.ROSInterruptException:
        pass
