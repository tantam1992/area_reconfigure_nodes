#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose

# Constants
DOOR_POLYGON = [(-24.54, -4.19), (-25.68, 1.61), (-30.62, 1.14), (-29.60, -4.99)]

class ACBLGLiftScanSwitcher:
    def __init__(self):
        rospy.init_node('ACB_LG_Lift_scan_switch', anonymous=True)

        # Subscribers
        self.pose_sub = rospy.Subscriber('robot_pose', Pose, self.pose_callback)
        self.lidar_top_sub = rospy.Subscriber('/lidarTop/scan', LaserScan, self.lidar_top_callback)
        self.livox_back_sub = rospy.Subscriber('/livox_back/scan', LaserScan, self.livox_back_callback)

        # Publisher
        self.scan_pub = rospy.Publisher('/ACB_LG_Lift/scan', LaserScan, queue_size=10)

        # State variables
        self.current_pose = None
        self.lidar_top_scan = None
        self.livox_back_scan = None

        rospy.loginfo("ACB_5F_Lift scan switch node started") # Add a start message

    def pose_callback(self, pose_msg):
        self.current_pose = pose_msg
        self.publish_scan()

    def lidar_top_callback(self, msg):
        self.lidar_top_scan = msg
        self.publish_scan()

    def livox_back_callback(self, msg):
        self.livox_back_scan = msg
        self.publish_scan()

    def publish_scan(self):
        if not self.current_pose or not self.lidar_top_scan or not self.livox_back_scan:
            rospy.logwarn("Missing pose or scan data, cannot publish /ACB_LG_Lift/scan") # Use logwarn
            return

        if self.is_in_door_area():
            self.scan_pub.publish(self.livox_back_scan)
            rospy.loginfo("Publishing /livox_back/scan to /ACB_LG_Lift/scan") # Add log info
        else:
            self.scan_pub.publish(self.lidar_top_scan)
            rospy.loginfo("Publishing /lidarTop/scan to /ACB_LG_Lift/scan") # Add log info

    def is_in_door_area(self):
        if not self.current_pose:
            return False

        robot_pos = self.current_pose.position
        return self.point_inside_polygon(robot_pos.x, robot_pos.y, DOOR_POLYGON)

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
        switcher = ACBLGLiftScanSwitcher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass