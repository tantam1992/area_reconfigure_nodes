#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

def ramp_reconf_enable_publisher():
    rospy.init_node('ramp_reconf_enable_publisher', anonymous=True)
    rate = rospy.Rate(1)  # Publish rate in Hz

    # Create a publisher for /rampreconf_enable topic
    ramp_reconf_enable_pub = rospy.Publisher('rampreconf_enable', Bool, queue_size=10)

    while not rospy.is_shutdown():
        # Create a Bool message
        enable_msg = Bool()
        enable_msg.data = True  # Set the enable/disable status here

        # Publish the message
        ramp_reconf_enable_pub.publish(enable_msg)

        # Sleep to maintain the desired publish rate
        rate.sleep()

if __name__ == '__main__':
    try:
        ramp_reconf_enable_publisher()
    except rospy.ROSInterruptException:
        pass