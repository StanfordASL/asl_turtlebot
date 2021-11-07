#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8

MIN_WIDTH = 0
MAX_WIDTH = 100

if __name__ == '__main__':
    rospy.init_node('gripper_publisher')
    pub = rospy.Publisher('gripper_pose_remote', Int8, queue_size=1)

    while not rospy.is_shutdown():
        str_input = input("Enter gripper width (0-100): ")
        try:
            width = int(str_input)
            width = min(MAX_WIDTH, max(MIN_WIDTH, width))
            print("Sending {}.".format(width))
            pub.publish(width)
        except:
            print("Enter a valid number (received {}).".format(str_input))
