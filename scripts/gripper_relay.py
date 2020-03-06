#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8

class GripperRelay:
    def __init__(self):
        rospy.init_node('gripper_relay')
        self.pub = rospy.Publisher('/gripper_pose', Int8, queue_size=1)
        self.sub = rospy.Subscriber('/gripper_pose_remote', Int8, self.gripper_callback)

    def gripper_callback(self, msg):
        self.pub.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    gr = GripperRelay()
    gr.run()
