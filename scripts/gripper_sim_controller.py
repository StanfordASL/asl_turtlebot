#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8, Float64

pub_right = None
pub_left = None

GRIPPER_CLOSED = 0
GRIPPER_OPEN = 100
GRIPPER_WIDTH = 0.03

def gripper_command_callback(pose):
    global pub_right, pub_left

    gripper_width = GRIPPER_WIDTH / GRIPPER_OPEN * \
                    min(GRIPPER_OPEN, max(GRIPPER_CLOSED, pose.data))
    pub_right.publish(gripper_width)
    pub_left.publish(gripper_width)

if __name__ == '__main__':
    rospy.init_node('gripper_sim_controller')
    pub_right = rospy.Publisher('gripper/right_gripper_controller/command', Float64, queue_size=1)
    pub_left = rospy.Publisher('gripper/left_gripper_controller/command', Float64, queue_size=1)
    rospy.Subscriber('gripper_pose', Int8, gripper_command_callback)
    rospy.spin()
