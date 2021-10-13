#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

def callback(data):
    #import pdb;pdb.set_trace()
    rospy.loginfo(rospy.get_caller_id() + "Receiving Odometry results: {}".format(data.pose))

def subscriber():
    rospy.init_node('odom_subscriber_node', anonymous=True)
    rospy.Subscriber('/odom', Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()
