#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray, Pose2D
from std_msgs.msg import Float32MultiArray, String
import tf
import numpy as np
from numpy import linalg
from utils import wrapToPi

# control gains
K1 = 0.4
K2 = 0.8
K3 = 0.8

# tells the robot to stay still
# if it doesn't get messages within that time period
TIMEOUT = 1.0

# maximum velocity
V_MAX = 0.2

# maximim angular velocity
W_MAX = 1

# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup (hw2+)
use_gazebo = rospy.get_param("sim")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")


print "pose_controller settings:\n"
print "use_gazebo = %s\n" % use_gazebo
print "mapping = %s\n" % mapping

class PoseController:

    def __init__(self):
        rospy.init_node('turtlebot_pose_controller', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # current state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # goal state
        self.x_g = 0.0
        self.y_g = 0.0
        self.theta_g = 0.0

        

        # time last pose command was received
        self.cmd_pose_time = rospy.get_rostime()
        # if using gazebo, then subscribe to model states
        if use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
 
        self.trans_listener = tf.TransformListener()


        ######### YOUR CODE HERE ############
        # create a subscriber that receives Pose2D messages and
        # calls cmd_pose_callback. It should subscribe to '/cmd_pose'




        ######### END OF YOUR CODE ##########



    def gazebo_callback(self, data):
        if "turtlebot3_burger" in data.name:
            pose = data.pose[data.name.index("turtlebot3_burger")]
            twist = data.twist[data.name.index("turtlebot3_burger")]
            self.x = pose.position.x
            self.y = pose.position.y
            quaternion = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.theta = euler[2]

    def cmd_pose_callback(self, data):
        ######### YOUR CODE HERE ############
        # fill out cmd_pose_callback




        ######### END OF YOUR CODE ##########
        self.cmd_pose_time = rospy.get_rostime()


    def get_ctrl_output(self):
        """ runs a simple feedback pose controller """
        if (rospy.get_rostime().to_sec()-self.cmd_pose_time.to_sec()) < TIMEOUT:
            # if you are not using gazebo, your need to use a TF look-up to find robot's states
            # relevant for hw 3+
            if not use_gazebo:
                try:
                    origin_frame = "/map" if mapping else "/odom"
                    (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                    self.x = translation[0]
                    self.y = translation[1]
                    euler = tf.transformations.euler_from_quaternion(rotation)
                    self.theta = euler[2]
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    pass

            ######### YOUR CODE HERE ############
            # robot's state is self.x, self.y, self.theta
            # robot's desired state is self.x_g, self.y_g, self.theta_g
            # fill out cmd_x_dot = ... cmd_theta_dot = ...





            ######### END OF YOUR CODE ##########

        else:
            # haven't received a command in a while so stop
            rospy.loginfo("Pose controller TIMEOUT: commanding zero controls")
            cmd_x_dot = 0
            cmd_theta_dot = 0


        cmd = Twist()
        cmd.linear.x = cmd_x_dot
        cmd.angular.z = cmd_theta_dot
        return cmd

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            ctrl_output = self.get_ctrl_output()
            self.pub.publish(ctrl_output)
            rate.sleep()

if __name__ == '__main__':

    pctrl = PoseController()
    pctrl.run()
