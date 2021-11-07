#!/usr/bin/env python3
import sys, os

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray, Pose2D
from std_msgs.msg import Float32MultiArray, String
import tf
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), "utils"))
from controllers.P2_pose_stabilization import PoseController


class PoseControllerParams:
    def __init__(self, verbose=False):
        # If sim is True (i.e. using gazebo), we want to subscribe to
        # /gazebo/model_states. Otherwise, we will use a TF lookup.
        self.use_gazebo = rospy.get_param("sim")

        # If using gmapping, we will have a map frame. Otherwise, it will be odom frame.
        self.mapping = rospy.get_param("map")

        # Control gains
        self.k1 = rospy.get_param("~k1", 0.4)
        self.k2 = rospy.get_param("~k2", 0.8)
        self.k3 = rospy.get_param("~k3", 0.8)

        # Maximum velocity
        self.v_max = rospy.get_param("~v_max", 0.2)

        # Maximum angular velocity
        self.om_max = rospy.get_param("~om_max", 1.0)

        # Tells the robot to stay still if it doesn't get messages for TIMEOUT seconds.
        self.timeout = rospy.get_param("~timeout", 1.0)

        if verbose:
            print("PoseControllerParams:")
            print("    use_gazebo = {}".format(self.use_gazebo))
            print("    mapping = {}".format(self.mapping))
            print(
                "    k1, k2, k3 = {}, {}, {}".format(self.k1, self.k2, self.k3)
            )
            print("    v_max, om_max = {}, {}".format(self.v_max, self.om_max))
            print("    timeout = {}".format(self.timeout))


class PoseControllerNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("turtlebot_pose_controller", anonymous=True)
        self.params = PoseControllerParams(verbose=True)

        # Current state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Controller from HW1 P2
        self.controller = PoseController(
            self.params.k1,
            self.params.k2,
            self.params.k3,
            self.params.v_max,
            self.params.om_max,
        )

        self.controller.load_goal(self.x, self.y, self.theta)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Time last pose command was received
        self.cmd_pose_time = rospy.get_rostime()

        # If using gazebo, we have access to perfect state
        if self.params.use_gazebo:
            rospy.Subscriber(
                "/gazebo/model_states", ModelStates, self.gazebo_callback
            )
        self.trans_listener = tf.TransformListener()

        ########## Code starts here ##########
        # TODO: Create a subscriber to the '/cmd_pose' topic that receives
        #       Pose2D messages and calls cmd_pose_callback.

        ########## Code ends here ##########

    def gazebo_callback(self, msg):
        if "turtlebot3_burger" not in msg.name:
            return

        pose = msg.pose[msg.name.index("turtlebot3_burger")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]

    def cmd_pose_callback(self, msg):
        ########## Code starts here ##########
        # TODO: Update the goal pose in the pose controller.

        ########## Code ends here ##########

        # Record time of pose update
        self.cmd_pose_time = rospy.get_rostime()

    def compute_control(self):
        if (
            rospy.get_rostime().to_sec() - self.cmd_pose_time.to_sec()
            > self.params.timeout
        ):
            # haven't received a command in a while so stop
            rospy.loginfo("Pose controller TIMEOUT: commanding zero controls")
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            return cmd

        # if you are not using gazebo, your need to use a TF look-up to find
        # robot's states relevant for hw 3+
        if not self.params.use_gazebo:
            try:
                origin_frame = "/map" if self.params.mapping else "/odom"
                translation, rotation = self.trans_listener.lookupTransform(
                    origin_frame, "/base_footprint", rospy.Time(0)
                )
                self.x, self.y = translation[0], translation[1]
                self.theta = tf.transformations.euler_from_quaternion(rotation)[
                    2
                ]
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                pass

        ######### YOUR CODE HERE ############
        # TODO: Use your pose controller to compute controls (V, om) given the
        #       robot's current state.

        ######### END OF YOUR CODE ##########

        cmd = Twist()
        cmd.linear.x = V
        cmd.angular.z = om
        return cmd

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            ctrl_output = self.compute_control()
            self.pub.publish(ctrl_output)
            rate.sleep()


if __name__ == "__main__":
    pctrl = PoseControllerNode()
    pctrl.run()
