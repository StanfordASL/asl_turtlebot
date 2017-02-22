#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import tf
import tf2_ros
from collections import deque
from ekf import Localization_EKF
from ExtractLines import ExtractLines
from maze_sim_parameters import LineExtractionParams, NoiseParams, MapParams

def get_yaw_from_quaternion(quat):
    return tf.transformations.euler_from_quaternion([quat.x,
                                                     quat.y,
                                                     quat.z,
                                                     quat.w])[2]

class LocalizationVisualizer:

    def __init__(self):
        rospy.init_node('turtlebot_localization')
        
        ## Use simulation time (i.e. get time from rostopic /clock)
        rospy.set_param('use_sim_time', 'true')
        rate = rospy.Rate(10)
        while rospy.Time.now() == rospy.Time(0):
            rate.sleep()

        ## Get transformation of camera frame with respect to the base frame
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        while True:
            try:
                # notably camera_link and not camera_depth_frame below, not sure why
                self.raw_base_to_camera = self.tfBuffer.lookup_transform("base_footprint", "camera_link", rospy.Time()).transform
                break
            except tf2_ros.LookupException:
                rate.sleep()
        rotation = self.raw_base_to_camera.rotation
        translation = self.raw_base_to_camera.translation
        tf_theta = get_yaw_from_quaternion(rotation)
        self.base_to_camera = [translation.x,
                               translation.y,
                               tf_theta]

        ## Initial state for EKF
        self.EKF = None
        self.EKF_time = None
        self.current_control = np.zeros(2)
        self.latest_pose = None
        self.latest_pose_time = None
        self.controls = deque()
        self.scans = deque()

        ## Set up publishers and subscribers
        self.tfBroadcaster = tf.TransformBroadcaster()
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/cmd_vel_mux/input/teleop', Twist, self.control_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.state_callback)
        self.ground_truth_ct = 0

    def scan_callback(self, msg):
        if self.EKF:
            self.scans.append((msg.header.stamp,
                               np.array([i*msg.angle_increment + msg.angle_min for i in range(len(msg.ranges))]),
                               np.array(msg.ranges)))

    def control_callback(self, msg):
        if self.EKF:
            self.controls.append((rospy.Time.now(), np.array([msg.linear.x, msg.angular.z])))

    def state_callback(self, msg):
        self.ground_truth_ct = self.ground_truth_ct + 1
        # `rostopic hz /gazebo/model_states` = 1000; let's broadcast the transform at 20Hz to reduce lag
        if self.ground_truth_ct % 50 == 0:
            self.latest_pose_time = rospy.Time.now()
            self.latest_pose = msg.pose[msg.name.index("mobile_base")]
            self.tfBroadcaster.sendTransform((self.latest_pose.position.x, self.latest_pose.position.y, 0),
                                             (self.latest_pose.orientation.x,
                                              self.latest_pose.orientation.y,
                                              self.latest_pose.orientation.z,
                                              self.latest_pose.orientation.w),
                                             self.latest_pose_time, "ground_truth", "world")

    def run(self):
        rate = rospy.Rate(100)

        while not self.latest_pose:
            rate.sleep()

        x0 = np.array([self.latest_pose.position.x,
                       self.latest_pose.position.y,
                       get_yaw_from_quaternion(self.latest_pose.orientation)])
        self.EKF_time = self.latest_pose_time
        self.EKF = Localization_EKF(x0, NoiseParams["P0"], NoiseParams["Q"],
                                    MapParams, self.base_to_camera, NoiseParams["g"])
        self.OLC = Localization_EKF(x0, NoiseParams["P0"], NoiseParams["Q"],
                                    MapParams, self.base_to_camera, NoiseParams["g"])

        while True:
            if not self.scans:
                rate.sleep()
                continue

            while self.controls and self.controls[0][0] <= self.scans[0][0]:
                next_timestep, next_control = self.controls.popleft()
                if next_timestep < self.EKF_time:    # guard against time weirdness (msgs out of order)
                    continue
                self.EKF.transition_update(self.current_control,
                                           next_timestep.to_time() - self.EKF_time.to_time())
                self.OLC.transition_update(self.current_control,
                                           next_timestep.to_time() - self.EKF_time.to_time())
                self.EKF_time, self.current_control = next_timestep, next_control
                self.tfBroadcaster.sendTransform((self.EKF.x[0], self.EKF.x[1], 0),
                                                 tf.transformations.quaternion_from_euler(0, 0, self.EKF.x[2]),
                                                 self.EKF_time, "EKF", "world")
                self.tfBroadcaster.sendTransform((self.OLC.x[0], self.OLC.x[1], 0),
                                                 tf.transformations.quaternion_from_euler(0, 0, self.OLC.x[2]),
                                                 self.EKF_time, "open_loop", "world")
            
            scan_time, theta, rho = self.scans.popleft()
            if scan_time < self.EKF_time:
                continue
            self.EKF.transition_update(self.current_control,
                                       scan_time.to_time() - self.EKF_time.to_time())
            self.OLC.transition_update(self.current_control,
                                       scan_time.to_time() - self.EKF_time.to_time())
            self.EKF_time = scan_time
            alpha, r, C_AR, _, _ = ExtractLines(theta, rho,
                                                LineExtractionParams,
                                                NoiseParams["var_theta"],
                                                NoiseParams["var_rho"])
            Z = np.vstack((alpha, r))
            self.EKF.measurement_update(Z, C_AR)
            while len(self.scans) > 1:    # keep only the last element in the queue, if we're falling behind
                self.scans.popleft()

if __name__ == '__main__':
    vis = LocalizationVisualizer()
    vis.run()
