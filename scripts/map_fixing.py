#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TransformStamped, Point
from visualization_msgs.msg import Marker
import numpy as np
import scipy.linalg
import tf
import tf2_ros
from copy import deepcopy
from collections import deque
from ekf import SLAM_EKF
from ExtractLines import ExtractLines
from maze_sim_parameters import LineExtractionParams, NoiseParams, ARENA, ArenaParams

def get_yaw_from_quaternion(quat):
    return tf.transformations.euler_from_quaternion([quat.x,
                                                     quat.y,
                                                     quat.z,
                                                     quat.w])[2]

def create_transform_msg(translation, rotation, child_frame, base_frame, time=None):
    t = TransformStamped()
    t.header.stamp = time if time else rospy.Time.now()
    t.header.frame_id = base_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]
    return t

def line_endpoints_from_alpha_and_r(alpha, r, d = 100.0):
    return ((r*np.cos(alpha) - d*np.sin(alpha), r*np.sin(alpha) + d*np.cos(alpha)),
            (r*np.cos(alpha) + d*np.sin(alpha), r*np.sin(alpha) - d*np.cos(alpha)))

class EKF_SLAM_Visualizer:

    def __init__(self):
        rospy.init_node('turtlebot_mapping')
        
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
                self.raw_base_to_camera = self.tfBuffer.lookup_transform("base_footprint", "base_scan", rospy.Time()).transform
                break
            except tf2_ros.LookupException:
                rate.sleep()
        rotation = self.raw_base_to_camera.rotation
        translation = self.raw_base_to_camera.translation
        tf_theta = get_yaw_from_quaternion(rotation)
        self.base_to_camera = [translation.x,
                               translation.y,
                               tf_theta]

        ## Colocate the `ground_truth` and `base_footprint` frames for visualization purposes
        tf2_ros.StaticTransformBroadcaster().sendTransform(
            create_transform_msg((0,0,0), (0,0,0,1), "ground_truth", "base_footprint")
        )

        ## Initial state for EKF
        self.EKF = None
        self.EKF_time = None
        self.current_control = np.zeros(2)
        self.latest_pose = None
        self.latest_pose_time = None
        self.controls = deque()
        self.scans = deque()


        N_map_lines = ArenaParams.shape[1]
        self.x0_map = ArenaParams.T.flatten()
        self.x0_map[4:] = self.x0_map[4:] + np.vstack((NoiseParams["std_alpha"]*np.random.randn(N_map_lines-2),    # first two lines fixed
                                                       NoiseParams["std_r"]*np.random.randn(N_map_lines-2))).T.flatten()
        self.P0_map = np.diag(np.concatenate((np.zeros(4),
                              np.array([[NoiseParams["std_alpha"]**2 for i in range(N_map_lines-2)],
                                        [NoiseParams["std_r"]**2 for i in range(N_map_lines-2)]]).T.flatten())))

        ## Set up publishers and subscribers
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.control_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.state_callback)
        self.ground_truth_ct = 0

        self.ground_truth_map_pub = rospy.Publisher("ground_truth_map", Marker, queue_size=10)
        self.ground_truth_map_marker = Marker()
        self.ground_truth_map_marker.header.frame_id = "/world"
        self.ground_truth_map_marker.header.stamp = rospy.Time.now()
        self.ground_truth_map_marker.ns = "ground_truth"
        self.ground_truth_map_marker.type = 5    # line list
        self.ground_truth_map_marker.pose.orientation.w = 1.0
        self.ground_truth_map_marker.scale.x = .025
        self.ground_truth_map_marker.scale.y = .025
        self.ground_truth_map_marker.scale.z = .025
        self.ground_truth_map_marker.color.r = 0.0
        self.ground_truth_map_marker.color.g = 1.0
        self.ground_truth_map_marker.color.b = 0.0
        self.ground_truth_map_marker.color.a = 1.0
        self.ground_truth_map_marker.lifetime = rospy.Duration(1000)
        self.ground_truth_map_marker.points = sum([[Point(p1[0], p1[1], 0),
                                                    Point(p2[0], p2[1], 0)] for p1, p2 in ARENA], [])

        self.EKF_map_pub = rospy.Publisher("EKF_map", Marker, queue_size=10)
        self.EKF_map_marker = deepcopy(self.ground_truth_map_marker)
        self.EKF_map_marker.color.r = 1.0
        self.EKF_map_marker.color.g = 0.0
        self.EKF_map_marker.color.b = 0.0
        self.EKF_map_marker.color.a = 1.0



    def scan_callback(self, msg):
        if self.EKF:
            self.scans.append((msg.header.stamp,
                               np.array([i*msg.angle_increment + msg.angle_min for i in range(len(msg.ranges))]),
                               np.array(msg.ranges)))

    def control_callback(self, msg):
        if self.EKF:
            self.controls.append((rospy.Time.now(), np.array([msg.linear.x, msg.angular.z]) + .1*np.random.randn(2)))

    def state_callback(self, msg):
        self.ground_truth_ct = self.ground_truth_ct + 1
        # `rostopic hz /gazebo/model_states` = 1000; let's broadcast the transform at 20Hz to reduce lag
        if self.ground_truth_ct % 50 == 0:
            self.latest_pose_time = rospy.Time.now()
            self.latest_pose = msg.pose[msg.name.index("turtlebot3_burger")]
            self.tfBroadcaster.sendTransform(create_transform_msg(
                (self.latest_pose.position.x, self.latest_pose.position.y, 0),
                (self.latest_pose.orientation.x, self.latest_pose.orientation.y, self.latest_pose.orientation.z, self.latest_pose.orientation.w),
                "base_footprint", "world", self.latest_pose_time)
            )

    def run(self):
        rate = rospy.Rate(100)

        while not self.latest_pose:
            rate.sleep()

        x0_pose = np.array([self.latest_pose.position.x,
                            self.latest_pose.position.y,
                            get_yaw_from_quaternion(self.latest_pose.orientation)])
        P0_pose = NoiseParams["P0"]
        self.EKF_time = self.latest_pose_time
        self.EKF = SLAM_EKF(np.concatenate((x0_pose, self.x0_map)), scipy.linalg.block_diag(P0_pose, self.P0_map),
                            NoiseParams["Q"], self.base_to_camera, 2*NoiseParams["g"])
        self.OLC = SLAM_EKF(np.concatenate((x0_pose, self.x0_map)), scipy.linalg.block_diag(P0_pose, self.P0_map),
                            NoiseParams["Q"], self.base_to_camera, 2*NoiseParams["g"])

        while True:
            if not self.scans:
                rate.sleep()
                continue

            self.EKF_map_marker.points = []
            for j in range((self.EKF.x.size - 3)/2):
                p1, p2 = line_endpoints_from_alpha_and_r(self.EKF.x[3+2*j], self.EKF.x[3+2*j+1])
                self.EKF_map_marker.points.extend([Point(p1[0], p1[1], 0), Point(p2[0], p2[1], 0)])
            self.ground_truth_map_pub.publish(self.ground_truth_map_marker)
            self.EKF_map_pub.publish(self.EKF_map_marker)

            while self.controls and self.controls[0][0] <= self.scans[0][0]:
                next_timestep, next_control = self.controls.popleft()
                if next_timestep < self.EKF_time:    # guard against time weirdness (msgs out of order)
                    continue
                self.EKF.transition_update(self.current_control,
                                           next_timestep.to_time() - self.EKF_time.to_time())
                self.OLC.transition_update(self.current_control,
                                           next_timestep.to_time() - self.EKF_time.to_time())
                self.EKF_time, self.current_control = next_timestep, next_control
                self.tfBroadcaster.sendTransform(create_transform_msg(
                    (self.EKF.x[0], self.EKF.x[1], 0),
                    tf.transformations.quaternion_from_euler(0, 0, self.EKF.x[2]),
                    "EKF", "world", self.EKF_time)
                )
                self.tfBroadcaster.sendTransform(create_transform_msg(
                    (self.OLC.x[0], self.OLC.x[1], 0),
                    tf.transformations.quaternion_from_euler(0, 0, self.OLC.x[2]),
                    "open_loop", "world", self.EKF_time)
                )
            
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
    vis = EKF_SLAM_Visualizer()
    vis.run()
