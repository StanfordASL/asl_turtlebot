#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject, DetectedObjectList
from std_msgs.msg import String
import tf
import numpy as np
from numpy import linalg
from scipy import ndimage
from utils import wrapToPi
from planners import AStar, compute_smoothed_traj
from grids import StochOccupancyGrid2D
import scipy.interpolate
import matplotlib.pyplot as plt
from controllers import PoseController, TrajectoryTracker, HeadingController
from enum import Enum
from visualization_msgs.msg import Marker

from dynamic_reconfigure.server import Server
from asl_turtlebot.cfg import NavigatorConfig

import pdb

# state machine modes, not all implemented
class Mode(Enum):
    IDLE = 0
    ALIGN = 1
    TRACK = 2
    PARK = 3

class Navigator:
    """
    This node handles point to point turtlebot motion, avoiding obstacles.
    It is the sole node that should publish to cmd_vel
    """
    def __init__(self):
        rospy.init_node('turtlebot_navigator', anonymous=True)
        self.mode = Mode.IDLE

        #delivery lists
        self.delivery_req_list = []
        self.ifdelivery  = False
        self.detected_objects_names = []
        self.detected_objects = []
        self.marker_dict = {}
        self.objectname_markerLoc_dict = {}

        #stop sign params
        self.stop_min_dist = 0.5
        self.stop_time = 3.
        self.crossing_time = 3.

        # current state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # goal state
        self.x_g = None
        self.y_g = None
        self.theta_g = None

        # initial state
        self.x_init = rospy.get_param("~x_pos",3.15)
        self.y_init = rospy.get_param("~y_pos",1.6)
        self.z_init = rospy.get_param("~z_pos",0.0)
        self.th_init = 0.0

        # map parameters
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0,0]
        self.map_probs = []
        self.occupancy = None
        self.occupancy_updated = False
        self.map_threshold = 50 

        # plan parameters
        self.plan_resolution =  0.1/2.0
        self.plan_horizon = 4.0

        # time when we started following the plan
        self.current_plan_start_time = rospy.get_rostime()
        self.current_plan_duration = 0
        self.plan_start = [0.,0.]
        
        # Robot limits
        self.v_max = 0.2    # maximum velocity (orig is 0.2)
        self.om_max = 0.3   # maximum angular velocity (orig is 0.4)

        self.v_des = 0.12   # desired cruising velocity
        self.theta_start_thresh = 0.05   # threshold in theta to start moving forward when path-following
        self.start_pos_thresh = 0.2     # threshold to be far enough into the plan to recompute it

        # threshold at which navigator switches from trajectory to pose control
        self.near_thresh = 0.2 #orig was 0.2
        self.at_thresh = 0.1 #orig  was 0.02
        self.at_thresh_theta = 0.05

        # trajectory smoothing
        self.spline_alpha = 0.15
        self.traj_dt = 0.1

        # trajectory tracking controller parameters
        self.kpx = 0.5 #orig was 0.5
        self.kpy = 0.5 #orig was 0.5
        self.kdx = 1.5 #orig was 1.5
        self.kdy = 1.5 #orig was 1.5

        # pose controller parameters
        self.k1 = 0.7
        self.k2 = 0.7
        self.k3 = 0.7

        # heading controller parameters
        self.kp_th = 2.0 #orig was 2.

        self.traj_controller = TrajectoryTracker(self.kpx, self.kpy, self.kdx, self.kdy, self.v_max, self.om_max)
        self.pose_controller = PoseController(self.k1, self.k2, self.k3, self.v_max, self.om_max)
        self.heading_controller = HeadingController(self.kp_th, self.om_max)

        self.nav_planned_path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
        self.nav_smoothed_path_pub = rospy.Publisher('/cmd_smoothed_path', Path, queue_size=10)
        self.nav_smoothed_path_rej_pub = rospy.Publisher('/cmd_smoothed_path_rejected', Path, queue_size=10)
        self.nav_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.inflated_occupancy_grid = rospy.Publisher('/inflated_occupancy_grid', OccupancyGrid, queue_size=10)

        self.trans_listener = tf.TransformListener()

        #self.cfg_srv = Server(NavigatorConfig, self.dyn_cfg_callback)
        
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/map_metadata', MapMetaData, self.map_md_callback)
        rospy.Subscriber('/cmd_nav', Pose2D, self.cmd_nav_callback)
        rospy.Subscriber('/delivery_request',  String, self.delivery_callback)
        rospy.Subscriber('/detected_objects_list', DetectedObjectList, self.detected_obj_callback)
        rospy.Subscriber('/marker_topic_0', Marker, self.marker_callback)
        rospy.Subscriber('/marker_topic_1', Marker, self.marker_callback)
        rospy.Subscriber('/marker_topic_2', Marker, self.marker_callback)
        rospy.Subscriber('/marker_topic_3', Marker, self.marker_callback)
        rospy.Subscriber('/marker_topic_4', Marker, self.marker_callback)
        #rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)

    def marker_callback(self, msg):
        self.marker_dict[msg.id] = (msg.pose.position.x, msg.pose.position.y) 

    def detected_obj_callback(self, msg):
        self.detected_objects_names = msg.objects
        self.detected_objects = msg.ob_msgs

    def delivery_callback(self, msg):
        self.delivery_req_list.append(msg.data)
        if msg.data in self.detected_objects_names:
            self.ifdelivery = True
            self.delivery_req_list = msg.data.split(',')
            #find what index is associated with what object
            #The markers are numbers as the robot sees it
            #and the detected objects list is labeled as the  robot sees it
            for i in range(len(self.detected_objects_names)):
                self.objectname_markerLoc_dict[self.detected_objects_names[i]] = self.marker_dict[i]
            #we assume we have all the items in the list and the map is known
            for i in range(self.delivery_req_list):
                self.deliver(self.delivery_req_list[i])
        elif msg.data in ['waypoint1']:
            self.x_g = 3.38
            self.y_g = 2.36 
            self.theta_g = np.pi/2.0
            self.replan()
        elif msg.data in ['waypoint2']: 
            self.x_g = 3.06 #3.2 
            self.y_g = 2.79 #2.82
            self.theta_g = -np.pi
            self.replan()
        elif msg.data in ['waypoint3']:
            self.x_g = 1.67 #1.62
            self.y_g = 2.78#2.73
            self.theta_g = -np.pi #-np.pi
            self.replan() 
        elif msg.data in ['waypoint4']:
            self.x_g = 0.684
            self.y_g = 2.75
            self.theta_g = -1.81
            self.replan()
        elif msg.data in ['waypoint5']:
            self.x_g = 0.25
            self.y_g = 1.63
            self.theta_g = 0.0
            self.replan()
        elif msg.data in ['waypoint6']:
            self.x_g = 1.078
            self.y_g = 1.63
            self.theta_g = 0.0
            self.replan()
        elif msg.data in ['waypoint7']:
            self.x_g = 2.01
            self.y_g = 1.65
            self.theta_g = 0.0
            self.replan()
        elif msg.data in ['waypoint8']:
            self.x_g = 2.31
            self.y_g = 1.33
            self.theta_g = -np.pi/2
            self.replan()
        elif msg.data in ['waypoint9']:
            self.x_g = 2.07
            self.y_g = 0.36
            self.theta_g = -np.pi
            self.replan()
        elif msg.data in ['waypoint10']:
            self.x_g = 1.07
            self.y_g = 0.26
            self.theta_g = -np.pi
            self.replan()
        elif msg.data in ['home']:
            self.x_g = self.x_init
            self.y_g = self.y_init
            self.theta_g  = 0.0
            self.replan()

    def deliver(self, object_name):
        #find what index is associated with what object
        #The markers are numbers as the robot sees it
        #and the detected objects list is labeled as the  robot sees it
        self.x_g, self.y_g = self.objectname_markerLoc_dict[object_name]  
        self.theta_g = 0.0
        self.replan()

    def dyn_cfg_callback(self, config, level):
        rospy.loginfo("Reconfigure Request: k1:{k1}, k2:{k2}, k3:{k3}".format(**config))
        self.pose_controller.k1 = config["k1"]
        self.pose_controller.k2 = config["k2"]
        self.pose_controller.k3 = config["k3"]
        return config

    def cmd_nav_callback(self, data):
        """
        loads in goal if different from current goal, and replans
        """
        if data.x != self.x_g or data.y != self.y_g or data.theta != self.theta_g:

            self.x_g = data.x
            self.y_g = data.y
            self.theta_g = data.theta
            rospy.loginfo("cmd_nav_callback goal: " + str(self.x_g)) 
            self.replan()

    def map_md_callback(self, msg):
        """
        receives maps meta data and stores it
        """
        self.map_width = msg.width
        self.map_height = msg.height
        self.map_resolution = msg.resolution
        self.map_origin = (msg.origin.position.x,msg.origin.position.y)

    def map_callback(self,msg):
        """
        receives new map info and updates the map
        msg.data is the map data, in row-major order, starting with (0,0).  Occupancy
        probabilities are in the range [0,100].  Unknown is -1.
        """
        self.map_probs = msg.data
        #we should get a int8 array, so we reshape into a 2D array
        map_probs2D = np.array(msg.data).reshape((msg.info.height,msg.info.width))
        #create a mask so that we don't touch -1 unknown values
        mask = np.logical_not(map_probs2D<0)
        #threshold so that anything below this level is set to 0
        map_probs2D_thresholded = (map_probs2D >= self.map_threshold) * 100
        #dilate the map so that we don't crash into walls
        map_probs2D_dilated = ndimage.binary_dilation(map_probs2D_thresholded,iterations=1).astype(np.int8) *100
        #add back unknown values into the map
        map_probs2D_dilated[mask] = -1
        #reshape back into original format
        inflated_OG = map_probs2D_dilated.reshape((msg.info.height*msg.info.width))

        #publish the inflated occupancy grid (for debugging)
        inflated_OG_msg = OccupancyGrid()
        inflated_OG_msg.info = msg.info
        inflated_OG_msg.data = inflated_OG
        self.inflated_occupancy_grid.publish(inflated_OG_msg)

        # if we've received the map metadata and have a way to update it:
        if self.map_width>0 and self.map_height>0 and len(self.map_probs)>0:
            self.occupancy = StochOccupancyGrid2D(self.map_resolution,
                                                  self.map_width,
                                                  self.map_height,
                                                  self.map_origin[0],
                                                  self.map_origin[1],
                                                  8,
                                                  inflated_OG)
            if self.x_g is not None:
                # if we have a goal to plan to, replan
                rospy.loginfo("replanning because of new map")
                self.replan() # new map, need to replan

    def shutdown_callback(self):
        """
        publishes zero velocities upon rospy shutdown
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.nav_vel_pub.publish(cmd_vel)

    def stop_sign_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign
        dist = msg.distance

        # if close enough and in nav mode, stop
        if dist > 0 and dist < self.stop_min_dist and self.mode == Mode.TRACK:
            self.init_stop_sign()

    def init_stop_sign(self):
         """ initiates a stop sign maneuver """
         self.stop_sign_start = rospy.get_rostime()
         self.mode = Mode.STOP
    def has_stopped(self):
        """ checks if stop sign maneuver is over """
        return self.mode == Mode.STOP and \
               rospy.get_rostime() - self.stop_sign_start > rospy.Duration.from_sec(self.stop_time)
    def stay_idle(self):
        """ sends zero velocity to stay put """

        vel_g_msg = Twist()
        self.cmd_vel_publisher.publish(vel_g_msg)
    def init_crossing(self):
        """ initiates an intersection crossing maneuver """

        self.cross_start = rospy.get_rostime()
        self.mode = Mode.CROSS

    def has_crossed(self):
        """ checks if crossing maneuver is over """

        return self.mode == Mode.CROSS and \
               rospy.get_rostime() - self.cross_start > rospy.Duration.from_sec(self.crossing_time)

    def near_goal(self):
        """
        returns whether the robot is close enough in position to the goal to
        start using the pose controller
        """
        return linalg.norm(np.array([self.x-self.x_g, self.y-self.y_g])) < self.near_thresh

    def at_goal(self):
        """
        returns whether the robot has reached the goal position with enough
        accuracy to return to idle state
        """
        return (linalg.norm(np.array([self.x-self.x_g, self.y-self.y_g])) < self.at_thresh and abs(wrapToPi(self.theta - self.theta_g)) < self.at_thresh_theta)

    def aligned(self):
        """
        returns whether robot is aligned with starting direction of path
        (enough to switch to tracking controller)
        """
        return (abs(wrapToPi(self.theta - self.th_init)) < self.theta_start_thresh)
        
    def close_to_plan_start(self):
        return (abs(self.x - self.plan_start[0]) < self.start_pos_thresh and abs(self.y - self.plan_start[1]) < self.start_pos_thresh)

    def snap_to_grid(self, x):
        return (self.plan_resolution*round(x[0]/self.plan_resolution), self.plan_resolution*round(x[1]/self.plan_resolution))

    def switch_mode(self, new_mode):
        rospy.loginfo("Switching from %s -> %s", self.mode, new_mode)
        self.mode = new_mode

    def publish_planned_path(self, path, publisher):
        # publish planned plan for visualization
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for state in path:
            pose_st = PoseStamped()
            pose_st.pose.position.x = state[0]
            pose_st.pose.position.y = state[1]
            pose_st.pose.orientation.w = 1
            pose_st.header.frame_id = 'map'
            path_msg.poses.append(pose_st)
        publisher.publish(path_msg)

    def publish_smoothed_path(self, traj, publisher):
        # publish planned plan for visualization
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for i in range(traj.shape[0]):
            pose_st = PoseStamped()
            pose_st.pose.position.x = traj[i,0]
            pose_st.pose.position.y = traj[i,1]
            pose_st.pose.orientation.w = 1
            pose_st.header.frame_id = 'map'
            path_msg.poses.append(pose_st)
        publisher.publish(path_msg)

    def publish_control(self):
        """
        Runs appropriate controller depending on the mode. Assumes all controllers
        are all properly set up / with the correct goals loaded
        """
        t = self.get_current_plan_time()

        if self.mode == Mode.PARK:
            V, om = self.pose_controller.compute_control(self.x, self.y, self.theta, t)
        elif self.mode == Mode.TRACK:
            V, om = self.traj_controller.compute_control(self.x, self.y, self.theta, t)
        elif self.mode == Mode.ALIGN:
            V, om = self.heading_controller.compute_control(self.x, self.y, self.theta, t)
        else:
            V = 0.
            om = 0.

        #print('V om ', V, om)

        cmd_vel = Twist()
        cmd_vel.linear.x = V
        cmd_vel.angular.z = om
        self.nav_vel_pub.publish(cmd_vel)

    def get_current_plan_time(self):
        t = (rospy.get_rostime()-self.current_plan_start_time).to_sec()
        return max(0.0, t)  # clip negative time to 0

    def replan(self):
        """
        loads goal into pose controller
        runs planner based on current pose
        if plan long enough to track:
            smooths resulting traj, loads it into traj_controller
            sets self.current_plan_start_time
            sets mode to ALIGN
        else:
            sets mode to PARK
        """
        # Make sure we have a map
        if not self.occupancy:
            rospy.loginfo("Navigator: replanning canceled, waiting for occupancy map.")
            self.switch_mode(Mode.IDLE)
            return

        # Attempt to plan a path
        #state_min = self.snap_to_grid((-self.plan_horizon, -self.plan_horizon))
        #state_max = self.snap_to_grid((self.plan_horizon, self.plan_horizon))
        state_min = self.snap_to_grid((0.0, 0.0))
        state_max = self.snap_to_grid((self.plan_horizon, self.plan_horizon))

        x_init = self.snap_to_grid((self.x, self.y))
        self.plan_start = x_init
        x_goal = self.snap_to_grid((self.x_g, self.y_g))

        rospy.loginfo('In replan, x_init,y_init,th_init:' + str(x_init) + ', '+str(self.th_init))
        rospy.loginfo('In replan, x_goal and th_g is:' + str(x_goal) + ', '+str(self.theta_g))

        problem = AStar(state_min,state_max,x_init,x_goal,self.occupancy,self.plan_resolution)

        rospy.loginfo("Navigator: computing navigation plan")
        success =  problem.solve()
        if not success:
            rospy.loginfo("Planning failed")
            return
        rospy.loginfo("Planning Succeeded")

        planned_path = problem.path
        

        # Check whether path is too short
        if len(planned_path) < 4:    
            rospy.loginfo("Path too short to track")
            self.switch_mode(Mode.PARK)
            return

        # Smooth and generate a trajectory
        traj_new, t_new = compute_smoothed_traj(planned_path, self.v_des, self.spline_alpha, self.traj_dt)

        # If currently tracking a trajectory, check whether new trajectory will take more time to follow
        if self.mode == Mode.TRACK:
            t_remaining_curr = self.current_plan_duration - self.get_current_plan_time()

            # Estimate duration of new trajectory
            th_init_new = traj_new[0,2]
            th_err = wrapToPi(th_init_new - self.theta)
            t_init_align = abs(th_err/self.om_max)
            t_remaining_new = t_init_align + t_new[-1]

            if t_remaining_new > t_remaining_curr:
                rospy.loginfo("New plan rejected (longer duration than current plan)")
                self.publish_smoothed_path(traj_new, self.nav_smoothed_path_rej_pub)
                return

        # Otherwise follow the new plan
        self.publish_planned_path(planned_path, self.nav_planned_path_pub)
        self.publish_smoothed_path(traj_new, self.nav_smoothed_path_pub)

        self.pose_controller.load_goal(self.x_g, self.y_g, self.theta_g)

        print('pose_controller.load_goal:', self.x_g)

        self.traj_controller.load_traj(t_new, traj_new)

        self.current_plan_start_time = rospy.get_rostime()
        self.current_plan_duration = t_new[-1]

        self.th_init = traj_new[0,2]
        self.heading_controller.load_goal(self.th_init)

        if not self.aligned():
            rospy.loginfo("Not aligned with start direction")
            self.switch_mode(Mode.ALIGN)
            return

        rospy.loginfo("Ready to track")
        self.switch_mode(Mode.TRACK)

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            # try to get state information to update self.x, self.y, self.theta
            try:
                (translation,rotation) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                self.current_plan = []
                rospy.loginfo("Navigator: waiting for state info")
                self.switch_mode(Mode.IDLE)
                print e
                pass

            # STATE MACHINE LOGIC
            # some transitions handled by callbacks
            if self.mode == Mode.IDLE:
                pass
            elif self.mode == Mode.STOP:
                # At a stop sign
                while True:
                    self.stay_idle()
                    if self.has_stopped():
                        self.init_crossing()
                        break
            elif self.mode == Mode.CROSS:
                # Crossing an intersection
                while True:
                    self.replan()
                    if self.has_crossed():
                        self.mode = Mode.TRACK
                        break
            elif self.mode == Mode.ALIGN:
                if self.aligned():
                    self.current_plan_start_time = rospy.get_rostime()
                    self.switch_mode(Mode.TRACK)
            elif self.mode == Mode.TRACK:
                if self.near_goal():
                    self.switch_mode(Mode.PARK)
                elif not self.close_to_plan_start():
                    rospy.loginfo("replanning because far from start")
                    self.replan()
                elif (rospy.get_rostime() - self.current_plan_start_time).to_sec() > self.current_plan_duration:
                    rospy.loginfo("replanning because out of time")
                    self.replan() # we aren't near the goal but we thought we should have been, so replan
            elif self.mode == Mode.PARK:
                if self.at_goal():
                    # forget about goal:
                    self.x_g = None
                    self.y_g = None
                    self.theta_g = None
                    self.switch_mode(Mode.IDLE)

            self.publish_control()

            rate.sleep()

if __name__ == '__main__':    
    nav = Navigator()
    rospy.on_shutdown(nav.shutdown_callback)
    nav.run()
