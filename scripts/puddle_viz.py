#!/usr/bin/env python
import rospy
import numpy as np
import tf
import tf.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker

# look up doc.
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


from std_msgs.msg import ColorRGBA
import matplotlib.pyplot as plt


'''
This code identifies shiny points on the floor (aka puddle) and estimates its position.
It looks straight ahead within a 60 degree view, and only RHO meters in front.
Also look at velodyne_filter.launch

It broadcasts a TF frame called puddle, and draws a circle about the mean of the puddle position.

Although this "works", it is not perfect and it can be significantly improved.
- Currently, it is very noisy, as it considers other points from far away places.
- Currently, the puddle size is fixed.
- What happens when there are multiple nearby puddles?
- Does not visualize the "viewing sector" of the robot.
'''

# min number of points to be considered a puddle 
MIN_POINTS = 3

# look ahead distance to search for puddles
RHO = 1.5

def compute_ellipse_points(a, b):
    th = np.arange(0, 2*np.pi+1, 0.2)
    x = a * np.cos(th)
    y = b * np.sin(th)
    return np.stack([x,y])

def initialize_puddle_marker():
    puddle_marker = Marker()
    puddle_marker.header.frame_id = "/puddle"
    puddle_marker.ns = "ellipse"
    puddle_marker.type = Marker.LINE_STRIP
    puddle_marker.scale.x = 0.01
    puddle_marker.frame_locked = True
    puddle_marker.color.g = 1
    puddle_marker.color.a = 1
    return puddle_marker

class PuddleViz:
    def __init__(self):
        rospy.init_node("puddle_viz")
        self.puddle_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.puddle_mean = None
        self.puddle_viz_pub = rospy.Publisher("/viz/puddle", Marker, queue_size=10)
        self.puddle_marker = initialize_puddle_marker()
        rospy.Subscriber("/velodyne_puddle_filter", PointCloud2, self.velodyne_callback)


    def velodyne_callback(self, msg):
        '''
        This is an example of how to process PointCloud2 data.
        pc2.read_points creates an _iterator_ object.
        '''
        lidar_info = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        num_points = len(list(lidar_info))
        lidar_info = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        x_coords = np.zeros(num_points)
        y_coords = np.zeros(num_points)
        z_coords = np.zeros(num_points)
        i = 0
        # looping through the point cloud
        for p in lidar_info:

            x_coords[i] = p[0]
            y_coords[i] = p[1]
            z_coords[i] = p[2]
            i += 1

        pt_ranges = np.hypot(x_coords, y_coords)
        pt_angles = np.arctan2(y_coords, x_coords)

        # filter based on range
        pts_within_range = (pt_ranges < RHO)

        # filter based on angle
        pts_within_angle = (pt_angles < np.pi/6) & (pt_angles > -np.pi/6)

        # filtered points
        filtered_points = pts_within_range & pts_within_angle

        x_filtered = x_coords[filtered_points]
        y_filtered = y_coords[filtered_points]
        z_filtered = z_coords[filtered_points]
        self.puddle_time = msg.header.stamp

        if sum(filtered_points) > MIN_POINTS:
            self.puddle_mean = (np.mean(x_filtered), np.mean(y_filtered), np.mean(z_filtered))
            self.puddle_var = (np.var(x_filtered), np.var(y_filtered), np.var(z_filtered))



    def loop(self):

        if self.puddle_mean is not None:
            pt = PointStamped()
            pt.header.frame_id = '/velodyne'
            pt.header.stamp = self.puddle_time
            pt.point.x = self.puddle_mean[0]
            pt.point.y = self.puddle_mean[1]
            pt.point.z = self.puddle_mean[2]

            try:
                # send a tf transform of the puddle location in the map frame
                self.tf_listener.waitForTransform("/map", '/velodyne', self.puddle_time, rospy.Duration(.05))
                puddle_map_pt = self.tf_listener.transformPoint("/map", pt)
                self.puddle_broadcaster.sendTransform((puddle_map_pt.point.x, puddle_map_pt.point.y, puddle_map_pt.point.z), 
                                                       [0, 0, 0, 1],
                                                       self.puddle_time,
                                                       "/puddle",
                                                       "/map")
                
                # make puddle marker
                ellipse_points = compute_ellipse_points(0.2, 0.2)
                self.puddle_marker.points = []
                for i in range(ellipse_points.shape[-1]):
                    # print("drawing ellipse")
                    self.puddle_marker.points.append(Point(ellipse_points[0,i], ellipse_points[1,i], 0)) 
                self.puddle_viz_pub.publish(self.puddle_marker)

            except:
                pass


    def run(self):
        rate = rospy.Rate(50) # 50 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


if __name__ == '__main__':
    puddle_viz = PuddleViz()
    puddle_viz.run()