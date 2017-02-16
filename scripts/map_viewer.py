#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
import numpy as np
import math
import matplotlib.pyplot as plt


class MapViewer:

    def __init__(self):
        self.map_width = 0
        self.map_height = 0

    def map_md_callback(self,msg):
        self.map_width = msg.width
        self.map_height = msg.height

    def map_callback(self,msg):
        if self.map_width>0 and self.map_height>0:
            map_np = np.array(msg.data);
            map_np = map_np.reshape(self.map_height,self.map_width)
            plt.imshow(map_np, cmap='hot', interpolation='nearest')
            plt.show()

    def run(self):
        rospy.init_node('map_viewer', anonymous=True)
        rospy.Subscriber("map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("map_metadata", MapMetaData, self.map_md_callback)
        rospy.spin()

if __name__ == '__main__':
    mv = MapViewer()
    mv.run()
