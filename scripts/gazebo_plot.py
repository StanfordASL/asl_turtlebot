#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import numpy as np
from gazebo_msgs.msg import ModelStates 


BUF_SIZE = 5000

class GazeboPlot:

    def __init__(self):
	self.x = []
	self.y = []

	plt.ion()
	self.fig = plt.figure()
	self.ax = self.fig.add_subplot(111)
	self.ax.set_xlim(-3,3)
	self.ax.set_ylim(-3,3)
	self.line1, = self.ax.plot(self.x, self.y, 'h') 

    def gazebo_callback(self, data):
        pose = data.pose[data.name.index("turtlebot3_burger")]
	if len(self.x)==0 or len(self.y)==0:
		self.x = [pose.position.x] * BUF_SIZE
		self.y = [pose.position.y] * BUF_SIZE
	else:
        	self.x = self.x[1:]
		self.y = self.y[1:]
		self.x.append(pose.position.x)
		self.y.append(pose.position.y)

    def run(self):
        rospy.init_node('gazebo_plot', anonymous=True)
	rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
	rate = rospy.Rate(100)
        while not rospy.is_shutdown():
		self.line1.set_xdata(self.x)
		self.line1.set_ydata(self.y)
    		self.fig.canvas.draw()	
            	rate.sleep()


if __name__ == '__main__':
	gz_plot = GazeboPlot()
	gz_plot.run()


