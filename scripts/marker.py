#!/usr/bin/env python

'''
DESCRIPTION: Creates an array of markers that are published on the /markers topic
These markers can then be visualized in Rviz. 

Currently create a circular marker at the robot's position. Can be extended to add 
other circular markers of different colors and shapes for objects such as food, 
stop signs, puddles, etc.
'''

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from gazebo_msgs.msg import ModelStates
import tf

#----Module level variables---
#Robot marker definitions
ROBOT_SIZE = (0.16, 0.16, 0.16)
ROBOT_COLOR = (1.0, 0.0, 0.0, 0.8)
ROBOT_FRAME = "base_footprint"
ROBOT_POSITION_IN_FRAME = (0.0, 0.0, 0.0)

#Food marker definitions example
FOOD_SIZE = (0.16, 0.16, 0.16)
FOOD_COLOR = (0.0, 1.0, 0.0, 0.8)
FOOD_FRAME = "map"
FOOD_POSITION_IN_FRAME = (1.0, 1.0, 0.0)


class MarkerPublisher:

	def __init__(self):
		rospy.init_node('RvizMarkers', anonymous=True)
		#Publisher to publish markers to Rviz
		self.pub = rospy.Publisher('/markers', MarkerArray, queue_size=10) 
		#self.pub2 = rospy.Publisher('/Food_marker', Marker, queue_size=10) 

		# current state
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0
		self.z = 1

		# goal state
		self.x_g = 0.0
		self.y_g = 0.0
		self.theta_g = 0.0

		# marker array
		self.marker_array = MarkerArray()
		self.numObjects = 0
		self.markers = []

		#Create Robot marker
		self.createMarker(ROBOT_FRAME, ROBOT_POSITION_IN_FRAME, ROBOT_COLOR, ROBOT_SIZE)

		#Create example food marker (Will add functionality for automatically adding
		# food markers later)
		self.createMarker(FOOD_FRAME, FOOD_POSITION_IN_FRAME, FOOD_COLOR,FOOD_SIZE)


	'''
	Function: 		createMarkers
	Description: 	Adds a new marker to self.markers. Right now just adds
					a cylinder with the given parameters.

					EXAMPLE PARAMETERS
					frame = "map"
					color = [r,g,b,a]
					pos_in_frame = [0,0,0]
					scale = [0.16,0.16,0.16]
	'''
	def createMarker(self, frame, pos_in_frame, color,scale):
		#Create our new marker
		NewMarker = Marker(
					type=3, #Cylinder
					id=self.numObjects,
					lifetime=rospy.Duration(1.5),
					pose=Pose(Point(*pos_in_frame), Quaternion(0, 0, 0, 1)),
					scale=Vector3(*scale),
					header=Header(frame_id=frame), 
					color=ColorRGBA(*color),
					)
		#Increase our marker object count
		self.numObjects += 1

		#Add marker to our list of active markers
		self.markers.append(NewMarker)


	#Not implemented yet. Deletes markers if desired
	def deleteMarker(self, marker_num):
		pass
			
	#Construct MarkerArray and publish
	def publish_markers(self):
		#TODO: Find out a way to append new markers directly to MarkerArray
		self.marker_array = MarkerArray(markers = self.markers)
		self.pub.publish(self.marker_array)

	
	############################
	#### Callback functions ####
	############################

	#Probably need a callback for whatever is detecting new food items, stop signs, 
	#puddles, etc
		
	#Run function
	def run(self):
		rospy.loginfo("Starting Rviz Marker Node")
		rate = rospy.Rate(5)
		while not rospy.is_shutdown():
			self.publish_markers()
			rate.sleep()


if __name__ == '__main__':
	try:
		MarkerPub = MarkerPublisher()
		MarkerPub.run()
	except rospy.ROSInterruptException:
		pass




