#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Int8

def gripper_publisher(pwm):
	rospy.loginfo("Starting....")
	pub = rospy.Publisher('gripper_pose', Int8, queue_size = 10)
	rospy.init_node('gripper_publisher', anonymous = True)
	rate = rospy.Rate(10)
	#while not rospy.is_shutdown():
	pub.publish(pwm)
	rate.sleep()

if __name__ == '__main__':
	try:
		while(1):
			key = int(input("Enter 1 to close and 0 to open the gripper : "))
			if key == 1:
				gripper_publisher(0)
			elif key == 0:
				gripper_publisher(100)
			else:
				sys.exit()
			print("")
	except:
		pass
