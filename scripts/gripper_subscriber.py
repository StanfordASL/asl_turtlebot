#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8

import pigpio
import time

SERVO_PIN = 18
PULSE_WIDTH_OPEN = 1800
PULSE_WIDTH_CLOSED = 950

class Gripper:
	def __init__(self):
		rospy.loginfo("Starting subcribing")
		self.pi = pigpio.pi()
		self.pi.set_mode(SERVO_PIN, pigpio.OUTPUT)
		self.pi.set_servo_pulsewidth(SERVO_PIN, PULSE_WIDTH_OPEN)
		#time.sleep(1000)
		self.pulse_width = PULSE_WIDTH_OPEN
		self.pi.stop()

	def gripper_callback(self, data):
		temp = self.pulse_width
		self.pi = pigpio.pi()
		self.pi.set_mode(SERVO_PIN, pigpio.OUTPUT)
		scale = (PULSE_WIDTH_OPEN - PULSE_WIDTH_CLOSED)/100
		self.pulse_width = PULSE_WIDTH_CLOSED + (scale * data.data)
		if (self.pulse_width >  PULSE_WIDTH_OPEN or self.pulse_width < PULSE_WIDTH_CLOSED):
			self.pulse_width = temp
			rospy.loginfo("Invalid pulse width")
		self.pi.set_servo_pulsewidth(SERVO_PIN, self.pulse_width)
		#time.sleep(1000)
		rospy.loginfo("Pulse width is set to : %d" %(self.pulse_width))
		self.pi.stop()		

	def gripper_subscriber(self):
		rospy.init_node('gripper_subscriber', anonymous = True)
		rospy.Subscriber('gripper_pose', Int8, self.gripper_callback)
		rospy.loginfo("Subcriber Spinning")
		self.pi.stop()
		rospy.spin()

if __name__ == '__main__':
	rospy.loginfo("Where am I?")
	servo = Gripper()
	servo.gripper_subscriber()

