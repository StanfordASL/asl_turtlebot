#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, CameraInfo

class CameraRelay:
    def __init__(self):
        rospy.init_node('camera_relay', anonymous=True)
        rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.camera_callback, queue_size=1, buff_size=2**24)
        rospy.Subscriber('/raspicam_node/camera_info', CameraInfo, self.camera_info_callback, queue_size=1, buff_size=2**24)
        self.camera_info_repub = rospy.Publisher('/camera_relay/camera_info', CameraInfo, queue_size=1)
        self.camera_repub = rospy.Publisher('/camera_relay/image/compressed', CompressedImage, queue_size=1)
        self.msg = None
        self.info = None

    def camera_callback(self, msg):
        self.msg = msg

    def camera_info_callback(self, msg):
        self.info = msg

    def run(self):
        rate = rospy.Rate(30) # 30 Hz
        while not rospy.is_shutdown():
            if self.msg is not None:
                self.camera_repub.publish(self.msg)
            if self.info is not None:   
                self.camera_info_repub.publish(self.info)
            rate.sleep()

if __name__ == '__main__':
    relay = CameraRelay()
    relay.run()
