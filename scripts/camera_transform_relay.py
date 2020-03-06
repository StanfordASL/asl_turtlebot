#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, CameraInfo

class ImageTransformer:
    def __init__(self):
        rospy.init_node('tb3_image_transformer',anonymous=True)
        rospy.Subscriber('/raspicam_node/image/compressed',
                            CompressedImage,
                            self.transform_image_and_repub,
                            queue_size=1,buff_size = 2**24)
        rospy.Subscriber('/raspicam_node/camera_info', CameraInfo, self.camera_info_callback, queue_size=1, buff_size=2**24)
        self.image_repub = rospy.Publisher('/camera_relay/image/compressed', CompressedImage, queue_size=1)
        self.camera_info_repub = rospy.Publisher('/camera_relay/camera_info', CameraInfo, queue_size=1)

    def transform_image_and_repub(self, msg):
        img = cv2.imdecode(np.fromstring(msg.data, np.uint8), cv2.IMREAD_COLOR)
        img_rotated = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        repub_msg = CompressedImage()
        repub_msg.header = msg.header
        repub_msg.format = "jpeg"
        repub_msg.data = np.array(cv2.imencode('.jpg', img_rotated)[1]).tostring()
        self.image_repub.publish(repub_msg)

    def camera_info_callback(self, msg):
        self.camera_info_repub.publish(msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    r = ImageTransformer()
    r.run()
