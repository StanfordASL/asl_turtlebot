#!/usr/bin/env python

import multiprocessing
import signal
import time

import cv2 as cv
import numpy as np
import rospy

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage

def servo(img, cx_des=0.43, cy_des=0.9, gain_V=0.1, gain_om=1.):
    """
    Return control commands (V, om) to servo towards the pink flag base given an
    input image. If no flag base is detected, returns 0.

    Args:
        img     - 308x410 BGR8 OpenCV image (portrait),
                  a.k.a np.array(shape=(410,308,3), dtype=np.uint8).
        cx_des  - desired horizontal position of the flag in the image
                  (0.0 is left side, 1.0 is right side).
        cy_des  - desired vertical position of the flag in the image
                  (0.0 is top, 1.0 is bottom).
        gain_V  - error gain for V.
        gain_om - error gain for om.

    Return:
        V  - command linear velocity.
        om - command angular velocity.
    """

    def pink_centroid(img):
        """
        Return the centroid of the pink pixels, along with the segmentation mask.
        """
        # Extract the pink pixels
        lab = cv.cvtColor(img, cv.COLOR_BGR2LAB)
        mask = cv.inRange(lab, (0, 158, 98), (255, 255, 168))
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, np.ones((3,3)), iterations=3)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, np.ones((3,3)))

        # Extract the largest connected component
        num_cc, mask_cc, stats, centroids = cv.connectedComponentsWithStats(mask)
        if num_cc < 2:
            return None, mask_cc
        areas = stats[1:,cv.CC_STAT_AREA]
        idx_max_area = np.argmax(areas) + 1

        cx, cy = centroids[idx_max_area]
        return (cx, cy), mask_cc

    # Compute the centroid of pink pixels
    centroid, mask = pink_centroid(img)
    if centroid is None:
        return 0., 0.

    # Compute the control commands
    cx, cy = centroid
    om = gain_om * (cx_des - cx / img.shape[1])
    V = gain_V * (cy_des - cy / img.shape[0])
    return V, om

class VisualServo:
    """
    Class that spawns a ROS node as a child process, subscribes to the camera
    topic, and continuously computes (but doesn't publish) visual servo commands.

    Usage:
        vs = VisualServo()
        vs.start()
        while running:
            V, om = vs.get_control_command()
        vs.join()
    """

    def __init__(self):
        self.control_command = multiprocessing.Array('d', (0., 0.))
        self.p = multiprocessing.Process(target=self._worker, args=(self.control_command,))

    @staticmethod
    def _worker(control_command):
        print("Starting visual servo worker")
        rospy.init_node('visual_servo_worker')
        bridge = CvBridge()

        def camera_callback(msg):
            img = bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            V, om = servo(img)
            control_command[0] = V
            control_command[1] = om

        sub = rospy.Subscriber('/camera_relay/image/compressed', CompressedImage,
                               camera_callback, queue_size=1, buff_size=2**24)
        rospy.spin()

    def start(self):
        """
        Start the visual servo process.
        """
        self.p.start()

    def join(self):
        """
        Clean up the visual servo process.
        """
        self.p.join()

    def get_control_command(self):
        """
        Get the last computed visual servo command.
        """
        V = self.control_command[0]
        om = self.control_command[1]
        return V, om

if __name__ == '__main__':
    is_running = [True]

    # Initialize visual servo
    vs = VisualServo()
    vs.start()

    # Initialize ROS node
    rospy.init_node('visual_servo')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Ctrl-c handler
    def ctrl_c_handler(signal, frame):
        is_running[0] = False
    signal.signal(signal.SIGINT, ctrl_c_handler)

    # Run servo loop
    print("Starting visual servo")
    r = rospy.Rate(29)
    while is_running[0] and not rospy.is_shutdown():
        V, om = vs.get_control_command()
        twist = Twist()
        twist.linear.x, twist.angular.z = V, om
        pub.publish(twist)
        r.sleep()

    # Clear velocity command before exiting
    print("Cleaning up")
    pub.publish(Twist())
    vs.join()

    time.sleep(0.1)
