#!/usr/bin/env python

import argparse
import multiprocessing
import signal
import time

import cv2 as cv
import numpy as np
import rospy

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage

CX_DES, CY_DES = (0.43, 0.9)
GAIN_V, GAIN_OM = (0.1, 1.)

def servo(img, cx_des=CX_DES, cy_des=CY_DES, gain_V=GAIN_V, gain_om=GAIN_OM):
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
        gain_V  - error gain for V (higher means faster convergence).
        gain_om - error gain for om (higher means faster convergence).

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

    def __init__(self, cx_des=CX_DES, cy_des=CY_DES, gain_V=GAIN_V, gain_om=GAIN_OM):
        self.control_command = multiprocessing.Array('d', (0., 0.))
        self.p = multiprocessing.Process(target=self._worker,
                                         args=(self.control_command, cx_des,
                                               cy_des, gain_V, gain_om))

    @staticmethod
    def _worker(control_command, cx_des, cy_des, gain_V, gain_om):
        print("Starting visual servo worker")
        rospy.init_node('visual_servo_worker')
        bridge = CvBridge()

        def camera_callback(msg):
            img = bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            V, om = servo(img, cx_des, cy_des, gain_V, gain_om)
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
    parser = argparse.ArgumentParser()
    parser.add_argument('pub_topic', nargs='?', default='/cmd_vel',
                        help="ROS topic to publish the command velocities")
    parser.add_argument('--cx_des', default=CX_DES, type=float,
                        help="Desired horizontal position of the flag in the image")
    parser.add_argument('--cy_des', default=CY_DES, type=float,
                        help="Desired vertical position of the flag in the image")
    parser.add_argument('--gain_V', default=GAIN_V, type=float,
                        help="Error gain for V")
    parser.add_argument('--gain_om', default=GAIN_OM, type=float,
                        help="Error gain for om")
    args = parser.parse_args(rospy.myargv()[1:])

    is_running = [True]

    # Initialize visual servo
    vs = VisualServo(args.cx_des, args.cy_des, args.gain_V, args.gain_om)
    vs.start()

    # Initialize ROS node
    rospy.init_node('visual_servo')
    pub = rospy.Publisher(args.pub_topic, Twist, queue_size=10)

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
