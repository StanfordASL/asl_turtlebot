#!/usr/bin/env python

import rospy
import os
# watch out on the order for the next two imports lol
from tf import TransformListener
import tensorflow as tf
import numpy as np
from sensor_msgs.msg import CompressedImage, Image, CameraInfo, LaserScan
from asl_turtlebot.msg import DetectedObject, DetectedObjectList
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math

# path to the trained conv net
PATH_TO_MODEL = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../tfmodels/ssd_resnet_50_fpn_coco.pb')
PATH_TO_LABELS = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../tfmodels/coco_labels.txt')

# set to True to use tensorflow and a conv net
# False will use a very simple color thresholding to detect stop signs only
USE_TF = True
# minimum score for positive detection
MIN_SCORE = .5

def load_object_labels(filename):
    """ loads the coco object readable name """

    fo = open(filename,'r')
    lines = fo.readlines()
    fo.close()
    object_labels = {}
    for l in lines:
        object_id = int(l.split(':')[0])
        label = l.split(':')[1][1:].replace('\n','').replace('-','_').replace(' ','_')
        object_labels[object_id] = label

    return object_labels

class Detector:

    def __init__(self):
        rospy.init_node('turtlebot_detector', anonymous=True)
        self.bridge = CvBridge()

        self.detected_objects_pub = rospy.Publisher('/detector/objects', DetectedObjectList, queue_size=10)

        if USE_TF:
            self.detection_graph = tf.Graph()
            with self.detection_graph.as_default():
                od_graph_def = tf.GraphDef()
                with tf.gfile.GFile(PATH_TO_MODEL, 'rb') as fid:
                    serialized_graph = fid.read()
                    od_graph_def.ParseFromString(serialized_graph)
                    tf.import_graph_def(od_graph_def,name='')
                self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                self.d_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                self.d_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                self.d_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                self.num_d = self.detection_graph.get_tensor_by_name('num_detections:0')
                config = tf.ConfigProto()
                config.gpu_options.allow_growth = True
            self.sess = tf.Session(graph=self.detection_graph, config=config)
            # self.sess = tf.Session(graph=self.detection_graph)

        # camera and laser parameters that get updated
        self.cx = 0.
        self.cy = 0.
        self.fx = 1.
        self.fy = 1.
        self.laser_ranges = []
        self.laser_angle_increment = 0.01 # this gets updated

        self.object_publishers = {}
        self.object_labels = load_object_labels(PATH_TO_LABELS)

        self.tf_listener = TransformListener()
        rospy.Subscriber('/raspicam_node/image_raw', Image, self.camera_callback, queue_size=1, buff_size=2**24)
        rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.compressed_camera_callback, queue_size=1, buff_size=2**24)
        rospy.Subscriber('/raspicam_node/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)

    def run_detection(self, img):
        """ runs a detection method in a given image """

        image_np = self.load_image_into_numpy_array(img)
        image_np_expanded = np.expand_dims(image_np, axis=0)

        if USE_TF:
            # uses MobileNet to detect objects in images
            # this works well in the real world, but requires
            # good computational resources
            with self.detection_graph.as_default():
                (boxes, scores, classes, num) = self.sess.run(
                [self.d_boxes,self.d_scores,self.d_classes,self.num_d],
                feed_dict={self.image_tensor: image_np_expanded})

            return self.filter(boxes[0], scores[0], classes[0], num[0])

        else:
            # uses a simple color threshold to detect stop signs
            # this will not work in the real world, but works well in Gazebo
            # with only stop signs in the environment
            R = image_np[:,:,0].astype(np.int) > image_np[:,:,1].astype(np.int) + image_np[:,:,2].astype(np.int)
            Ry, Rx, = np.where(R)
            if len(Ry)>0 and len(Rx)>0:
                xmin, xmax = Rx.min(), Rx.max()
                ymin, ymax = Ry.min(), Ry.max()
                boxes = [[float(ymin)/image_np.shape[1], float(xmin)/image_np.shape[0], float(ymax)/image_np.shape[1], float(xmax)/image_np.shape[0]]]
                scores = [.99]
                classes = [13]
                num = 1
            else:
                boxes = []
                scores = 0
                classes = 0
                num = 0

            return boxes, scores, classes, num

    def filter(self, boxes, scores, classes, num):
        """ removes any detected object below MIN_SCORE confidence """

        f_scores, f_boxes, f_classes = [], [], []
        f_num = 0

        for i in range(num):
            if scores[i] >= MIN_SCORE:
                f_scores.append(scores[i])
                f_boxes.append(boxes[i])
                f_classes.append(int(classes[i]))
                f_num += 1
            else:
                break

        return f_boxes, f_scores, f_classes, f_num

    def load_image_into_numpy_array(self, img):
        """ converts opencv image into a numpy array """

        (im_height, im_width, im_chan) = img.shape

        return np.array(img.data).reshape((im_height, im_width, 3)).astype(np.uint8)

    def project_pixel_to_ray(self,u,v):
        """ takes in a pixel coordinate (u,v) and returns a tuple (x,y,z)
        that is a unit vector in the direction of the pixel, in the camera frame.
        This function access self.fx, self.fy, self.cx and self.cy """

        x = (u - self.cx)/self.fx
        y = (v - self.cy)/self.fy
        norm = math.sqrt(x*x + y*y + 1)
        x /= norm
        y /= norm
        z = 1.0 / norm

        return (x,y,z)

    def estimate_distance(self, thetaleft, thetaright, ranges):
        """ estimates the distance of an object in between two angles
        using lidar measurements """

        offset = int(np.pi/self.laser_angle_increment)
        # add offset and wrap to number of points. laserscan should always have the same number of points. (2*offset = len(ranges))
        # See https://github.com/StanfordASL/velodyne/blob/master/velodyne_laserscan/src/VelodyneLaserScan.cpp#L110
        leftray_indx = (offset + min(max(0, int(thetaleft/self.laser_angle_increment)), offset * 2)) % (offset * 2)
        rightray_indx = (offset + min(max(0, int(thetaright/self.laser_angle_increment)), offset * 2)) % (offset * 2)

        if leftray_indx<rightray_indx:
            meas = ranges[rightray_indx:] + ranges[:leftray_indx]
        else:
            meas = ranges[rightray_indx:leftray_indx]

        num_m = 0
        dists = []
        for m in meas:
            if m>0 and m<float('Inf'):
                dists.append(m)
                num_m += 1

        dists = np.sort(np.array(dists))
        m = min(10, num_m)
        return np.mean(dists[:m])


    def camera_callback(self, msg):
        """ callback for camera images """

        # save the corresponding laser scan
        img_laser_ranges = list(self.laser_ranges)

        try:
            img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            img_bgr8 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.camera_common(img_laser_ranges, img, img_bgr8)

    def compressed_camera_callback(self, msg):
        """ callback for camera images """

        # save the corresponding laser scan
        img_laser_ranges = list(self.laser_ranges)

        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, "passthrough")
            img_bgr8 = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.camera_common(img_laser_ranges, img, img_bgr8)

    def camera_common(self, img_laser_ranges, img, img_bgr8):
        (img_h,img_w,img_c) = img.shape

        # runs object detection in the image
        (boxes, scores, classes, num) = self.run_detection(img)

        if num > 0:
            # create list of detected objects
            detected_objects = DetectedObjectList()

            # some objects were detected
            for (box,sc,cl) in zip(boxes, scores, classes):
                ymin = int(box[0]*img_h)
                xmin = int(box[1]*img_w)
                ymax = int(box[2]*img_h)
                xmax = int(box[3]*img_w)
                xcen = int(0.5*(xmax-xmin)+xmin)
                ycen = int(0.5*(ymax-ymin)+ymin)

                cv2.rectangle(img_bgr8, (xmin,ymin), (xmax,ymax), (255,0,0), 2)

                # computes the vectors in camera frame corresponding to each sides of the box
                rayleft = self.project_pixel_to_ray(xmin,ycen)
                rayright = self.project_pixel_to_ray(xmax,ycen)

                # convert the rays to angles (with 0 poiting forward for the robot)
                thetaleft = math.atan2(-rayleft[0],rayleft[2])
                thetaright = math.atan2(-rayright[0],rayright[2])
                if thetaleft<0:
                    thetaleft += 2.*math.pi
                if thetaright<0:
                    thetaright += 2.*math.pi

                # estimate the corresponding distance using the lidar
                dist = self.estimate_distance(thetaleft,thetaright,img_laser_ranges)

                if not self.object_publishers.has_key(cl):
                    self.object_publishers[cl] = rospy.Publisher('/detector/'+self.object_labels[cl],
                        DetectedObject, queue_size=10)

                # publishes the detected object and its location
                object_msg = DetectedObject()
                object_msg.id = cl
                object_msg.name = self.object_labels[cl]
                object_msg.confidence = sc
                object_msg.distance = dist
                object_msg.thetaleft = thetaleft
                object_msg.thetaright = thetaright
                object_msg.corners = [ymin,xmin,ymax,xmax]
                self.object_publishers[cl].publish(object_msg)

                # add detected object to detected objects list
                detected_objects.objects.append(self.object_labels[cl])
                detected_objects.ob_msgs.append(object_msg)

            self.detected_objects_pub.publish(detected_objects)

        # displays the camera image
        #cv2.imshow("Camera", img_bgr8)
        #cv2.waitKey(1)

    def camera_info_callback(self, msg):
        """ extracts relevant camera intrinsic parameters from the camera_info message.
        cx, cy are the center of the image in pixel (the principal point), fx and fy are
        the focal lengths. Stores the result in the class itself as self.cx, self.cy,
        self.fx and self.fy """

        self.cx = msg.P[2]
        self.cy = msg.P[6]
        self.fx = msg.P[0]
        self.fy = msg.P[5]

    def laser_callback(self, msg):
        """ callback for thr laser rangefinder """

        self.laser_ranges = msg.ranges
        self.laser_angle_increment = msg.angle_increment

    def run(self):
        rospy.spin()

if __name__=='__main__':
    d = Detector()
    d.run()
