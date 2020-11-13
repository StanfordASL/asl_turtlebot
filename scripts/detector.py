#!/usr/bin/env python

import rospy
import os
# watch out on the order for the next two imports lol
from tf import TransformListener
try:
    import tensorflow as tf
except:
    pass
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from asl_turtlebot.msg import DetectedObject
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math

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

class DetectorParams:

    def __init__(self, verbose=False):

        # Set to True to use tensorflow and a conv net.
        # False will use a very simple color thresholding to detect stop signs only.
        self.use_tf = rospy.get_param("use_tf")

        # Path to the trained conv net
        #model_path = rospy.get_param("~model_path", "../tfmodels/stop_signs_gazebo.pb")
        model_path = rospy.get_param("~model_path", "../tfmodels/ssd_mobilenet_v1_coco.pb")
        label_path = rospy.get_param("~label_path", "../tfmodels/coco_labels.txt")
        self.model_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), model_path)
        self.label_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), label_path)

        # Minimum score for positive detection
        self.min_score = rospy.get_param("~min_score", 0.5)

        if verbose:
            print("DetectorParams:")
            print("    use_tf = {}".format(self.use_tf))
            print("    model_path = {}".format(model_path))
            print("    label_path = {}".format(label_path))
            print("    min_score = {}".format(self.min_score))

class Detector:

    def __init__(self):
        rospy.init_node('turtlebot_detector', anonymous=True)
        self.params = DetectorParams()
        self.bridge = CvBridge()

        if self.params.use_tf:
            self.detection_graph = tf.Graph()
            with self.detection_graph.as_default():
                od_graph_def = tf.GraphDef()
                with tf.gfile.GFile(self.params.model_path, 'rb') as fid:
                    serialized_graph = fid.read()
                    od_graph_def.ParseFromString(serialized_graph)
                    tf.import_graph_def(od_graph_def,name='')
                self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                self.d_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                self.d_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                self.d_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                self.num_d = self.detection_graph.get_tensor_by_name('num_detections:0')
            self.sess = tf.Session(graph=self.detection_graph)

        # camera and laser parameters that get updated
        self.cx = 0.
        self.cy = 0.
        self.fx = 1.
        self.fy = 1.
        self.laser_ranges = []
        self.laser_angle_increment = 0.01 # this gets updated

        self.object_publishers = {}
        self.object_labels = load_object_labels(self.params.label_path)

        self.tf_listener = TransformListener()
        rospy.Subscriber('/camera/image_raw', Image, self.camera_callback, queue_size=1)
        rospy.Subscriber('/camera/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)

    def run_detection(self, img):
        """ runs a detection method in a given image """

        image_np = self.load_image_into_numpy_array(img)
        image_np_expanded = np.expand_dims(image_np, axis=0)

        if self.params.use_tf:
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
            if scores[i] >= self.params.min_score:
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

    def project_pixel_to_ray(self, u, v):
        """ takes in a pixel coordinate (u,v) and returns a tuple (x,y,z)
        that is a unit vector in the direction of the pixel, in the camera frame """

        ########## Code starts here ##########
        # See Lecture 9, 9.1.1
        # Since it is in the camera frame (x, y, z) = (Xc, Yc, Zc)
        # See equation 6, set Zc =  alpha
        # TODO: Compute x, y, z.
        #z = self.fx
        #x = ((u - self.cx)/self.fx)*z
        #y = ((v - self.cy)/self.fy)*z
        x_unnormalized =  ((u - self.cx)/self.fx)
        y_unnormalized =  ((v - self.cy)/self.fy)
        z_unnormalized = 1.0
        pixel_dir = np.array([x_unnormalized,y_unnormalized,z_unnormalized])
        pixel_dir_norm = pixel_dir/np.linalg.norm(pixel_dir)
        x = pixel_dir_norm[0]
        y = pixel_dir_norm[1]
        z = pixel_dir_norm[2]
        ########## Code ends here ##########

        return x, y, z

    def estimate_distance(self, thetaleft, thetaright, ranges):
        """ estimates the distance of an object in between two angles
        using lidar measurements """

        leftray_indx = min(max(0,int(thetaleft/self.laser_angle_increment)),len(ranges))
        rightray_indx = min(max(0,int(thetaright/self.laser_angle_increment)),len(ranges))

        if leftray_indx<rightray_indx:
            meas = ranges[rightray_indx:] + ranges[:leftray_indx]
        else:
            meas = ranges[rightray_indx:leftray_indx]

        num_m, dist = 0, 0
        for m in meas:
            if m>0 and m<float('Inf'):
                dist += m
                num_m += 1
        if num_m>0:
            dist /= num_m

        return dist

    def camera_callback(self, msg):
        """ callback for camera images """

        # save the corresponding laser scan
        img_laser_ranges = list(self.laser_ranges)

        try:
            img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            img_bgr8 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        (img_h,img_w,img_c) = img.shape

        # runs object detection in the image
        (boxes, scores, classes, num) = self.run_detection(img)

        if num > 0:
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

        # displays the camera image
        cv2.imshow("Camera", img_bgr8)
        cv2.waitKey(1)

    def camera_info_callback(self, msg):
        """ extracts relevant camera intrinsic parameters from the camera_info message.
        cx, cy are the center of the image in pixel (the principal point), fx and fy are
        the focal lengths. """

        ########## Code starts here ##########
        # TODO: Extract camera intrinsic parameters.

	# CameraInfo.msg.K [1 x 9] array http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html

        self.cx = msg.K[2]
        self.cy = msg.K[5]
        self.fx = msg.K[0]
        self.fy = msg.K[4]

        ########## Code ends here ##########

    def laser_callback(self, msg):
        """ callback for thr laser rangefinder """

        self.laser_ranges = msg.ranges
        self.laser_angle_increment = msg.angle_increment

    def run(self):
        rospy.spin()

if __name__=='__main__':
    d = Detector()
    d.run()
