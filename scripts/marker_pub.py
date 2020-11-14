#!/usr/bin/env python
import rospy
import tf
from visualization_msgs.msg import Marker
from asl_turtlebot.msg import DetectedObject,DetectedObjectList
from gazebo_msgs.msg import ModelStates
import numpy as np
from utils import wrapToPi
#this node is subscribed to the detector and publishes marker information,  which  we will  view  the topic  in rviz 
#This  node  publishes  marker info to rviz and  a  DetectedObjectList (list of strings of  objects and list of Detected Objects)

#threshold at which we store the detected object
CONFIDENCE = 0.85

class MarkerPublisher:
    def __init__(self):
        rospy.init_node('turtlebot_marker_tracker', anonymous=True)
        #initialize variables
        self.markerID = 0
        self.detected_obj_names = []
        self.detected_obj = []
        self.msg_detected_obj = None       
 
        #robot state
        self.x = 0
        self.y = 0
        self.theta = 0        
        self.trans_listener = tf.TransformListener()

        #marker publishers
        self.marker_publishers = {}
        self.marker_info = {}
        
        #list of detected object publishers
        self.ob_list_pub  = rospy.Publisher('/detected_objects_list', DetectedObjectList,  queue_size=10)
        self.msg_detected_obj = None

        #subscribers
        rospy.Subscriber('/detector/donut', DetectedObject, self.object_callback)
        rospy.Subscriber('/detector/apple', DetectedObject, self.object_callback)
        rospy.Subscriber('/detector/pizza', DetectedObject, self.object_callback)
        rospy.Subscriber('/detector/banana', DetectedObject, self.object_callback)
        rospy.Subscriber('/detector/orange', DetectedObject, self.object_callback)
        rospy.Subscriber('/detector/broccoli', DetectedObject, self.object_callback)
        rospy.Subscriber('/detector/cake', DetectedObject, self.object_callback)

    def object_callback(self, data):
        #when an  object is detected, we compute the location of the object and  store it
        NAME  = data.name
        dist  = data.distance
        confidence = data.confidence

        #compute  location of food detected
        th_diff = 0.5*(wrapToPi(data.thetaleft) - wrapToPi(data.thetaright))
        th_center = wrapToPi(data.thetaleft) -  th_diff
        th_loc = self.theta + th_center
        food_loc_x  = self.x + dist*np.cos(th_loc)
        food_loc_y  = self.y + dist*np.sin(th_loc)
        LOCATIONS  =  (food_loc_x,food_loc_y)

        if data.name not in self.marker_publishers and confidence > CONFIDENCE:
            self.marker_publishers[data.name] = rospy.Publisher('/marker_topic_'+str(self.markerID)+'/', Marker, queue_size=10)
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time()
            marker.id = self.markerID
            marker.type = 2 # sphere
            marker.pose.position.x = LOCATIONS[0]
            marker.pose.position.y = LOCATIONS[1]
            marker.pose.position.z = 0.1
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0 # Don't forget to set the alpha!
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            self.marker_info[data.name] =  marker
            self.markerID += 1

        #publish  the  detected  objects  list and  its locations
        #add food vendor to list if not  there
        if data.name not in self.detected_obj_names and confidence > CONFIDENCE:
            self.detected_obj_names.append(data.name)
            self.detected_obj.append(data)

            self.msg_detected_obj = DetectedObjectList()
            self.msg_detected_obj.objects = self.detected_obj_names
            self.msg_detected_obj.ob_msgs = self.detected_obj

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            #get state info to update self.x, self.y, self.theta
            try:
                (translation,rotation) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
                rospy.loginfo("Self.theta is {:.2f}".format(self.theta))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.loginfo("Waiting  for state info")
                print e
                pass

            #continuously publish markers 
            if len(self.marker_publishers) != 0:
                for marker_name in self.marker_publishers:
                    self.marker_publishers[marker_name].publish(self.marker_info[marker_name])
            #print('Published marker!')

            #continuously publish list of detected objects and  locations
            if self.msg_detected_obj is not None:
                 self.ob_list_pub.publish(self.msg_detected_obj)
            rate.sleep()


if __name__ == '__main__':
    try:
        marker = MarkerPublisher()
        marker.run()
    except rospy.ROSInterruptException:
        pass
