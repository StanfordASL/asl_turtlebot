import rospy
from visualization_msgs.msg import Marker
from asl_turtlebot.msg import DetectedObjectList
#this node is subscribed to the detector and publishes marker information,  which  we will  view  the topic  in rviz 
#This  node  publishes  marker info to rviz and  a  DetectedObjectList (list of strings of  objects and list of Detected Objects)
class MarkerPublisher:
    def __init__(self):
        rospy.init_node('turtlebot_marker_tracker', anonymous=True)
        #initialize variables
        self.markerID = 0

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

    def object_callback(self, data):
        #when an  object is detected, we compute the location of the object and  store it
        NAME  = data.name
        dist  = data.distance

        #compute  location of food detected
        th_diff = 0.5*(data.thetaleft - data.thetaright)
        th_center = data.thetaleft -  th_diff
        food_loc_x  = self.x + dist*np.cos(th_center)
        food_loc_y  = self.y + dist*np.sin(th_center)
        LOCATIONS  =  (food_loc_x,food_loc_y)

        #when object detected, create a  marker  publisher if it  isn't already in the list
        if data.name not in self.marker_publishers:
            self.marker_publishers[data.name] = rospy.Publisher('/marker_topic_'+str(self.markerID)+'/', Marker, queue_size=10)
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time()
            marker.id = self.markerID
            marker.type = 2 # sphere
            marker.pose.position.x = LOCATIONS[0]
            marker.pose.position.y = LOCATIONS[1]
            marker.pose.position.z = 1
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
        self.msg_detected_obj = DetectedObjectList()
        #add food vendor to list if not  there
        if data.name not in self.msg_detected_obj.objects:
            self.msg_detected_obj.objects.append(data.name)
            self.msg_detected_obj.ob_msgs.append(data)

    def run():
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
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
