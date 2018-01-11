#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, UInt16

class Success:

    def __init__(self):
        rospy.init_node('success', anonymous=True)
        rospy.Subscriber('success', Bool, self.success_callback)
        self.flag_pub = rospy.Publisher('servo', UInt16, queue_size=10)
        self.flag_pub.publish(0)

    def success_callback(self, msg):
        if msg.data:
            self.flag_pub.publish(170)
        else:
            self.flag_pub.publish(10)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    s = Success()
    s.run()