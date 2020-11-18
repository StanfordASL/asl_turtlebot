import numpy as np
from utils import wrapToPi
import rospy
from std_msgs.msg import Float64

# command zero velocities once we are this close to the goal
RHO_THRES = 0.05
ALPHA_THRES = 0.1
DELTA_THRES = 0.1

class PoseController:
    """ Pose stabilization controller """
    def __init__(self, k1, k2, k3, V_max=0.5, om_max=1):
        self.k1 = k1
        self.k2 = k2
        self.k3 = k3

        self.V_max = V_max
        self.om_max = om_max

        #*** ADDED G.S. 10/28/20 **************************************************
        self.alpha_pub = rospy.Publisher('/controller/alpha', Float64, queue_size=1)
        self.delta_pub = rospy.Publisher('/controller/delta', Float64, queue_size=1)
        self.rho_pub   = rospy.Publisher('/controller/rho', Float64, queue_size=1)
        #***************************************************************************


    def load_goal(self, x_g, y_g, th_g):
        """ Loads in a new goal position """
        self.x_g = x_g
        self.y_g = y_g
        self.th_g = th_g

    def compute_control(self, x, y, th, t):
        """
        Inputs:
            x,y,th: Current state
            t: Current time (you shouldn't need to use this)
        Outputs: 
            V, om: Control actions

        Hints: You'll need to use the wrapToPi function. The np.sinc function
        may also be useful, look up its documentation
        """
        ########## Code starts here ##########
        
        
        xg = self.x_g
        yg = self.y_g
        thg = self.th_g

        # compute range to goal

        rho = np.sqrt(  ( x - xg )**2.0 + ( y - yg )**2.0 ) 

        # compute absolute bearing to target

        beta = np.arctan2( ( yg - y ), ( xg - x ) )

        # compute relative bearing to target

        #alpha = utils.wrapToPi( beta - theta )
        alpha = wrapToPi( beta - th )

        # compute the polar-coordinate angle delta (in desired position frame)

        #delta = utils.wrapToPi( beta - th_g )
        delta = wrapToPi( beta - thg )

        # compute the controls

        V = self.k1 * rho * np.cos( alpha )
                
        tmp = np.sinc( alpha / np.pi ) * np.cos( alpha )
        om = self.k2 * alpha + self.k1 * tmp * ( alpha + self.k3 * delta )

        #print(' V = ', V )
        #print(' om = ', om )
        """
        #Publishing our computed alpha, delta, and rho values for navigation
        self.alpha_pub = rospy.Publisher('controller/alpha', Float64, queue_size=1)
        self.delta_pub = rospy.Publisher('controller/delta', Float64, queue_size=1)
        self.rho_pub   = rospy.Publisher('controller/rho', Float64, queue_size=1)
        rospy.init_node('pose_controller', anonymous=True)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            alpha_ros = Float64()
            self.alpha_pub.publish(alpha_ros)
            delta_ros = Float64()
            self.delta_pub.publish(delta_ros)
            rho_ros = Float64()
            self.rho_pub.publish(rho_ros)
            rate.sleep() 
        """             
        self.alpha_pub.publish(Float64(alpha))
	self.delta_pub.publish(Float64(delta))
	self.rho_pub.publish(Float64(rho))

        #print('PUBLISHED alpha delta rho')
        ########## Code ends here ##########

        # apply control limits
        V = np.clip(V, -self.V_max, self.V_max)
                                
        om = np.clip(om, -self.om_max, self.om_max)

        return V, om
