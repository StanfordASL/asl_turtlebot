<<<<<<< HEAD
# Karen May Wang (kmwang14@stanford.edu)
# AA274A
# 10/29/2020
import numpy as np
from utils import wrapToPi
=======
import numpy as np
from utils import wrapToPi
import rospy
from std_msgs.msg import Float64
>>>>>>> cbed96532dbfb677b5f75b40e72ba7a4a39dd643

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

<<<<<<< HEAD
=======
        #*** ADDED G.S. 10/28/20 **************************************************
        self.alpha_pub = rospy.Publisher('/controller/alpha', Float64, queue_size=1)
        self.delta_pub = rospy.Publisher('/controller/delta', Float64, queue_size=1)
        self.rho_pub   = rospy.Publisher('/controller/rho', Float64, queue_size=1)
        #***************************************************************************


>>>>>>> cbed96532dbfb677b5f75b40e72ba7a4a39dd643
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
<<<<<<< HEAD
        Outputs:
=======
        Outputs: 
>>>>>>> cbed96532dbfb677b5f75b40e72ba7a4a39dd643
            V, om: Control actions

        Hints: You'll need to use the wrapToPi function. The np.sinc function
        may also be useful, look up its documentation
        """
        ########## Code starts here ##########
<<<<<<< HEAD
        #change of coordinates so that origin is at (x_g,y_g,th_g)
        xstar = x-self.x_g
        ystar = y-self.y_g
        thstar = th-self.th_g
        #rotate coordinate so that xprime axis is along the unicycle main axis
        xprime =  np.cos(self.th_g)*xstar + np.sin(self.th_g)*ystar
        yprime = -np.sin(self.th_g)*xstar + np.cos(self.th_g)*ystar
        #rho is distance of the reference point of the unicycle w.r.t goal location
        rho = np.sqrt(xprime**2 + yprime**2)
        #alpha is angle of pointing vector to the goal w.r.t. the unicycle main axis
        alpha = np.arctan2(yprime,xprime) - thstar + np.pi
        #delta is angle of the same pointing vector w.r.t. the XN axis
        delta = alpha + thstar
        #make sure alpha and delta remain in the range [-pi,pi]
        alpha = wrapToPi(alpha)
        delta = wrapToPi(delta)

        #command zero velocities once any of these thresholds are met
        if rho <= RHO_THRES and np.abs(alpha) <= ALPHA_THRES and np.abs(delta) <= DELTA_THRES:
            V  = 0.0
            om = 0.0
        else:
            V = self.k1*rho*np.cos(alpha)
            om = self.k2*alpha + self.k1*np.sinc(alpha/np.pi)*np.cos(alpha)*(alpha + self.k3*delta)
=======
        
        
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
>>>>>>> cbed96532dbfb677b5f75b40e72ba7a4a39dd643
        ########## Code ends here ##########

        # apply control limits
        V = np.clip(V, -self.V_max, self.V_max)
<<<<<<< HEAD
=======
                                
>>>>>>> cbed96532dbfb677b5f75b40e72ba7a4a39dd643
        om = np.clip(om, -self.om_max, self.om_max)

        return V, om
