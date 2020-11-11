import numpy as np
from utils import wrapToPi

# command zero velocities once we are this close to the goal
RHO_THRES = 0.05
ALPHA_THRES = 0.1
DELTA_THRES = 0.1

class HeadingController:
    """
    pose stabilization controller
    """
    def __init__(self, kp, om_max=1):
        self.kp = kp
        self.om_max = om_max

    def load_goal(self, th_g):
        """
        loads in a new goal position
        """
        self.th_g = th_g

    def compute_control(self, x, y, th, t):
        err = wrapToPi(self.th_g - th)
        om = self.kp*err

        # apply control limits
        V = 0
        om = np.clip(om, -self.om_max, self.om_max)

        return V, om
