import numpy as np
import scipy.interpolate

def compute_smoothed_traj(path, V_des, alpha, dt):
    """
    Fit cubic spline to a path and generate a resulting trajectory for our
    wheeled robot.

    Inputs:
        path (np.array [N,2]): Initial path
        V_des (float): Desired nominal velocity, used as a heuristic to assign nominal
            times to points in the initial path
        alpha (float): Smoothing parameter (see documentation for
            scipy.interpolate.splrep)
        dt (float): Timestep used in final smooth trajectory
    Outputs:
        traj_smoothed (np.array [N,7]): Smoothed trajectory
        t_smoothed (np.array [N]): Associated trajectory times
    Hint: Use splrep and splev from scipy.interpolate
    """
    ########## Code starts here ##########
    tck, u = scipy.interpolate.splprep(np.array(path).transpose(), s=alpha)
    u = np.linspace(0, 1.00, 1000)
    x, y = scipy.interpolate.splev(u, tck)
    path_len = np.sum(np.sqrt((x[:-1] - x[1:])**2 + (y[:-1] - y[1:])**2))
    t_move = path_len / V_des
    t_smoothed = np.append(np.arange(0, t_move, dt), t_move)
    u_new = t_smoothed/t_move
    x, y = scipy.interpolate.splev(u_new, tck)
    xd, yd = scipy.interpolate.splev(u_new, tck, der=1) / t_move
    xdd, ydd = scipy.interpolate.splev(u_new, tck, der=2) / t_move**2
    th = np.arctan2(yd,xd)
#    xd = yd = xdd = ydd = th = np.zeros(len(x))

    traj_smoothed = np.array([x, y, th, xd, yd, xdd, ydd]).transpose()

    ########## Code ends here ##########
    ########## Code ends here ##########

    return traj_smoothed, t_smoothed
