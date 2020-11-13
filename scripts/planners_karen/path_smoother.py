# Karen May Wang (kmwang14@stanford.edu)
# AA274A
# 10/29/2020
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
    N = len(path)
    t_old = np.zeros(N)
    #compute time it takes to go through path
    #estimate the time for each of the points assuming we travel at fixed speed
    #Vdes along each segment
    for i in range(N-1):
        x1, y1 = path[i]
        x2, y2 = path[i+1]
        distance = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        dt_old = distance/V_des
        t_old[i+1] = t_old[i] + dt_old
    #resample this time to the new smoothed trajectory time
    t_smoothed = np.arange(0.0,t_old[-1],dt)
    N_new = len(t_smoothed)
    traj_smoothed = np.zeros((N_new,7))
    #find the smoothed paths
    x_old, y_old = zip(*path)
    x_smoothed_knots = scipy.interpolate.splrep(t_old,x_old,k=3,s=alpha)
    y_smoothed_knots = scipy.interpolate.splrep(t_old,y_old,k=3,s=alpha)

    x_smoothed = scipy.interpolate.splev(t_smoothed,x_smoothed_knots)
    y_smoothed = scipy.interpolate.splev(t_smoothed,y_smoothed_knots)
    xd_smoothed = scipy.interpolate.splev(t_smoothed,x_smoothed_knots,der=1)
    yd_smoothed = scipy.interpolate.splev(t_smoothed,y_smoothed_knots,der=1)
    xdd_smoothed = scipy.interpolate.splev(t_smoothed,x_smoothed_knots,der=2)
    ydd_smoothed = scipy.interpolate.splev(t_smoothed,y_smoothed_knots,der=2)
    th_smoothed = np.arctan2(yd_smoothed,xd_smoothed)

    traj_smoothed[:,0] = x_smoothed
    traj_smoothed[:,1] = y_smoothed
    traj_smoothed[:,2] = th_smoothed
    traj_smoothed[:,3] = xd_smoothed
    traj_smoothed[:,4] = yd_smoothed
    traj_smoothed[:,5] = xdd_smoothed
    traj_smoothed[:,6] = ydd_smoothed
    ########## Code ends here ##########

    return traj_smoothed, t_smoothed
