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
    x0=[]
    y0=[]
    t=[0]
    traj_smoothed=[]
        
    for i in range(len(path)):
        x0.append(path[i][0])
        y0.append(path[i][1])
    
    for i in range (len(path)-1):
        norm=np.sqrt((path[i+1][0]-path[i][0])**2+(path[i+1][1]-path[i][1])**2)
        t.append(t[-1]+norm/V_des)
   
    
    t_smoothed=np.arange(0,t[-1],dt)
    #print(t,t_smoothed)
       
    tck_x=scipy.interpolate.splrep(np.array(t),np.array(x0),k=3,s=alpha)
    x=scipy.interpolate.splev(t_smoothed, tck_x,der=0)
   
    tck_y=scipy.interpolate.splrep(np.array(t),np.array(y0),k=3,s=alpha)
    y=scipy.interpolate.splev(t_smoothed, tck_y,der=0)
    
    dx=scipy.interpolate.splev(t_smoothed, tck_x,der=1)
    dy=scipy.interpolate.splev(t_smoothed, tck_y,der=1)
    
    ddx=scipy.interpolate.splev(t_smoothed, tck_x,der=2)
    ddy=scipy.interpolate.splev(t_smoothed, tck_y,der=2)
    
    th=np.arctan2(dy,dx)
    
    for i in range (len(x)):
        traj_smoothed.append([x[i],y[i],th[i],dx[i],dy[i],ddx[i],ddy[i]])
    traj_smoothed=np.array(traj_smoothed)
    ########## Code ends here ##########

    return traj_smoothed, t_smoothed
