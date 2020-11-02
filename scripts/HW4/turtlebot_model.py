# Karen May Wang (kmwang14@stanford.edu)
# AA274A
# 10/29/2020
import numpy as np
import pdb
EPSILON_OMEGA = 1e-3

def compute_dynamics(xvec, u, dt, compute_jacobians=True):
    """
    Compute Turtlebot dynamics (unicycle model).

    Inputs:
                     xvec: np.array[3,] - Turtlebot state (x, y, theta).
                        u: np.array[2,] - Turtlebot controls (V, omega).
        compute_jacobians: bool         - compute Jacobians Gx, Gu if true.
    Outputs:
         g: np.array[3,]  - New state after applying u for dt seconds.
        Gx: np.array[3,3] - Jacobian of g with respect to xvec.
        Gu: np.array[3,2] - Jacobian of g with respect to u.
    """
    ########## Code starts here ##########
    # TODO: Compute g, Gx, Gu
    # HINT: To compute the new state g, you will need to integrate the dynamics of x, y, theta
    # HINT: Since theta is changing with time, try integrating x, y wrt d(theta) instead of dt by introducing om
    # HINT: When abs(om) < EPSILON_OMEGA, assume that the theta stays approximately constant ONLY for calculating the next x, y
    #       New theta should not be equal to theta. Jacobian with respect to om is not 0.
    x  = xvec[0]
    y  = xvec[1]
    th = xvec[2]
    V  = u[0] 
    om = u[1] 
    #Compute new state g by integrating dynamics of x, y, theta
    #We assume that V, om are constant over dt
    g = np.zeros(3)
    #if abs(om) < EPSILON_OMEGA, assume theta is constant for calculating x, y
    #update om first
    g[2] = th + om*dt
    if abs(om) < EPSILON_OMEGA:
       #g[0] = x + V*np.cos(g[2])*dt 
       #g[1] = y + V*np.sin(g[2])*dt
       g[0] = x + (V/2.0)*(np.cos(th) + np.cos(g[2]))*dt
       g[1] = y + (V/2.0)*(np.sin(th) + np.sin(g[2]))*dt
    else:
       g[0] = x + (V/om)*(np.sin(g[2]) - np.sin(th)) 
       g[1] = y - (V/om)*(np.cos(g[2]) - np.cos(th))
    #Compute Jacobian with respect to state variables
    # [dg[0]/dx, dg[0]/dy, dg[0]/dth]
    # [dg[1]/dx, dg[1]/dy, dg[1]/dth]
    # [dg[2]/dx, dg[2]/dy, dg[2]/dth]
    if abs(om) < EPSILON_OMEGA:
       Gx = np.array([[1.0,   0.0,  -V*np.sin(g[2])*dt],
                      [0.0,    1.0,  V*np.cos(g[2])*dt],
                      [0.0,    0.0,  1.0]])
       #Gx = np.array([[1.0,   0.0,  -(V/2.0)*(np.sin(g[2]))*dt],
       #               [0.0,    1.0,  (V/2.0)*(np.cos(g[2]))*dt],
       #               [0.0,    0.0,  1.0]])
    else:
       Gx = np.array([[1.0,   0.0,  (V/om)*(np.cos(g[2])-np.cos(th))],
                      [0.0,    1.0, (V/om)*(np.sin(g[2])-np.sin(th))],
                      [0.0,    0.0,  1.0]])
    #Compute Jacobian with respect to control variables
    # [dg[0]/dV, dg[0]/dom]
    # [dg[1]/dV, dg[1]/dom]
    # [dg[2]/dV, dg[2]/dom]
    if abs(om) < EPSILON_OMEGA:
       #Gu = np.array([[ np.cos(g[2])*dt, -V*np.sin(g[2])*dt**2],
       #               [np.sin(g[2])*dt,   V*np.cos(g[2])*dt**2],
       #               [0,         dt]])
       Gu = np.array([[ 0.5*(np.cos(th) + np.cos(g[2]))*dt, -(V/2.0)*np.sin(g[2])*dt**2],
                      [ 0.5*(np.sin(th) + np.sin(g[2]))*dt,  (V/2.0)*np.cos(g[2])*dt**2],
                      [0,         dt]])
    else:
       Gu = np.array([[ (1./om)*(np.sin(g[2])-np.sin(th)), -(V/om**2)*(np.sin(g[2])-np.sin(th)) + (V/om)*np.cos(g[2])*dt],
                      [-(1./om)*(np.cos(g[2])-np.cos(th)),  (V/om**2)*(np.cos(g[2])-np.cos(th)) + (V/om)*np.sin(g[2])*dt],
                      [0,         dt]])
    ########## Code ends here ##########

    if not compute_jacobians:
        return g

    return g, Gx, Gu

def transform_line_to_scanner_frame(line, x, tf_base_to_camera, compute_jacobian=True):
    """
    Given a single map line in the world frame, outputs the line parameters
    in the scanner frame so it can be associated with the lines extracted
    from the scanner measurements.

    Input:
                     line: np.array[2,] - map line (alpha, r) in world frame.
                        x: np.array[3,] - pose of base (x, y, theta) in world frame.
        tf_base_to_camera: np.array[3,] - pose of camera (x, y, theta) in base frame.
         compute_jacobian: bool         - compute Jacobian Hx if true.
    Outputs:
         h: np.array[2,]  - line parameters in the scanner (camera) frame.
        Hx: np.array[2,3] - Jacobian of h with respect to x.
    """
    alpha, r = line

    ########## Code starts here ##########
    # TODO: Compute h, Hx
    # HINT: Calculate the pose of the camera in the world frame (x_cam, y_cam, th_cam), a rotation matrix may be useful.
    # HINT: To compute line parameters in the camera frame h = (alpha_in_cam, r_in_cam), 
    #       draw a diagram with a line parameterized by (alpha,r) in the world frame and 
    #       a camera frame with origin at x_cam, y_cam rotated by th_cam wrt to the world frame
    # HINT: What is the projection of the camera location (x_cam, y_cam) on the line r? 
    # HINT: To find Hx, write h in terms of the pose of the base in world frame (x_base, y_base, th_base)
    #line in world frame
    xw = x[0]
    yw = x[1]
    thw = x[2]
    xcam = tf_base_to_camera[0]
    ycam = tf_base_to_camera[1]
    thcam = tf_base_to_camera[2]
    h = np.zeros(2)
    h[0] = alpha - (thw + thcam)
    h[1] = r- (np.cos(alpha)*(xw + xcam*np.cos(thw)-ycam*np.sin(thw)) + np.sin(alpha)*(yw+xcam*np.sin(thw)+ycam*np.cos(thw)) )
    #Jacobian is
    #dh[0]/dx dh[0]/dy dh[0]dth
    #dh[1]/dx dh[1]/dy dh[1]dth
    Hx = np.array([[0.0,            0.0,            -1.0],
                   [-np.cos(alpha), -np.sin(alpha), -np.cos(alpha)*(-xcam*np.sin(thw)-ycam*np.cos(thw)) - np.sin(alpha)*(xcam*np.cos(thw)-ycam*np.sin(thw)) ]])
    ########## Code ends here ##########

    if not compute_jacobian:
        return h

    return h, Hx


def normalize_line_parameters(h, Hx=None):
    """
    Ensures that r is positive and alpha is in the range [-pi, pi].

    Inputs:
         h: np.array[2,]  - line parameters (alpha, r).
        Hx: np.array[2,n] - Jacobian of line parameters with respect to x.
    Outputs:
         h: np.array[2,]  - normalized parameters.
        Hx: np.array[2,n] - Jacobian of normalized line parameters. Edited in place.
    """
    alpha, r = h
    if r < 0:
        alpha += np.pi
        r *= -1
        if Hx is not None:
            Hx[1,:] *= -1
    alpha = (alpha + np.pi) % (2*np.pi) - np.pi
    h = np.array([alpha, r])

    if Hx is not None:
        return h, Hx
    return h
