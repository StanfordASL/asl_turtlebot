import numpy as np

EPSILON_OMEGA = 1e-3

def compute_dynamics(x, u, dt, compute_jacobians=True):
    """
    Compute Turtlebot dynamics (unicycle model).

    Inputs:
                        x: np.array[3,] - Turtlebot state (x, y, theta).
                        u: np.array[2,] - Turtlebot controls (V, omega).
        compute_jacobians: bool         - compute Jacobians Gx, Gu if true.
    Outputs:
         g: np.array[3,]  - New state after applying u for dt seconds.
        Gx: np.array[3,3] - Jacobian of g with respect to x.
        Gu: np.array[3,2] - Jacobian of g with respect ot u.
    """
    ########## Code starts here ##########
    # TODO: Compute g, Gx, Gu
    x_t = x[0]
    y_t = x[1]
    theta_t = x[2]
    V_t = u[0]
    om_t = u[1]

    g = np.zeros(3)
    Gx = np.zeros((3,3))
    Gu = np.zeros((3,2))

    g[2] = om_t*dt + theta_t
    if (abs(om_t) >= EPSILON_OMEGA):
        g[0] = V_t*np.sin(g[2])/om_t - V_t*np.sin(theta_t)/om_t + x_t
        g[1] = -V_t*np.cos(g[2])/om_t + V_t*np.cos(theta_t)/om_t + y_t
        Gx[0,0] = 1.
        Gx[1,1] = 1.
        Gx[2,2] = 1.
        Gx[0,2] = V_t*np.cos(g[2])/om_t - V_t*np.cos(theta_t)/om_t
        Gx[1,2] = V_t*np.sin(g[2])/om_t - V_t*np.sin(theta_t)/om_t

        Gu = np.array([[np.sin(g[2])/om_t - np.sin(theta_t)/om_t, -V_t*np.sin(g[2])/(om_t**2) + V_t*np.cos(g[2])*dt/om_t + V_t*np.sin(theta_t)/(om_t**2)],
                       [-np.cos(g[2])/om_t + np.cos(theta_t)/om_t, V_t*np.cos(g[2])/(om_t**2) + V_t*np.sin(g[2])*dt/om_t - V_t*np.cos(theta_t)/(om_t**2)],
                       [0, dt]])
    else:
        g[0] = V_t*np.cos(g[2])*dt + x_t
        g[1] = V_t*np.sin(g[2])*dt + y_t

        Gx[0,0] = 1.
        Gx[1,1] = 1.
        Gx[2,2] = 1.
        Gx[0,2] = -V_t*np.sin(g[2])*dt
        Gx[1,2] = V_t*np.cos(g[2])*dt
        '''
        Gu = np.array([[np.cos(g[2])*dt, -V_t*np.cos(g[2])*dt/(2*om_t) - V_t*np.sin(g[2])*dt**2],
                       [np.sin(g[2])*dt, -V_t*np.sin(g[2])*dt/(2*om_t) + V_t*np.cos(g[2])*dt**2],
                       [0, dt]])
        '''
        Gu = np.array([[np.cos(g[2])*dt, -0.5*V_t*np.sin(g[2])*dt**2],
                       [np.sin(g[2])*dt, 0.5*V_t*np.cos(g[2])*dt**2],
                       [0, dt]])

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

    rot_mat = np.array([[np.cos(x[2]), -np.sin(x[2])],
                          [np.sin(x[2]), np.cos(x[2])]])
    rotate_cw = np.matmul(rot_mat, np.array([tf_base_to_camera[0], tf_base_to_camera[1]]))
    x_cam_base = rotate_cw[0]
    y_cam_base = rotate_cw[1]

    x_cam_w = x[0] + x_cam_base
    y_cam_w = x[1] + y_cam_base

    a = np.cos(alpha)
    b = np.sin(alpha)
    c = -r
    #x_nearest = (b*(b*x_cam_w - a*y_cam_w) - a*c)/(a**2 + b**2)
    #y_nearest = (a*(-b*x_cam_w + a*y_cam_w) - b*c)/(a**2 + b**2)
    #dist = (a*x_cam_w + b*y_cam_w +c)/np.sqrt(a**2 + b**2)
    th = x[2]
    th_cam = tf_base_to_camera[2]

    dist = r - x_cam_w*np.cos(alpha) - y_cam_w*np.sin(alpha)

    h = np.zeros(2)
    #h[0] = np.pi + alpha - th - th_cam

    h[0] = alpha - th - th_cam
    h[1] = dist

    Hx = np.array([[0, 0, -1],
                  [-np.cos(alpha), -np.sin(alpha), np.cos(alpha)*tf_base_to_camera[0]*np.sin(th) + np.cos(alpha)*tf_base_to_camera[1]*np.cos(th) - np.sin(alpha)*tf_base_to_camera[0]*np.cos(th) + np.sin(alpha)*tf_base_to_camera[1]*np.sin(th)]])
    '''
    Hx = np.array([[0, 0, -1],
                  [np.cos(alpha), np.sin(alpha), -np.cos(alpha)*tf_base_to_camera[0]*np.sin(th) - np.cos(alpha)*tf_base_to_camera[1]*np.cos(th) + np.sin(alpha)*tf_base_to_camera[0]*np.cos(th) - np.sin(alpha)*tf_base_to_camera[1]*np.sin(th)]])
    '''

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
