import numpy as np
import scipy.linalg    # you may find scipy.linalg.block_diag useful
import turtlebot_model as tb

class Ekf(object):
    """
    Base class for EKF Localization and SLAM.

    Usage:
        ekf = EKF(x0, Sigma0, R)
        while True:
            ekf.transition_update(u, dt)
            ekf.measurement_update(z, Q)
            localized_state = ekf.x
    """

    def __init__(self, x0, Sigma0, R):
        """
        EKF constructor.

        Inputs:
                x0: np.array[n,]  - initial belief mean.
            Sigma0: np.array[n,n] - initial belief covariance.
                 R: np.array[2,2] - control noise covariance (corresponding to dt = 1 second).
        """
        self.x = x0  # Gaussian belief mean
        self.Sigma = Sigma0  # Gaussian belief covariance
        self.R = R  # Control noise covariance (corresponding to dt = 1 second)

    def transition_update(self, u, dt):
        """
        Performs the transition update step by updating (self.x, self.Sigma).

        Inputs:
             u: np.array[2,] - zero-order hold control input.
            dt: float        - duration of discrete time step.
        Output:
            None - internal belief state (self.x, self.Sigma) should be updated.
        """
        g, Gx, Gu = self.transition_model(u, dt)

        ########## Code starts here ##########
        # TODO: Update self.x, self.Sigma.
        self.x = g
        self.Sigma = np.matmul(np.matmul(Gx,self.Sigma),Gx.T) + dt*np.matmul(np.matmul(Gu,self.R),Gu.T)

        ########## Code ends here ##########

    def transition_model(self, u, dt):
        """
        Propagates exact (nonlinear) state dynamics.

        Inputs:
             u: np.array[2,] - zero-order hold control input.
            dt: float        - duration of discrete time step.
        Outputs:
             g: np.array[n,]  - result of belief mean propagated according to the
                                system dynamics with control u for dt seconds.
            Gx: np.array[n,n] - Jacobian of g with respect to belief mean self.x.
            Gu: np.array[n,2] - Jacobian of g with respect to control u.
        """
        raise NotImplementedError("transition_model must be overriden by a subclass of EKF")

    def measurement_update(self, z_raw, Q_raw):
        """
        Updates belief state according to the given measurement.

        Inputs:
            z_raw: np.array[2,I]   - matrix of I columns containing (alpha, r)
                                     for each line extracted from the scanner
                                     data in the scanner frame.
            Q_raw: [np.array[2,2]] - list of I covariance matrices corresponding
                                     to each (alpha, r) column of z_raw.
        Output:
            None - internal belief state (self.x, self.Sigma) should be updated.
        """
        z, Q, H = self.measurement_model(z_raw, Q_raw)
        if z is None:
            # Don't update if measurement is invalid
            # (e.g., no line matches for line-based EKF localization)
            return

        ########## Code starts here ##########
        # TODO: Update self.x, self.Sigma.

        S_t = np.matmul(np.matmul(H, self.Sigma), H.T) + Q
        K_t = np.matmul(np.matmul(self.Sigma, H.T), np.linalg.inv(S_t))
        self.x = self.x + np.dot(K_t, z).flatten()
        self.Sigma = self.Sigma - np.matmul(np.matmul(K_t, S_t), K_t.T)

        ########## Code ends here ##########

    def measurement_model(self, z_raw, Q_raw):
        """
        Converts raw measurements into the relevant Gaussian form (e.g., a
        dimensionality reduction). Also returns the associated Jacobian for EKF
        linearization.

        Inputs:
            z_raw: np.array[2,I]   - I lines extracted from scanner data in
                                     columns representing (alpha, r) in the scanner frame.
            Q_raw: [np.array[2,2]] - list of I covariance matrices corresponding
                                     to each (alpha, r) column of z_raw.
        Outputs:
            z: np.array[2K,]   - measurement mean.
            Q: np.array[2K,2K] - measurement covariance.
            H: np.array[2K,n]  - Jacobian of z with respect to the belief mean self.x.
        """
        raise NotImplementedError("measurement_model must be overriden by a subclass of EKF")


class EkfLocalization(Ekf):
    """
    EKF Localization.
    """

    def __init__(self, x0, Sigma0, R, map_lines, tf_base_to_camera, g):
        """
        EkfLocalization constructor.

        Inputs:
                       x0: np.array[3,]  - initial belief mean.
                   Sigma0: np.array[3,3] - initial belief covariance.
                        R: np.array[2,2] - control noise covariance (corresponding to dt = 1 second).
                map_lines: np.array[2,J] - J map lines in columns representing (alpha, r).
        tf_base_to_camera: np.array[3,]  - (x, y, theta) transform from the
                                           robot base to camera frame.
                        g: float         - validation gate.
        """
        self.map_lines = map_lines  # Matrix of J map lines with (alpha, r) as columns
        self.tf_base_to_camera = tf_base_to_camera  # (x, y, theta) transform
        self.g = g  # Validation gate
        super(self.__class__, self).__init__(x0, Sigma0, R)

    def transition_model(self, u, dt):
        """
        Turtlebot dynamics (unicycle model).
        """

        ########## Code starts here ##########
        # TODO: Compute g, Gx, Gu using tb.compute_dynamics().
        g, Gx, Gu = tb.compute_dynamics(self.x, u, dt)

        ########## Code ends here ##########

        return g, Gx, Gu

    def measurement_model(self, z_raw, Q_raw):
        """
        Assemble one joint measurement and covariance from the individual values
        corresponding to each matched line feature.
        """
        v_list, Q_list, H_list = self.compute_innovations(z_raw, Q_raw)
        if not v_list:
            print("Scanner sees {} lines but can't associate them with any map entries."
                  .format(z_raw.shape[1]))
            return None, None, None

        ########## Code starts here ##########
        # TODO: Compute z, Q.
        z = np.expand_dims(v_list[0], axis=1)
        Q = np.zeros((2*len(v_list), 2*len(v_list)))
        Q[0:2,0:2] = Q_list[0]
        H = H_list[0]

        for i in range(1,len(v_list)):
            z = np.vstack((z,np.expand_dims(v_list[i], axis=1)))
            Q[i*2:i*2+2, i*2:i*2+2] = Q_list[i]
            H = np.vstack((H,H_list[i]))
        ########## Code ends here ##########

        return z, Q, H

    def compute_innovations(self, z_raw, Q_raw):
        """
        Given lines extracted from the scanner data, tries to associate each one
        to the closest map entry measured by Mahalanobis distance.

        Inputs:
            z_raw: np.array[2,I]   - I lines extracted from scanner data in
                                     columns representing (alpha, r) in the scanner frame.
            Q_raw: [np.array[2,2]] - list of I covariance matrices corresponding
                                     to each (alpha, r) column of z_raw.
        Outputs:
            v_list: [np.array[2,]]  - list of at most I innovation vectors
                                      (predicted map measurement - scanner measurement).
            Q_list: [np.array[2,2]] - list of covariance matrices of the
                                      innovation vectors (from scanner uncertainty).
            H_list: [np.array[2,3]] - list of Jacobians of the innovation
                                      vectors with respect to the belief mean self.x.
        """
        def angle_diff(a, b):
            a = a % (2. * np.pi)
            b = b % (2. * np.pi)
            diff = a - b
            if np.size(diff) == 1:
                if np.abs(a - b) > np.pi:
                    sign = 2. * (diff < 0.) - 1.
                    diff += sign * 2. * np.pi
            else:
                idx = np.abs(diff) > np.pi
                sign = 2. * (diff[idx] < 0.) - 1.
                diff[idx] += sign * 2. * np.pi
            return diff

        hs, Hs = self.compute_predicted_measurements()

        ########## Code starts here ##########
        # TODO: Compute v_list, Q_list, H_list
        v_list = []
        Q_list = []
        H_list = []

        for i in range(z_raw.shape[1]):
            z_i = z_raw[:,i]
            Q_i = Q_raw[i]
            found = False

            for j in range(hs.shape[1]):
                h_j = hs[:,j]
                H_j = Hs[j]
                v_ij = angle_diff(z_i, h_j)
                v_ij[1] = z_i[1] - h_j[1]

                S_ij = np.matmul(np.matmul(H_j,self.Sigma),H_j.T) + Q_i
                v_ij_vect = np.expand_dims(v_ij, axis=1)
                d_ij = np.matmul(np.matmul(v_ij_vect.T,np.linalg.inv(S_ij)),v_ij_vect)

                if (d_ij < (self.g)**2):
                    if (not found):
                        v_ij_found = v_ij
                        Q_i_found = Q_i
                        H_j_found = H_j
                        d_ij_found = d_ij
                        found = True
                    else:
                        if (d_ij < d_ij_found):
                            v_ij_found = v_ij
                            Q_i_found = Q_i
                            H_j_found = H_j
                            d_ij_found = d_ij

            if (found):
                v_list.append(v_ij_found)
                Q_list.append(Q_i_found)
                H_list.append(H_j_found)

        ########## Code ends here ##########

        return v_list, Q_list, H_list

    def compute_predicted_measurements(self):
        """
        Given a single map line in the world frame, outputs the line parameters
        in the scanner frame so it can be associated with the lines extracted
        from the scanner measurements.

        Input:
            None
        Outputs:
                 hs: np.array[2,J]  - J line parameters in the scanner (camera) frame.
            Hx_list: [np.array[2,3]] - list of Jacobians of h with respect to the belief mean self.x.
        """
        hs = np.zeros_like(self.map_lines)
        Hx_list = []
        for j in range(self.map_lines.shape[1]):
            ########## Code starts here ##########
            # TODO: Compute h, Hx using tb.transform_line_to_scanner_frame().
            h, Hx = tb.transform_line_to_scanner_frame(self.map_lines[:,j], self.x, self.tf_base_to_camera)

            ########## Code ends here ##########

            h, Hx = tb.normalize_line_parameters(h, Hx)
            hs[:,j] = h
            Hx_list.append(Hx)

        return hs, Hx_list


class EkfSlam(Ekf):
    """
    EKF SLAM.
    """

    def __init__(self, x0, Sigma0, R, tf_base_to_camera, g):
        """
        EKFSLAM constructor.

        Inputs:
                       x0: np.array[3+2J,]     - initial belief mean.
                   Sigma0: np.array[3+2J,3+2J] - initial belief covariance.
                        R: np.array[2,2]       - control noise covariance
                                                 (corresponding to dt = 1 second).
        tf_base_to_camera: np.array[3,]  - (x, y, theta) transform from the
                                           robot base to camera frame.
                        g: float         - validation gate.
        """
        self.tf_base_to_camera = tf_base_to_camera  # (x, y, theta) transform
        self.g = g  # Validation gate
        super(self.__class__, self).__init__(x0, Sigma0, R)

    def transition_model(self, u, dt):
        """
        Combined Turtlebot + map dynamics.
        Adapt this method from EkfLocalization.transition_model().
        """
        g = np.copy(self.x)
        Gx = np.eye(self.x.size)
        Gu = np.zeros((self.x.size, 2))

        ########## Code starts here ##########
        # TODO: Compute g, Gx, Gu.
        x = self.x[:3]

        x_t = x[0]
        y_t = x[1]
        theta_t = x[2]
        V_t = u[0]
        om_t = u[1]

        g[2] = om_t*dt + theta_t

        if (abs(om_t) >= 1e-3):
            g[0] = V_t*np.sin(g[2])/om_t - V_t*np.sin(theta_t)/om_t + x_t
            g[1] = -V_t*np.cos(g[2])/om_t + V_t*np.cos(theta_t)/om_t + y_t

            Gx[0,0] = 1.
            Gx[1,1] = 1.
            Gx[2,2] = 1.
            Gx[0,2] = V_t*np.cos(g[2])/om_t - V_t*np.cos(theta_t)/om_t
            Gx[1,2] = V_t*np.sin(g[2])/om_t - V_t*np.sin(theta_t)/om_t

            Gu[0,0] = np.sin(g[2])/om_t - np.sin(theta_t)/om_t
            Gu[0,1] = -V_t*np.sin(g[2])/(om_t**2) + V_t*np.cos(g[2])*dt/om_t + V_t*np.sin(theta_t)/(om_t**2)
            Gu[1,0] = -np.cos(g[2])/om_t + np.cos(theta_t)/om_t
            Gu[1,1] = V_t*np.cos(g[2])/(om_t**2) + V_t*np.sin(g[2])*dt/om_t - V_t*np.cos(theta_t)/(om_t**2)
            Gu[2,1] = dt
        else:
            g[0] = V_t*np.cos(g[2])*dt + x_t
            g[1] = V_t*np.sin(g[2])*dt + y_t

            Gx[0,0] = 1.
            Gx[1,1] = 1.
            Gx[2,2] = 1.
            Gx[0,2] = -V_t*np.sin(g[2])*dt
            Gx[1,2] = V_t*np.cos(g[2])*dt

            Gu[0,0] = np.cos(g[2])*dt
            Gu[0,1] = -0.5*V_t*np.sin(g[2])*dt**2
            Gu[1,0] = np.sin(g[2])*dt
            Gu[1,1] = 0.5*V_t*np.cos(g[2])*dt**2
            Gu[2,1] = dt

        ########## Code ends here ##########

        return g, Gx, Gu

    def measurement_model(self, z_raw, Q_raw):
        """
        Combined Turtlebot + map measurement model.
        Adapt this method from EkfLocalization.measurement_model().

        The ingredients for this model should look very similar to those for
        EkfLocalization. In particular, essentially the only thing that needs to
        change is the computation of Hx in self.compute_predicted_measurements()
        and how that method is called in self.compute_innovations() (i.e.,
        instead of getting world-frame line parameters from self.map_lines, you
        must extract them from the state self.x).
        """
        v_list, Q_list, H_list = self.compute_innovations(z_raw, Q_raw)
        if not v_list:
            print("Scanner sees {} lines but can't associate them with any map entries."
                  .format(z_raw.shape[1]))
            return None, None, None

        ########## Code starts here ##########
        # TODO: Compute z, Q, H.
        # Hint: Should be identical to EkfLocalization.measurement_model().
        z = np.expand_dims(v_list[0], axis=1)
        Q = np.zeros((2*len(v_list), 2*len(v_list)))
        Q[0:2,0:2] = Q_list[0]
        H = H_list[0]

        for i in range(1,len(v_list)):
            z = np.vstack((z,np.expand_dims(v_list[i], axis=1)))
            Q[i*2:i*2+2, i*2:i*2+2] = Q_list[i]
            H = np.vstack((H,H_list[i]))

        ########## Code ends here ##########

        return z, Q, H

    def compute_innovations(self, z_raw, Q_raw):
        """
        Adapt this method from EkfLocalization.compute_innovations().
        """
        def angle_diff(a, b):
            a = a % (2. * np.pi)
            b = b % (2. * np.pi)
            diff = a - b
            if np.size(diff) == 1:
                if np.abs(a - b) > np.pi:
                    sign = 2. * (diff < 0.) - 1.
                    diff += sign * 2. * np.pi
            else:
                idx = np.abs(diff) > np.pi
                sign = 2. * (diff[idx] < 0.) - 1.
                diff[idx] += sign * 2. * np.pi
            return diff

        hs, Hs = self.compute_predicted_measurements()

        ########## Code starts here ##########
        # TODO: Compute v_list, Q_list, H_list.
        v_list = []
        Q_list = []
        H_list = []

        for i in range(z_raw.shape[1]):
            z_i = z_raw[:,i]
            Q_i = Q_raw[i]
            found = False

            for j in range(hs.shape[1]):
                h_j = hs[:,j]
                H_j = Hs[j]
                v_ij = angle_diff(z_i, h_j)
                v_ij[1] = z_i[1] - h_j[1]

                S_ij = np.matmul(np.matmul(H_j,self.Sigma),H_j.T) + Q_i
                v_ij_vect = np.expand_dims(v_ij, axis=1)
                d_ij = np.matmul(np.matmul(v_ij_vect.T,np.linalg.inv(S_ij)),v_ij_vect)

                if (d_ij < (self.g)**2):
                    if (not found):
                        v_ij_found = v_ij
                        Q_i_found = Q_i
                        H_j_found = H_j
                        d_ij_found = d_ij
                        found = True
                    else:
                        if (d_ij < d_ij_found):
                            v_ij_found = v_ij
                            Q_i_found = Q_i
                            H_j_found = H_j
                            d_ij_found = d_ij

            if (found):
                v_list.append(v_ij_found)
                Q_list.append(Q_i_found)
                H_list.append(H_j_found)
        ########## Code ends here ##########

        return v_list, Q_list, H_list

    def compute_predicted_measurements(self):
        """
        Adapt this method from EkfLocalization.compute_predicted_measurements().
        """
        J = (self.x.size - 3) // 2
        hs = np.zeros((2, J))
        Hx_list = []
        for j in range(J):
            idx_j = 3 + 2 * j
            alpha, r = self.x[idx_j:idx_j+2]

            Hx = np.zeros((2,self.x.size))

            ########## Code starts here ##########
            # TODO: Compute h, Hx.
            x = self.x[:3]
            tf_base_to_camera = self.tf_base_to_camera

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
            dist = r - x_cam_w*np.cos(alpha) - y_cam_w*np.sin(alpha)

            th = x[2]
            th_cam = tf_base_to_camera[2]

            h = np.zeros(2)
            h[0] = alpha - th - th_cam
            h[1] = dist

            Hx[0,2] = -1
            Hx[1,0] = -np.cos(alpha)
            Hx[1,1] = -np.sin(alpha)
            Hx[1,2] = np.cos(alpha)*tf_base_to_camera[0]*np.sin(th) + np.cos(alpha)*tf_base_to_camera[1]*np.cos(th) - np.sin(alpha)*tf_base_to_camera[0]*np.cos(th) + np.sin(alpha)*tf_base_to_camera[1]*np.sin(th)

            # First two map lines are assumed fixed so we don't want to propagate
            # any measurement correction to them.
            if j >= 2:
                #Hx[:,idx_j:idx_j+2] = np.eye(2)  # FIX ME!
                Hx[0,idx_j] = 1
                Hx[1,idx_j] = -(-x[0]*np.sin(alpha) - np.sin(alpha)*np.cos(th)*tf_base_to_camera[0] + np.sin(alpha)*np.sin(th)*tf_base_to_camera[1] + x[1]*np.cos(alpha) + np.cos(alpha)*np.sin(th)*tf_base_to_camera[0] + np.cos(alpha)*np.cos(th)*tf_base_to_camera[1])
                Hx[1,idx_j+1] = 1
            ########## Code ends here ##########

            h, Hx = tb.normalize_line_parameters(h, Hx)
            hs[:,j] = h
            Hx_list.append(Hx)

        return hs, Hx_list
