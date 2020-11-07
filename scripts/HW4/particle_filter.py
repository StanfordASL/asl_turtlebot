# Karen May Wang (kmwang14@stanford.edu)
# AA274A
# 11/06/2020
import numpy as np
import scipy.linalg  # You may find scipy.linalg.block_diag useful
import scipy.stats  # You may find scipy.stats.multivariate_normal.pdf useful
import turtlebot_model as tb
import pdb
EPSILON_OMEGA = 1e-3

class ParticleFilter(object):
    """
    Base class for Monte Carlo localization and FastSLAM.

    Usage:
        pf = ParticleFilter(x0, R)
        while True:
            pf.transition_update(u, dt)
            pf.measurement_update(z, Q)
            localized_state = pf.x
    """

    def __init__(self, x0, R):
        """
        ParticleFilter constructor.

        Inputs:
            x0: np.array[M,3] - initial particle states.
             R: np.array[2,2] - control noise covariance (corresponding to dt = 1 second).
        """
        self.M = x0.shape[0]  # Number of particles
        self.xs = x0  # Particle set [M x 3]
        self.ws = np.repeat(1. / self.M, self.M)  # Particle weights (initialize to uniform) [M]
        self.R = R  # Control noise covariance (corresponding to dt = 1 second) [2 x 2]

    @property
    def x(self):
        """
        Returns the particle with the maximum weight for visualization.

        Output:
            x: np.array[3,] - particle with the maximum weight.
        """
        idx = self.ws == self.ws.max()
        x = np.zeros(self.xs.shape[1:])
        x[:2] = self.xs[idx,:2].mean(axis=0)
        th = self.xs[idx,2]
        x[2] = np.arctan2(np.sin(th).mean(), np.cos(th).mean())
        return x

    def transition_update(self, u, dt):
        """
        Performs the transition update step by updating self.xs.

        Inputs:
            u: np.array[2,] - zero-order hold control input.
            dt: float        - duration of discrete time step.
        Output:
            None - internal belief state (self.xs) should be updated.
        """
        ########## Code starts here ##########
        # TODO: Update self.xs.
        # Hint: Call self.transition_model().
        # Hint: You may find np.random.multivariate_normal useful.
        u_with_noise = np.zeros((self.M,2))
        for m in range(self.M):
            u_with_noise[m,:] = np.random.multivariate_normal(u,self.R)
        g = self.transition_model(u_with_noise, dt)
        self.xs = g
        ########## Code ends here ##########

    def transition_model(self, us, dt):
        """
        Propagates exact (nonlinear) state dynamics.

        Inputs:
            us: np.array[M,2] - zero-order hold control input for each particle.
            dt: float         - duration of discrete time step.
        Output:
            g: np.array[M,3] - result of belief mean for each particle
                               propagated according to the system dynamics with
                               control u for dt seconds.
        """
        raise NotImplementedError("transition_model must be overridden by a subclass of EKF")

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
            None - internal belief state (self.x, self.ws) is updated in self.resample().
        """
        raise NotImplementedError("measurement_update must be overridden by a subclass of EKF")

    def resample(self, xs, ws):
        """
        Resamples the particles according to the updated particle weights.

        Inputs:
            xs: np.array[M,3] - matrix of particle states.
            ws: np.array[M,]  - particle weights.

        Output:
            None - internal belief state (self.xs, self.ws) should be updated.
        """
        r = np.random.rand() / self.M

        ########## Code starts here ##########
        # TODO: Update self.xs, self.ws.
        # Note: Assign the weights in self.ws to the corresponding weights in ws
        #       when resampling xs instead of resetting them to a uniform
        #       distribution. This allows us to keep track of the most likely
        #       particle and use it to visualize the robot's pose with self.x.
        # Hint: To maximize speed, try to implement the resampling algorithm
        #       without for loops. You may find np.linspace(), np.cumsum(), and
        #       np.searchsorted() useful. This results in a ~10x speedup.
        indices = np.zeros(self.M,dtype='int') #Mx1 array
        c = np.ones(self.M)*ws[0] # Mx1 array
        m = np.arange(self.M)
        u = np.sum(ws)*(r + m/float(self.M))
        while len(np.where(c<u)[0]) > 0:
            ind_condition = np.where(c<u)[0]
            indices[ind_condition] += 1
            c[ind_condition] += ws[indices[ind_condition]]
        self.xs = xs[indices,:] 
        self.ws = ws[indices] 
        #i = 0
        #c = ws[0]
        #for m in range(self.M):
        #    u = np.sum(ws)*(r+m/float(self.M))
        #    while c < u:
        #        i += 1
        #        c += ws[i]
        #    self.xs[m,:] = xs[i,:]
        #    self.ws[m] = ws[i] 
        ########## Code ends here ##########

    def measurement_model(self, z_raw, Q_raw):
        """
        Converts raw measurements into the relevant Gaussian form (e.g., a
        dimensionality reduction).

        Inputs:
            z_raw: np.array[2,I]   - I lines extracted from scanner data in
                                     columns representing (alpha, r) in the scanner frame.
            Q_raw: [np.array[2,2]] - list of I covariance matrices corresponding
                                     to each (alpha, r) column of z_raw.
        Outputs:
            z: np.array[2I,]   - joint measurement mean.
            Q: np.array[2I,2I] - joint measurement covariance.
        """
        raise NotImplementedError("measurement_model must be overridden by a subclass of EKF")


class MonteCarloLocalization(ParticleFilter):

    def __init__(self, x0, R, map_lines, tf_base_to_camera, g):
        """
        MonteCarloLocalization constructor.

        Inputs:
                       x0: np.array[M,3] - initial particle states.
                        R: np.array[2,2] - control noise covariance (corresponding to dt = 1 second).
                map_lines: np.array[2,J] - J map lines in columns representing (alpha, r).
        tf_base_to_camera: np.array[3,]  - (x, y, theta) transform from the
                                           robot base to camera frame.
                        g: float         - validation gate.
        """
        self.map_lines = map_lines  # Matrix of J map lines with (alpha, r) as columns
        self.tf_base_to_camera = tf_base_to_camera  # (x, y, theta) transform
        self.g = g  # Validation gate
        super(self.__class__, self).__init__(x0, R)

    def transition_model(self, us, dt):
        """
        Unicycle model dynamics.

        Inputs:
            us: np.array[M,2] - zero-order hold control input for each particle.
            dt: float         - duration of discrete time step.
        Output:
            g: np.array[M,3] - result of belief mean for each particle
                               propagated according to the system dynamics with
                               control u for dt seconds.
        """

        ########## Code starts here ##########
        # TODO: Compute g.
        # Hint: We don't need Jacobians for particle filtering.
        # Hint: A simple solution can be using a for loop for each partical
        #       and a call to tb.compute_dynamics
        # Hint: To maximize speed, try to compute the dynamics without looping
        #       over the particles. If you do this, you should implement
        #       vectorized versions of the dynamics computations directly here
        #       (instead of modifying turtlebot_model). This results in a
        #       ~10x speedup.
        # Hint: This faster/better solution does not use loop and does 
        #       not call tb.compute_dynamics. You need to compute the idxs
        #       where abs(om) > EPSILON_OMEGA and the other idxs, then do separate 
        #       updates for them
         
        g = np.zeros((self.M,3))
        x = self.xs[:,0]
        y = self.xs[:,1]
        th = self.xs[:,2]
        #V = us[:,0]
        #om = us[:,1]
        if len(us) < self.M:
           V = np.expand_dims(us,axis=0)[:,0]
           om = np.expand_dims(us,axis=0)[:,1]
        else:
           V = us[:,0]
           om = us[:,1]

        #update theta for all particles first
        g[:,2] = th + om*dt
        #handle case when abs(om) < EPSILON_OMEGA
        #also for this case, use the 2nd-order trapezoidal method for the integration
        small_om_idxs = np.where(abs(om) < EPSILON_OMEGA)
        g[small_om_idxs,0] = x[small_om_idxs] + (V[small_om_idxs]/2.0)*(np.cos(th[small_om_idxs]) + np.cos(g[small_om_idxs,2]))*dt
        g[small_om_idxs,1] = y[small_om_idxs] + (V[small_om_idxs]/2.0)*(np.sin(th[small_om_idxs]) + np.sin(g[small_om_idxs,2]))*dt

        #else handle the normal case
        om_idxs = np.where(abs(om) >= EPSILON_OMEGA)
        g[om_idxs,0] = x[om_idxs] + (V[om_idxs]/om[om_idxs])*(np.sin(g[om_idxs,2]) - np.sin(th[om_idxs]))
        g[om_idxs,1] = y[om_idxs] - (V[om_idxs]/om[om_idxs])*(np.cos(g[om_idxs,2]) - np.cos(th[om_idxs]))

        ########## Code ends here ##########
        return g

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
            None - internal belief state (self.x, self.ws) is updated in self.resample().
        """
        xs = np.copy(self.xs)
        ws = np.zeros_like(self.ws)

        ########## Code starts here ##########
        # TODO: Compute new particles (xs, ws) with updated measurement weights.
        # Hint: To maximize speed, implement this without looping over the
        #       particles. You may find scipy.stats.multivariate_normal.pdf()
        #       useful.
        # Hint: You'll need to call self.measurement_model()
        vs, Q = self.measurement_model(z_raw, Q_raw)
        #vs is M x 2I and Q is 2I x 2I
        I = len(Q)/2
        ws = scipy.stats.multivariate_normal(mean=np.zeros(2*I),cov=Q).pdf(vs)
        ########## Code ends here ##########

        self.resample(xs, ws)

    def measurement_model(self, z_raw, Q_raw):
        """
        Assemble one joint measurement and covariance from the individual values
        corresponding to each matched line feature for each particle.

        Inputs:
            z_raw: np.array[2,I]   - I lines extracted from scanner data in
                                     columns representing (alpha, r) in the scanner frame.
            Q_raw: [np.array[2,2]] - list of I covariance matrices corresponding
                                     to each (alpha, r) column of z_raw.
        Outputs:
            vs: np.array[M,2I]  - joint measurement mean for M particles.
            Q: np.array[2I,2I] - joint measurement covariance.
        """
        vs = self.compute_innovations(z_raw, np.array(Q_raw))

        ########## Code starts here ##########
        # TODO: Compute Q.
        # Hint: You might find scipy.linalg.block_diag() useful
        I = len(Q_raw)
        Q = Q_raw[0] 
        for i in range(1,I):
            Q = scipy.linalg.block_diag(Q,Q_raw[i])

        ########## Code ends here ##########

        return vs, Q

    def compute_innovations(self, z_raw, Q_raw):
        """
        Given lines extracted from the scanner data, tries to associate each one
        to the closest map entry measured by Mahalanobis distance.

        Inputs:
            z_raw: np.array[2,I]   - I lines extracted from scanner data in
                                     columns representing (alpha, r) in the scanner frame.
            Q_raw: np.array[I,2,2] - I covariance matrices corresponding
                                     to each (alpha, r) column of z_raw.
        Outputs:
            vs: np.array[M,2I] - M innovation vectors of size 2I
                                 (predicted map measurement - scanner measurement).
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

        ########## Code starts here ##########
        # TODO: Compute vs (with shape [M x I x 2]).
        # Hint: Simple solutions: Using for loop, for each particle, for each 
        #       observed line, find the most likely map entry (the entry with 
        #       least Mahalanobis distance).
        # Hint: To maximize speed, try to eliminate all for loops, or at least
        #       for loops over J. It is possible to solve multiple systems with
        #       np.linalg.solve() and swap arbitrary axes with np.transpose().
        #       Eliminating loops over J results in a ~10x speedup.
        #       Eliminating loops over I results in a ~2x speedup.
        #       Eliminating loops over M results in a ~5x speedup.
        #       Overall, that's 100x!
        # Hint: For the faster solution, you might find np.expand_dims(), 
        #       np.linalg.solve(), np.meshgrid() useful.
        hs = self.compute_predicted_measurements()
        I = len(Q_raw)
        _,_,J = hs.shape
        vs = np.zeros((self.M,I,2))
 
        dmin = None
        #dmin = np.ones((self.M))*1e25
        for i in range(I):
            #go through all known map lines in camera frames and compute Mahalanobis distance
            vij = np.repeat(z_raw[:,i:i+1],J,axis=1) - hs #vij is Mx2xJ. z_raw is changed to 2xJ, hs is Mx2xJ
            vij[:,0,:] = angle_diff(z_raw[0,i:i+1],hs[:,0,:]) #replace computation of angle with angle_diff function
            vij_reshaped_int = vij.reshape(self.M,2*J,1,order='F') #M x 2*J
            vij_reshaped = vij_reshaped_int.reshape(self.M*2*J) #2xJxM x 1
            invQ_big = np.kron(np.eye(J*self.M),np.linalg.inv(Q_raw[i,:,:]))  #2xJxM x 2xJxM 
            dij_inter = np.dot(invQ_big,vij_reshaped)
            dij_inter2 = vij_reshaped*dij_inter
            dij = dij_inter2[0:-1:2] + dij_inter2[1::2]
            dij = dij.reshape(self.M,J)
            min_ind_i = np.arange(self.M)
            min_ind_j = np.argmin(dij,axis=1)
            dmin = dij[min_ind_i,min_ind_j]
            vij_min = vij[min_ind_i,:,min_ind_j]
            vs[min_ind_i,i,:] = vij_min

        ########## Code ends here ##########

        # Reshape [M x I x 2] array to [M x 2I]
        return vs.reshape((self.M,-1))  # [M x 2I]

    def compute_predicted_measurements(self):
        """
        Given a single map line in the world frame, outputs the line parameters
        in the scanner frame so it can be associated with the lines extracted
        from the scanner measurements.

        Input:
            None
        Output:
            hs: np.array[M,2,J] - J line parameters in the scanner (camera) frame for M particles.
        """
        ########## Code starts here ##########
        # TODO: Compute hs.
        # Hint: We don't need Jacobians for particle filtering.
        # Hint: Simple solutions: Using for loop, for each particle, for each 
        #       map line, transform to scanner frmae using tb.transform_line_to_scanner_frame()
        #       and tb.normalize_line_parameters()
        # Hint: To maximize speed, try to compute the predicted measurements
        #       without looping over the map lines. You can implement vectorized
        #       versions of turtlebot_model functions directly here. This
        #       results in a ~10x speedup.
        # Hint: For the faster solution, it does not call tb.transform_line_to_scanner_frame()
        #       or tb.normalize_line_parameters(), but reimplement these steps vectorized.
        _, J = self.map_lines.shape
        alpha = self.map_lines[0]  #J
        r     = self.map_lines[1]  #J
        alpha_repeat = np.repeat(alpha[:,np.newaxis],self.M,axis=1).T #M x J
        r_repeat     = np.repeat(r[:,np.newaxis],self.M,axis=1).T #M x J
        xcam = self.tf_base_to_camera[0]
        ycam = self.tf_base_to_camera[1]
        thcam = self.tf_base_to_camera[2]
        
        xw = self.xs[:,0] #M
        yw = self.xs[:,1] #M
        thw = self.xs[:,2] #M
        
        xw_rep = np.repeat(xw[:,np.newaxis],J,axis=1) #M x J
        yw_rep = np.repeat(yw[:,np.newaxis],J,axis=1) #M x J
        thw_rep = np.repeat(thw[:,np.newaxis],J,axis=1) #M x J
        hs = np.zeros((self.M,2,J))

        hs[:,0,:] = alpha_repeat - (thw_rep + thcam)#M x J

        hs[:,1,:] = r_repeat - (np.cos(alpha_repeat)*(xw_rep + xcam*np.cos(thw_rep)-ycam*np.sin(thw_rep)) + np.sin(alpha_repeat)*(yw_rep+xcam*np.sin(thw_rep)+ycam*np.cos(thw_rep)) )

        #normalize line parameters
        idx1, idx2 = np.where(hs[:,1,:] < 0)
        for i in range(len(idx1)):
            hs[idx1[i],1,idx2[i]] *= -1
            hs[idx1[i],0,idx2[i]] += np.pi
        hs[:,0,:] = (hs[:,0,:] + np.pi) % (2*np.pi) - np.pi
        ########## Code ends here ##########

        return hs

