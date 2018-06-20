import numpy as np
from numpy import sin, cos, sqrt
import scipy.linalg    # you may find scipy.linalg.block_diag useful
from ExtractLines import ExtractLines, normalize_line_parameters, angle_difference
from maze_sim_parameters import LineExtractionParams, NoiseParams, MapParams
import math

class EKF(object):

    def __init__(self, x0, P0, Q):
        self.x = x0    # Gaussian belief mean
        self.P = P0    # Gaussian belief covariance
        self.Q = Q     # Gaussian control noise covariance (corresponding to dt = 1 second)

    # Updates belief state given a discrete control step (Gaussianity preserved by linearizing dynamics)
    # INPUT:  (u, dt)
    #       u - zero-order hold control input
    #      dt - length of discrete time step
    # OUTPUT: none (internal belief state (self.x, self.P) should be updated)
    def transition_update(self, u, dt):
        g, Gx, Gu = self.transition_model(u, dt)

        self.x = g
        self.P = (Gx.dot(self.P).dot(np.transpose(Gx)) +
                  dt*(Gu.dot(self.Q).dot(np.transpose(Gu))));
        
    # Propagates exact (nonlinear) state dynamics; also returns associated Jacobians for EKF linearization
    # INPUT:  (u, dt)
    #       u - zero-order hold control input
    #      dt - length of discrete time step
    # OUTPUT: (g, Gx, Gu)
    #      g  - result of belief mean self.x propagated according to the system dynamics with control u for dt seconds
    #      Gx - Jacobian of g with respect to the belief mean self.x
    #      Gu - Jacobian of g with respect to the control u
    def transition_model(self, u, dt):
        raise NotImplementedError("transition_model must be overriden by a subclass of EKF")

    # Updates belief state according to a given measurement (with associated uncertainty)
    # INPUT:  (rawZ, rawR)
    #    rawZ - raw measurement mean
    #    rawR - raw measurement uncertainty
    # OUTPUT: none (internal belief state (self.x, self.P) should be updated)
    def measurement_update(self, rawZ, rawR):
        z, R, H = self.measurement_model(rawZ, rawR)
        if z is None:    # don't update if measurement is invalid (e.g., no line matches for line-based EKF localization)
            return

        S = H.dot(self.P).dot(np.transpose(H)) + R
        K = (self.P).dot(np.transpose(H)).dot(np.linalg.inv(S))
        self.x = self.x + np.ndarray.flatten(K.dot(z))
        self.P = self.P - K.dot(S).dot(np.transpose(K))



    # Converts raw measurement into the relevant Gaussian form (e.g., a dimensionality reduction);
    # also returns associated Jacobian for EKF linearization
    # INPUT:  (rawZ, rawR)
    #    rawZ - raw measurement mean
    #    rawR - raw measurement uncertainty
    # OUTPUT: (z, R, H)
    #       z - measurement mean (for simple measurement models this may = rawZ)
    #       R - measurement covariance (for simple measurement models this may = rawR)
    #       H - Jacobian of z with respect to the belief mean self.x
    def measurement_model(self, rawZ, rawR):
        raise NotImplementedError("measurement_model must be overriden by a subclass of EKF")


class Localization_EKF(EKF):

    def __init__(self, x0, P0, Q, map_lines, tf_base_to_camera, g):
        self.map_lines = map_lines                    # 2xJ matrix containing (alpha, r) for each of J map lines
        self.tf_base_to_camera = tf_base_to_camera    # (x, y, theta) transform from the robot base to the camera frame
        self.g = g                                    # validation gate
        super(self.__class__, self).__init__(x0, P0, Q)

    # Unicycle dynamics (Turtlebot 2)
    def transition_model(self, u, dt):
        v, om = u
        x, y, th = self.x
        
        dx = v*cos(th + om*dt/2.0)*dt;
        dy = v*sin(th + om*dt/2.0)*dt;
        dth = om*dt;
        g = np.array([x+dx, y+dy, th+dth]);

        # Gx = [dg/dx, dg/dy, dg/dth]
        Gx = np.array([[1, 0, -v*sin(th + om*dt/2.0)*dt],
                       [0, 1,  v*cos(th + om*dt/2.0)*dt],
                       [0, 0, 1]]);

        # Gu = [dg/dv, dg/dom]
        Gu = np.array([[cos(th + om*dt/2.0)*dt, -v*(dt/2.0)*sin(th + om*dt/2.0)*dt],
                       [sin(th + om*dt/2.0)*dt,  v*(dt/2.0)*cos(th + om*dt/2.0)*dt],
                       [0, dt]]);

        return g, Gx, Gu

    # Given a single map line m in the world frame, outputs the line parameters in the scanner frame so it can
    # be associated with the lines extracted from the scanner measurements
    # INPUT:  m = (alpha, r)
    #       m - line parameters in the world frame
    # OUTPUT: (h, Hx)
    #       h - line parameters in the scanner (camera) frame
    #      Hx - Jacobian of h with respect to the the belief mean self.x
    def map_line_to_predicted_measurement(self, m):
        alpha, r = m
        
        x, y, th = self.x
        xb2c, yb2c, thb2c = self.tf_base_to_camera

        # position of camera coordinate system in world frame
        xcam = x + xb2c*cos(th) - yb2c*sin(th)
        ycam = y + xb2c*sin(th) + yb2c*cos(th)

        rcam = sqrt(xcam**2 + ycam**2)
        acam = math.atan2(ycam, xcam)

        # distance from camera to line m
        rprime = r - rcam*cos(acam-alpha)

        # angle from camera frame to line m
        aprime = alpha - th - thb2c

        h = np.array([aprime, rprime]);

        dh2dx  = -cos(alpha)
        dh2dy  = -sin(alpha)
        dh2dth = (cos(alpha)*(xb2c*sin(th)+yb2c*cos(th)) -
                  sin(alpha)*(xb2c*cos(th)-yb2c*sin(th)))

        Hx = np.array([[0, 0, -1],
                       [dh2dx, dh2dy, dh2dth]]);

        flipped, h = normalize_line_parameters(h)
        if flipped:
            Hx[1,:] = -Hx[1,:]

        return h, Hx

    # Given lines extracted from the scanner data, tries to associate to each one the closest map entry
    # measured by Mahalanobis distance
    # INPUT:  (rawZ, rawR)
    #    rawZ - 2xI matrix containing (alpha, r) for each of I lines extracted from the scanner data (in scanner frame)
    #    rawR - list of I 2x2 covariance matrices corresponding to each (alpha, r) column of rawZ
    # OUTPUT: (v_list, R_list, H_list)
    #  v_list - list of at most I innovation vectors (predicted map measurement - scanner measurement)
    #  R_list - list of len(v_list) covariance matrices of the innovation vectors (from scanner uncertainty)
    #  H_list - list of len(v_list) Jacobians of the innovation vectors with respect to the belief mean self.x
    def associate_measurements(self, rawZ, rawR):

        I = np.shape(rawZ)[1]
        J = np.shape(self.map_lines)[1]
        
        v_list = []
        R_list = []
        H_list = []

        for i in range(I):       # for each scanner line
            dmin = self.g**2
            for j in range(J):   # consider every map line
                zi = rawZ[:,i]
                hj, Hj = self.map_line_to_predicted_measurement(self.map_lines[:,j])
                Ri = rawR[i]
                
                vij = zi-hj
                Sij = Hj.dot(self.P).dot(np.transpose(Hj)) + Ri
                dij = (np.transpose(vij)).dot(np.linalg.inv(Sij)).dot(vij)

                if dij < dmin:
                    v_best = vij
                    R_best = Ri
                    H_best = Hj
                    dmin = dij

            if dmin < self.g**2:  # if we found a match for scanner line i
                v_list.append(v_best)
                R_list.append(R_best)
                H_list.append(H_best)

        return v_list, R_list, H_list

    # Assemble one joint measurement, covariance, and Jacobian from the individual values corresponding to each
    # matched line feature
    def measurement_model(self, rawZ, rawR):
        v_list, R_list, H_list = self.associate_measurements(rawZ, rawR)
        if not v_list:
            print "Scanner sees", rawZ.shape[1], "line(s) but can't associate them with any map entries"
            return None, None, None

        z = np.ndarray.flatten(np.array(v_list))
        R = scipy.linalg.block_diag(*R_list)
        H = np.reshape(np.array(H_list), (2*len(v_list), 3))

        return z, R, H


class SLAM_EKF(EKF):

    def __init__(self, x0, P0, Q, tf_base_to_camera, g):
        self.tf_base_to_camera = tf_base_to_camera    # (x, y, theta) transform from the robot base to the camera frame
        self.g = g                                    # validation gate
        super(self.__class__, self).__init__(x0, P0, Q)

    # Combined Turtlebot + map dynamics
    # Adapt this method from Localization_EKF.transition_model.
    def transition_model(self, u, dt):
        v, om = u
        x, y, th = self.x[:3]

        dx = v*cos(th + om*dt/2.0)*dt;
        dy = v*sin(th + om*dt/2.0)*dt;
        dth = om*dt;
        # dalpha, dr = 0
        
        g = np.copy(self.x)
        g[:3] = np.array([x+dx, y+dy, th+dth]);

        # Gx = [dg/dx, dg/dy, dg/dth, dg/da1, dg/dr1, dg/da2, dg/dr2....]
        Gx = np.eye(self.x.size)
        Gx[0:3, 0:3] = np.array([[1, 0, -v*sin(th + om*dt/2.0)*dt],
                               [0, 1, v*cos(th + om*dt/2.0)*dt],
                               [0, 0, 1]]);

        # Gu = [dg/dv, dg/dom]
        Gu = np.zeros((self.x.size, 2))
        Gu[0:3, 0:2] = np.array([[cos(th + om*dt/2.0)*dt, -v*(dt/2.0)*sin(th + om*dt/2.0)*dt],
                                 [sin(th + om*dt/2.0)*dt,  v*(dt/2.0)*cos(th + om*dt/2.0)*dt],
                                 [0, dt]]);

        return g, Gx, Gu

    # Combined Turtlebot + map measurement model
    # Adapt this method from Localization_EKF.measurement_model.
    #
    # The ingredients for this model should look very similar to those for Localization_EKF.
    # In particular, essentially the only thing that needs to change is the computation
    # of Hx in map_line_to_predicted_measurement and how that method is called in
    # associate_measurements (i.e., instead of getting world-frame line parameters from
    # self.map_lines, you must extract them from the state self.x)
    def measurement_model(self, rawZ, rawR):
        v_list, R_list, H_list = self.associate_measurements(rawZ, rawR)
        if not v_list:
            print "Scanner sees", rawZ.shape[1], "line(s) but can't associate them with any map entries"
            return None, None, None

        z = np.ndarray.flatten(np.array(v_list))
        R = scipy.linalg.block_diag(*R_list)
        H = np.reshape(np.array(H_list), (2*len(v_list), len(self.x)))

        return z, R, H

    # Adapt this method from Localization_EKF.map_line_to_predicted_measurement.
    #
    # Note that instead of the actual parameters m = (alpha, r) we pass in the map line index j
    # so that we know which components of the Jacobian to fill in.
    def map_line_to_predicted_measurement(self, j):
        alpha, r = self.x[(3+2*j):(3+2*j+2)]    # j is zero-indexed! (yeah yeah I know this doesn't match the pset writeup)

        x, y, th = self.x[:3]
        xb2c, yb2c, thb2c = self.tf_base_to_camera

        # position of camera coordinate system in world frame (taken from Q1)
        xcam = x + xb2c*cos(th) - yb2c*sin(th)
        ycam = y + xb2c*sin(th) + yb2c*cos(th)
        rcam = sqrt(xcam**2 + ycam**2)
        acam = math.atan2(ycam, xcam)

        aprime = alpha - th - thb2c 
        rprime = r - rcam*cos(acam-alpha)
        h = np.array([aprime, rprime]);

        # also taken from Q1
        dh2dx  = -cos(alpha)
        dh2dy  = -sin(alpha)
        dh2dth = (cos(alpha)*(xb2c*sin(th)+yb2c*cos(th)) -
                  sin(alpha)*(xb2c*cos(th)-yb2c*sin(th)))
        
        Hx = np.zeros((2,self.x.size))
        Hx[:,:3] = np.array([[0, 0, -1],
                       [dh2dx, dh2dy, dh2dth]]);

        # First two map lines are assumed fixed so we don't want to propagate any measurement correction to them
        if j > 1:
            Hx[0, 3+2*j] = 1
            Hx[1, 3+2*j] = -rcam*sin(acam-alpha)
            Hx[0, 3+2*j+1] = 0
            Hx[1, 3+2*j+1] = 1

        flipped, h = normalize_line_parameters(h)
        if flipped:
            Hx[1,:] = -Hx[1,:]

        return h, Hx

    # Adapt this method from Localization_EKF.associate_measurements.
    def associate_measurements(self, rawZ, rawR):

        I = np.shape(rawZ)[1]
        J = (len(self.x) - 3)/2;
        
        v_list = []
        R_list = []
        H_list = []

        for i in range(I):       # for each scanner line
            dmin = self.g**2
            for j in range(J):   # consider every map line
                zi = rawZ[:,i]
                hj, Hj = self.map_line_to_predicted_measurement(j)
                Ri = rawR[i]
                
                vij = zi-hj
                Sij = Hj.dot(self.P).dot(np.transpose(Hj)) + Ri
                dij = (np.transpose(vij)).dot(np.linalg.inv(Sij)).dot(vij)

                if dij < dmin:
                    v_best = vij
                    R_best = Ri
                    H_best = Hj
                    dmin = dij

            if dmin < self.g**2:  # if we found a match for scanner line i
                v_list.append(v_best)
                R_list.append(R_best)
                H_list.append(H_best)

        return v_list, R_list, H_list

