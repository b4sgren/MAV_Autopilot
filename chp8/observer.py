"""
observer
    - Beard & McLain, PUP, 2012
    - Last Update:
        3/2/2019 - RWB
"""
import sys
import numpy as np
sys.path.append('..')
import parameters.control_parameters as CTRL
import parameters.sim_params as SIM
import parameters.sensor_parameters as SENSOR
import parameters.aerosonde_parameters as MAV
# from tools.rotations import Euler2Rotation # Need to write this

from messages.state_msg import StateMsg

class observer:
    def __init__(self, ts_control):
        # initialized estimated state message
        self.estimated_state = StateMsg()

        # use alpha filters to low pass filter gyros and accels
        self.lpf_gyro_x = alpha_filter(alpha=0.5)
        self.lpf_gyro_y = alpha_filter(alpha=0.5)
        self.lpf_gyro_z = alpha_filter(alpha=0.5)
        self.lpf_accel_x = alpha_filter(alpha=0.5)
        self.lpf_accel_y = alpha_filter(alpha=0.5)
        self.lpf_accel_z = alpha_filter(alpha=0.5)
        # use alpha filters to low pass filter static and differential pressure
        self.lpf_static = alpha_filter(alpha=0.9)
        self.lpf_diff = alpha_filter(alpha=0.5)

        # ekf for phi and theta
        self.attitude_ekf = ekf_attitude()
        # ekf for pn, pe, Vg, chi, wn, we, psi
        # self.position_ekf = ekf_position()

    def update(self, measurements):
        #probably want to do the lpf on accel  and pressure here
        static_p = self.lpf_static.update(measurements.static_pressure)
        diff_p = self.lpf_diff.update(measurements.diff_pressure)

        # estimates for p, q, r are low pass filter of gyro minus bias estimate
        # Shouldn't use the lpf version in the EKF. Do after attitude_ekf.update()
        self.estimated_state.p = self.lpf_gyro_x.update(measurements.gyro_x - self.estimated_state.bx)
        self.estimated_state.q = self.lpf_gyro_y.update(measurements.gyro_y - self.estimated_state.by)
        self.estimated_state.r = self.lpf_gyro_z.update(measurements.gyro_z - self.estimated_state.bz)

        #If I have time, try to implement the full state EKF instead of everything below

        # invert sensor model to get altitude and airspeed
        g = MAV.gravity
        rho = MAV.rho
        self.estimated_state.h = static_p / (rho * g)
        self.estimated_state.Va = np.sqrt((diff_p * g) / rho)

        # estimate phi and theta with simple ekf
        self.attitude_ekf.update(self.estimated_state, measurements)

        # estimate pn, pe, Vg, chi, wn, we, psi
        # self.position_ekf.update(self.estimated_state, measurements)

        # not estimating these
        self.estimated_state.alpha = self.estimated_state.theta
        self.estimated_state.beta = 0.0
        self.estimated_state.bx = 0.0
        self.estimated_state.by = 0.0
        self.estimated_state.bz = 0.0

        return self.estimated_state

class alpha_filter:
    # alpha filter implements a simple low pass filter
    # y[k] = alpha * y[k-1] + (1-alpha) * u[k]
    def __init__(self, alpha=0.5, y0=0.0):
        self.alpha = alpha  # filter parameter
        self.y = y0  # initial condition

    def update(self, u):
        self.y = self.alpha * self.y + (1.0 - self.alpha) * u
        return self.y

class ekf_attitude:
    # implement continous-discrete EKF to estimate roll and pitch angles
    def __init__(self):
        self.Q = np.diag([1e-9, 1e-9]) # This is a tuning parameter
        self.Q_gyro = np.eye(3) * SENSOR.gyro_sigma**2
        self.R_accel = np.eye(3) * SENSOR.accel_sigma**2
        self.N = 10  # number of prediction step per sample
        self.xhat = np.array([[0.0], [0.0]])  # initial state: phi, theta
        self.P = np.eye(2) * 0.1  # Represents uncertainty in initial conditions
        self.Ts = SIM.ts_control/self.N

    def update(self, state, measurement):
        self.propagate_model(state)
        self.measurement_update(state, measurement)
        state.phi = self.xhat.item(0)
        state.theta = self.xhat.item(1)

    def f(self, x, state):
        # system dynamics for propagation model: xdot = f(x, u)
        phi = x.item(0)
        theta = x.item(1)
        G = np.array([[1.0, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
                      [0.0, np.cos(phi), -np.sin(phi)]])
        u = np.array([[state.p, state.q, state.r]]).T

        _f = G @ u
        return _f

    def h(self, x, state):
        # measurement model y
        g = MAV.gravity
        Va = state.Va
        phi = x.item(0)
        theta = x.item(1)
        _h = np.array([[state.q * Va * np.sin(theta) + g * np.sin(theta)],
                       [state.r * Va * np.cos(theta) - state.p * Va * np.sin(theta) - g * np.cos(theta) * np.sin(phi)],
                       [-state.q * Va * np.cos(theta) - g * np.cos(theta) * np.cos(phi)]])
        return _h

    def propagate_model(self, state):
        # model propagation
        for i in range(0, self.N):
             # propagate model
            self.xhat = self.xhat + self.Ts * self.f(self.xhat, state);
            # compute Jacobian
            A = jacobian(self.f, self.xhat, state)
            # compute G matrix for gyro noise
            phi = self.xhat.item(0)
            theta = self.xhat.item(1)
            G = np.array([[1.0, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
                          [0.0, np.cos(phi), -np.sin(phi)]])

            # update P with continuous time model
            # self.P = self.P + self.Ts * (A @ self.P + self.P @ A.T + self.Q + G @ self.Q_gyro @ G.T)
            # convert to discrete time models
            A_d = np.eye(2) + A * self.Ts + (A @ A) * (self.Ts**2)/2
            G_d = self.Ts * G
            # update P with discrete time model
            # The G_d * Q_gyro * G_d.T is because we use a noisy measurement in G (see f()) and need to account for it
            self.P = A_d @ self.P @ A_d.T + G_d @ self.Q_gyro @ G_d.T + self.Q * self.Ts**2

    def measurement_update(self, state, measurement):
        # measurement updates
        threshold = 2.0
        h = self.h(self.xhat, state)
        C = jacobian(self.h, self.xhat, state)
        y = np.array([measurement.accel_x, measurement.accel_y, measurement.accel_z])
        L = self.P @ C.T @ np.linalg.inv(self.R_accel + C @ self.P @ C.T)

        # This seems to help but still doesn't work
        update = True;
        for i in range(h.shape[0]):
            if np.abs(y[i] - h[i]) > threshold:
                update = False

        if update:
            self.xhat = self.xhat + L @ (y - C @ self.xhat)
            I = np.eye(2)
            self.P = (I - L @ C) @ self.P @ (I - L @ C).T + L @ self.R_accel @ L.T

class ekf_position:
    # implement continous-discrete EKF to estimate pn, pe, chi, Vg
    def __init__(self):
        self.Q = np.diag([0.1, 0.1, 0.1, 0.1])
        self.R = np.diag([SENSORS.gps_n_sigma**2, SENSORS.gps_e_sigma**2,
                            SENSORS.gps_course_sigma**2, SENSORS.gps_Vg_sigma**2])
        self.N = 25  # number of prediction step per sample
        self.Ts = (SIM.ts_control / self.N)
        self.xhat = np.array([[0.0, 0.0, 0.0, 0.0]]).T  #Not sure all of these should start at 0
        self.P = np.eye(4) * 0.1
        self.gps_n_old = 9999
        self.gps_e_old = 9999
        self.gps_Vg_old = 9999
        self.gps_course_old = 9999


    def update(self, state, measurement):
        self.propagate_model(state)
        self.measurement_update(state, measurement)
        state.pn = self.xhat.item(0)
        state.pe = self.xhat.item(1)
        state.Vg = self.xhat.item(2)
        state.chi = self.xhat.item(3)
        state.wn = self.xhat.item(4)
        state.we = self.xhat.item(5)
        state.psi = self.xhat.item(6)

    def f(self, x, state):
        # system dynamics for propagation model: xdot = f(x, u)
        _f = 0
        return _f

    def h_gps(self, x, state):
        # measurement model for gps measurements
        _h = 0
        return _h

    def h_pseudo(self, x, state):
        # measurement model for wind triangale pseudo measurement
        _h = 0
        return _h

    def propagate_model(self, state):
        # model propagation
        for i in range(0, self.N):
            # propagate model
            self.xhat = self.xhat + self.Ts * self.f(self.xhat, state)
            # compute Jacobian
            A = jacobian(self.f, self.xhat, state)
            # update P with continuous time model
            # self.P = self.P + self.Ts * (A @ self.P + self.P @ A.T + self.Q + G @ self.Q_gyro @ G.T)
            # convert to discrete time models
            A_d = np.eye(4) + A * slef.Ts + A @ A * self.Ts**2 / 2.0
            # update P with discrete time model
            self.P = A_d @ self.P @ A_d.T + self.Q * self.Ts**2

    def measurement_update(self, state, measurement):
        # always update based on wind triangle pseudo measurement
        h = self.h_pseudo(self.xhat, state)
        C = jacobian(self.h_pseudo, self.xhat, state)
        y = np.array([0, 0]) #This is to estimate wn and we
        for i in range(0, 2):
            Ci = 0
            L = 0
            self.P = 0
            self.xhat = 0

        # only update GPS when one of the signals changes
        if (measurement.gps_n != self.gps_n_old) \
            or (measurement.gps_e != self.gps_e_old) \
            or (measurement.gps_Vg != self.gps_Vg_old) \
            or (measurement.gps_course != self.gps_course_old):

            h = self.h_gps(self.xhat, state)
            C = jacobian(self.h_gps, self.xhat, state)
            y = np.array([measurement.gps_n, measurement.gps_e, measurement.gps_Vg, measurement.gps_course])
            for i in range(0, 4):
                Ci = 0
                L = 0
                self.P = 0
                self.xhat = 0
            # update stored GPS signals
            self.gps_n_old = measurement.gps_n
            self.gps_e_old = measurement.gps_e
            self.gps_Vg_old = measurement.gps_Vg
            self.gps_course_old = measurement.gps_course

    def wrap(self, chi_c, chi):
        while chi_c-chi > np.pi:
            chi_c = chi_c - 2.0 * np.pi
        while chi_c-chi < -np.pi:
            chi_c = chi_c + 2.0 * np.pi
        return chi_c

def jacobian(fun, x, state):
    # compute jacobian of fun with respect to x
    f = fun(x, state)
    m = f.shape[0]
    n = x.shape[0]
    eps = 0.01  # deviation
    J = np.zeros((m, n))
    for i in range(0, n):
        x_eps = np.copy(x)
        x_eps[i][0] += eps
        f_eps = fun(x_eps, state)
        df = (f_eps - f) / eps
        J[:, i] = df[:, 0]
    return J
