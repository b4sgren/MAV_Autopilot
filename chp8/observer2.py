import sys 
sys.path.append('..')
import numpy as np 
import parameters.control_parameters as CTRL 
import parameters.sim_params as SIM 
import parameters.sensor_parameters as SENSOR 
import parameters.aerosonde_parameters as MAV 
from tools.tools import Euler2Rotation, Euler2Quaternion
from messages.state_msg import StateMsg

class observer:
    def __init__(self, ts_control):
        self.estimated_state = StateMsg()

        self.lpf_static = alpha_filter(alpha=0.9)
        self.lpf_diff = alpha_filter(alpha=0.5)
        self.lpf_gyrox = alpha_filter(alpha=0.5)
        self.lpf_gyroy = alpha_filter(alpha=0.5)
        self.lpf_gyroz = alpha_filter(alpha=0.5)
        # self.lpf_accelx = alpha_filter(alpha=0.5) #Do I need these for accel?
        # self.lpf_accely = alpha_filter(alpha=0.5)
        # self.lpf_accelz = alpha_filter(alpha=0.5)

        self.ekf = EKF()
    
    def update(self, measurements):
        # static_p = self.lpf_static.update(measurements.static_pressure)
        # diff_p = self.lpf_diff.update(measurements.diff_pressure)
        # measurements.static_pressure = self.lpf_static.update(measurements.static_pressure)
        # measurements.diff_pressure = self.lpf_diff.update(measurements.diff_pressure)


        self.estimated_state.p = self.lpf_gyrox.update(measurements.gyro_x - self.estimated_state.bx)
        self.estimated_state.q = self.lpf_gyroy.update(measurements.gyro_y - self.estimated_state.by)
        self.estimated_state.r = self.lpf_gyroz.update(measurements.gyro_z - self.estimated_state.bz)
        # measurements.accel_x = self.lpf_accelx.update(measurements.accel_x)
        # measurements.accel_y = self.lpf_accely.update(measurements.accel_y)
        # measurements.accel_z = self.lpf_accelz.update(measurements.accel_z)

        self.ekf.update(self.estimated_state, measurements)

        return self.estimated_state


class alpha_filter:
    def __init__(self, alpha=0.5, y0=0.0):
        self.alpha = alpha 
        self.y = y0
    
    def update(self, u):
        self.y = self.alpha * self.y + (1.0 - self.alpha) * u
        return self.y

class EKF:
    def __init__(self):
        self.xhat = np.zeros(9) # Currently assuming RPY and not quaternion. Also angular velocities are inputs not states
        self.accel = np.zeros(3)
        self.gyro = np.zeros(3)
        self.P = np.eye(9)
        self.Q = np.diag(np.ones(9)) * 0.1
        self.N = 10
        self.Ts = SIM.ts_control/self.N
    
    def update(self, state, measurement):
        self.accel = np.array([measurement.accel_x, measurement.accel_y, measurement.accel_z])
        self.gyro = np.array([measurement.gyro_x, measurement.gyro_y, measurement.gyro_z])
        self.propagate_model(state, measurement) #I may need to pass accelerometer into here
        self.measurement_update(state, measurement)
    
    def f(self, x, state, measurements):
        R = Euler2Rotation(state.phi, state.theta, state.psi)
        Va = state.Va 
        gamma = state.gamma 
        psi = state.psi
        phi = state.phi
        theta = state.theta
        
        x = np.array([state.pn, state.pe, -state.h])
        v = np.array([Va * np.cos(psi) * np.cos(gamma), Va * np.sin(psi) * np.cos(gamma), -Va * np.sin(gamma)])
        att = np.array([phi, theta, psi])

        # Would subtract the bias but I'm not using them so its always 0 right now
        w = np.array([measurements.gyro_x, measurements.gyro_y, measurements.gyro_z])
        a = np.array([measurements.accel_x, measurements.accel_y, measurements.accel_z])
        g = np.array([0.0, 0.0, 9.81])
        S = np.array([[1.0, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
                     [0.0, np.cos(phi), -np.sin(phi)],
                     [0.0, np.sin(phi) / np.cos(theta), np.cos(phi) / np.cos(theta)]])

        x_dot = R @ x 
        v_dot = np.cross(v, w) + a + R @ g
        att_dot = S @ w

        return np.block([x_dot, v_dot, att_dot])

    
    def h(self, x, state):
        debug = 1

    def propagate_model(self, state, measurements):
        for i in range(self.N):
            self.xhat += self.f(self.xhat, state, measurements) * self.Ts

            A = jacobian(self.f, self.xhat, state, measurements)
            Ad = np.eye(9) + A * self.Ts + A @ A * self.Ts**2 / 2.0
            Q_gyro = np.diag(np.ones(3)) * SENSOR.gyro_sigma
            Q_accel = np.diag(np.ones(3)) * SENSOR.accel_sigma
            Gw = np.zeros((9, 3))
            #Finish defiing Gw!!
            Ga = np.zeros((9,3))
            Ga[3:6, 3:6] = np.eye(3)

            self.P = Ad @ self.P @ Ad.T + Gw @ Q_gyro @ Gw.T + Ga @ Q_accel @ Ga.T + self.Q * self.Ts**2
    
    def measurement_update(self, state, measurements):
        debug = 1

def jacobian(fun, xhat, state, measurements):
    debug = 1