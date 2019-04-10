"""
mav_dynamics
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state

"""
import sys
sys.path.append('..')
import numpy as np
from math import asin, exp, acos

# load message types
from messages.state_msg import StateMsg
from messages.msg_sensors import msg_sensors

import parameters.aerosonde_parameters as MAV
import parameters.sensor_parameters as SENSOR
from tools.tools import Quaternion2Euler, Quaternion2Rotation

class mav_dynamics:
    def __init__(self, Ts):
        self.ts_simulation = Ts
        # _state = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]
        self._state = np.array([MAV.pn0, MAV.pe0, MAV.pd0, MAV.u0, MAV.v0, MAV.w0,
                                MAV.e0, MAV.e1, MAV.e2, MAV.e3, MAV.p0, MAV.q0, MAV.r0])
        self._state = self._state.reshape((13, 1))

        self._wind = np.zeros((3, 1))
        self.updateVelocityData()
        #store the forces
        self._forces = np.zeros((3, 1))
        self._Va = MAV.u0
        self._alpha = 0
        self._beta = 0
        #true state
        self.msg_true_state = StateMsg()

        self.sensors = msg_sensors()
        self._gps_eta_n = 0.0
        self._gps_eta_e = 0.0
        self._gps_eta_h = 0.0
        self._t_gps = 999. #timer so that gps only updates every ts_gps seconds

    ###################################
    # public functions
    def update_state(self, deltas, wind):
        '''
            Integrate the differential equations defining dynamics.
            Inputs are the forces and moments on the aircraft.
            Ts is the time step between function calls.
        '''
        forces_moments = self.calcForcesAndMoments(deltas)

        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self.ts_simulation
        k1 = self._derivatives(self._state, forces_moments)
        k2 = self._derivatives(self._state + time_step/2.*k1, forces_moments)
        k3 = self._derivatives(self._state + time_step/2.*k2, forces_moments)
        k4 = self._derivatives(self._state + time_step*k3, forces_moments)
        self._state += time_step/6 * (k1 + 2*k2 + 2*k3 + k4)

        # normalize the quaternion
        e0 = self._state.item(6)
        e1 = self._state.item(7)
        e2 = self._state.item(8)
        e3 = self._state.item(9)
        normE = np.sqrt(e0**2+e1**2+e2**2+e3**2)
        self._state[6][0] = self._state.item(6)/normE
        self._state[7][0] = self._state.item(7)/normE
        self._state[8][0] = self._state.item(8)/normE
        self._state[9][0] = self._state.item(9)/normE

        #update velocities
        self.updateVelocityData(wind)

        # update the message class for the true state
        self._update_msg_true_state()

    def updateSensors(self):
        theta = self.msg_true_state.theta
        phi = self.msg_true_state.phi
        g = MAV.gravity
        m = MAV.mass
        rho = MAV.rho
        sigma_a = SENSOR.accel_sigma
        sigma_g = SENSOR.gyro_sigma

        self.sensors.gyro_x = self.msg_true_state.p + SENSOR.gyro_x_bias + np.random.randn() * sigma_g
        self.sensors.gyro_y = self.msg_true_state.q + SENSOR.gyro_y_bias + np.random.randn() * sigma_g
        self.sensors.gyro_z = self.msg_true_state.r + SENSOR.gyro_z_bias + np.random.randn() * sigma_g
        self.sensors.accel_x = self._forces.item(0)/m  + g * np.sin(theta) + np.random.randn() * sigma_a
        self.sensors.accel_y = self._forces.item(1)/m - g * np.cos(theta) * np.sin(phi) + np.random.randn() * sigma_a
        self.sensors.accel_z = self._forces.item(2)/m - g * np.cos(theta) * np.cos(phi) + np.random.randn() * sigma_a
        self.sensors.static_pressure = rho * g * self.msg_true_state.h + np.random.randn() * SENSOR.static_pres_sigma
        self.sensors.diff_pressure = (rho * self.msg_true_state.Va**2)/2.0 + np.random.randn() * SENSOR.diff_pres_sigma

        if self._t_gps >= SENSOR.ts_gps:
            k_gps = SENSOR.gps_beta
            Ts = SENSOR.ts_gps

            self._gps_eta_n = np.exp(-k_gps * Ts) * self._gps_eta_n + np.random.randn() * SENSOR.gps_n_sigma
            self._gps_eta_e = np.exp(-k_gps * Ts) * self._gps_eta_e + np.random.randn() * SENSOR.gps_e_sigma
            self._gps_eta_h = np.exp(-k_gps * Ts) * self._gps_eta_h + np.random.randn() * SENSOR.gps_h_sigma
            self.sensors.gps_n = self.msg_true_state.pn + self._gps_eta_n
            self.sensors.gps_e = self.msg_true_state.pe + self._gps_eta_e
            self.sensors.gps_h = self.msg_true_state.h + self._gps_eta_h
            self.sensors.gps_Vg = self.msg_true_state.Vg + np.random.randn() * SENSOR.gps_Vg_sigma # Not sure this is right
            self.sensors.gps_course = self.msg_true_state.chi + np.random.randn() * SENSOR.gps_course_sigma #Not sure this is right
            self._t_gps = 0.0
        else:
            self._t_gps += self.ts_simulation

    ###################################
    # private functions
    def _derivatives(self, state, forces_moments):
        """
        for the dynamics xdot = f(x, u), returns f(x, u)
        """
        # extract the states
        pn = state.item(0)
        pe = state.item(1)
        pd = state.item(2)
        u = state.item(3)
        v = state.item(4)
        w = state.item(5)
        e0 = state.item(6)
        e1 = state.item(7)
        e2 = state.item(8)
        e3 = state.item(9)
        p = state.item(10)
        q = state.item(11)
        r = state.item(12)
        #   extract forces/moments
        fx = forces_moments.item(0)
        fy = forces_moments.item(1)
        fz = forces_moments.item(2)
        l = forces_moments.item(3)
        m = forces_moments.item(4)
        n = forces_moments.item(5)

        # position kinematics
        Rv_b = Quaternion2Rotation(np.array([e0, e1, e2, e3]))

        pos_dot = Rv_b @ np.array([u, v, w]).T
        pn_dot = pos_dot.item(0)
        pe_dot = pos_dot.item(1)
        pd_dot = pos_dot.item(2)

        # position dynamics
        u_dot = r*v - q*w + 1/MAV.mass * fx
        v_dot = p*w - r*u + 1/MAV.mass * fy
        w_dot = q*u - p*v + 1/MAV.mass * fz

        # rotational kinematics
        e0_dot = (-p * e1 - q * e2 - r * e3) * 0.5
        e1_dot = (p * e0 + r * e2 - q * e3) * 0.5
        e2_dot = (q * e0 - r * e1 + p * e3) * 0.5
        e3_dot = (r * e0 + q * e1 - p * e2) * 0.5

        # rotatonal dynamics
        p_dot = MAV.gamma1 * p * q - MAV.gamma2 * q * r + MAV.gamma3 * l + MAV.gamma4 * n
        q_dot = MAV.gamma5 * p * r - MAV.gamma6 * (p**2 - r**2) + 1/MAV.Jy * m
        r_dot = MAV.gamma7 * p * q - MAV.gamma1 * q * r + MAV.gamma4 * l + MAV.gamma8 * n

        # collect the derivative of the states
        x_dot = np.array([[pn_dot, pe_dot, pd_dot, u_dot, v_dot, w_dot,
                           e0_dot, e1_dot, e2_dot, e3_dot, p_dot, q_dot, r_dot]]).T
        return x_dot

    def _update_msg_true_state(self):
        # update the true state message:
        phi, theta, psi = Quaternion2Euler(self._state[6:10])
        self.msg_true_state.pn = self._state.item(0)
        self.msg_true_state.pe = self._state.item(1)
        self.msg_true_state.h = -self._state.item(2)
        self.msg_true_state.phi = phi
        self.msg_true_state.theta = theta
        self.msg_true_state.psi = psi
        self.msg_true_state.p = self._state.item(10)
        self.msg_true_state.q = self._state.item(11)
        self.msg_true_state.r = self._state.item(12)
        self.msg_true_state.Va = self._Va
        self.msg_true_state.alpha = self._alpha  # see line 164 updateVelocityData
        self.msg_true_state.beta = self._beta  # see line 167 updateVelocityData
        self.msg_true_state.Vg = np.linalg.norm(self._state[3:6])
        self.calcGammaAndChi()
        self.msg_true_state.wn = self._wind.item(0)
        self.msg_true_state.we = self._wind.item(1)

    def calcGammaAndChi(self):
        Rv_b = Quaternion2Rotation(self._state[6:10])
        Vg = Rv_b @ self._state[3:6]

        gamma = asin(-Vg.item(2)/np.linalg.norm(Vg)) #negative because h_dot = Vg sin(gamma)
        self.msg_true_state.gamma = gamma

        Vg_horz = Vg * np.cos(gamma)
        e1 = np.array([[1, 0, 0]]).T

        chi = acos(np.dot(e1.T, Vg_horz) / np.linalg.norm(Vg_horz))
        if(Vg_horz.item(1) < 0):
            chi *= -1
        self.msg_true_state.chi = chi


    def updateVelocityData(self, wind=np.zeros((6, 1))):
        Rb_v = Quaternion2Rotation(self._state[6:10]).T
        #wind in body frame
        self._wind = Rb_v @ wind[0:3] + wind[3:]
        V = self._state[3:6]

        #Compute Va
        Vr = V - self._wind
        self._Va = np.linalg.norm(Vr)

        #Compute alpha
        if Vr.item(0) == 0:
            self._alpha = np.sign(Vr.item(2)) * np.pi/2.0
        else:
            self._alpha = np.arctan2(Vr.item(2), Vr.item(0))

        #Compute beta
        temp = np.sqrt(Vr.item(0)**2 + Vr.item(2)**2)
        if temp == 0:
            self._beta = np.sign(Vr.item(1)) * np.pi/2.0
        else:
            self._beta = asin(Vr.item(1)/self._Va)

    def calcForcesAndMoments(self, delta):
        # Calculate gravitational forces in the body frame
        Rb_v = Quaternion2Rotation(self._state[6:10]).T
        fb_grav = Rb_v @ np.array([[0, 0, MAV.mass * MAV.gravity]]).T

        # Calculating longitudinal forces and moments
        fx, fz, m = self.calcLongitudinalForcesAndMoments(delta.item(0))
        fx += fb_grav.item(0)
        fz += fb_grav.item(2)

        # Calculating lateral forces and moments
        fy, l, n = self.calcLateralForcesAndMoments(delta.item(2), delta.item(3))
        fy += fb_grav.item(1)

        # Propeller force and moments
        #These may act a little fast
        fp, qp = self.calcThrustForceAndMoment(delta.item(1), self._Va)
        fx += fp
        l += -qp

        self._forces[0] = fx
        self._forces[1] = fy
        self._forces[2] = fz
        return np.array([[fx, fy, fz, l, m, n]]).T

    def calcThrustForceAndMoment(self, dt, Va):
        rho = MAV.rho
        D = MAV.D_prop

        V_in = MAV.V_max * dt

        a = (rho * D**5) / ((2 * np.pi)**2) * MAV.C_Q0
        b = (rho * (D**4) * MAV.C_Q1 * Va)/(2 * np.pi)  + (MAV.KQ**2)/MAV.R_motor
        c = rho * (D**3) * MAV.C_Q2 * (Va**2) - (MAV.KQ * V_in)/MAV.R_motor + MAV.KQ * MAV.i0

        Omega_op = (-b + np.sqrt((b**2) - 4 * a * c)) / (2. * a)
        J_op = (2 * np.pi * Va) / (Omega_op * D)

        CT = MAV.C_T2 * (J_op**2) + MAV.C_T1 * J_op + MAV.C_T0
        CQ = MAV.C_Q2 * (J_op**2) + MAV.C_Q1 * J_op + MAV.C_Q0

        # Qp = rho * (Omega_op / (2 * np.pi))**2 * (D**5) * CQ
        Qp = MAV.KQ * (1./MAV.R_motor * (V_in - MAV.KQ * Omega_op) - MAV.i0)
        Fp = CT * (rho * (Omega_op**2) * (D**4)) / ((2 * np.pi)**2)

        return Fp, Qp

    def calcLateralForcesAndMoments(self, da, dr):
        b = MAV.b
        Va = self._Va
        beta = self._beta
        p = self._state.item(10)
        r = self._state.item(12)
        rho = MAV.rho
        S = MAV.S_wing

        b2V = b / (2. * Va)
        q_bar = 0.5 * rho * (Va**2) * S

        # Calculating fy
        fy = q_bar * (MAV.C_Y_0 + MAV.C_Y_beta * beta + MAV.C_Y_p * b2V * p +\
             MAV.C_Y_r * b2V * r + MAV.C_Y_delta_a * da + MAV.C_Y_delta_r * dr)

        # Calculating l
        l = q_bar * b * (MAV.C_ell_0 + MAV.C_ell_beta * beta + MAV.C_ell_p * b2V * p +\
            MAV.C_ell_r * b2V * r + MAV.C_ell_delta_a * da + MAV.C_ell_delta_r * dr)

        # Calculating n
        n = q_bar * b * (MAV.C_n_0 + MAV.C_n_beta * beta + MAV.C_n_p * b2V * p +\
            MAV.C_n_r * b2V * r + MAV.C_n_delta_a * da + MAV.C_n_delta_r * dr)

        return fy, l, n

    def calcLongitudinalForcesAndMoments(self, de):
        M = MAV.M
        alpha = self._alpha
        alpha0 = MAV.alpha0
        rho = MAV.rho
        Va = self._Va
        S = MAV.S_wing
        q = self._state.item(11)
        c = MAV.c

        c2V = c / (2. * Va)
        q_bar = 0.5 * rho * (Va**2) * S
        e_negM = exp(-M * (alpha - alpha0))
        e_posM = exp(M * (alpha + alpha0))

        sigma_alpha = (1 + e_negM + e_posM) / ((1 + e_negM)*(1 + e_posM))

        CL_alpha = (1 - sigma_alpha) * (MAV.C_L_0 + MAV.C_L_alpha * alpha) + \
                    sigma_alpha * (2 * np.sign(alpha) * (np.sin(alpha)**2) * np.cos(alpha))
        F_lift = q_bar * (CL_alpha + MAV.C_L_q * c2V * q + MAV.C_L_delta_e * de)

        CD_alpha = MAV.C_D_p + ((MAV.C_L_0 + MAV.C_L_alpha * alpha)**2) / (np.pi * MAV.e * MAV.AR)
        F_drag = q_bar * (CD_alpha + MAV.C_D_q * c2V * q + MAV.C_D_delta_e * de)

        Rb_s = np.array([[np.cos(alpha), -np.sin(alpha)],
                         [np.sin(alpha), np.cos(alpha)]])
        fx_fz = Rb_s @ np.array([[-F_drag, -F_lift]]).T

        m = q_bar * c * (MAV.C_m_0 + MAV.C_m_alpha * alpha + MAV.C_m_q * c2V * q + MAV.C_m_delta_e * de)

        return fx_fz.item(0), fx_fz.item(1), m
