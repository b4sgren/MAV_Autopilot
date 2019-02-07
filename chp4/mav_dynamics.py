"""
mav_dynamics
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state

"""
import sys
sys.path.append('..')
import numpy as np
from math import asin, exp

# load message types
from messages.state_msg import StateMsg

import parameters.aerosonde_parameters as MAV
from tools.tools import Quaternion2Euler, Quaternion2Rotation

class mav_dynamics:
    def __init__(self, Ts):
        self.ts_simulation = Ts
        # set initial states based on parameter file
        # _state is the 13x1 internal state of the aircraft that is being propagated:
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
        # TODO Change the line below to use Quat2Rot
        Rv_b = np.array([[e1**2 + e0**2 - e2**2 - e3**2, 2*(e1*e2 - e3*e0), 2*(e1*e3 + e2*e0)],
                         [2*(e1*e2 + e3*e0), e2**2 + e0**2 - e1**2 - e3**2, 2*(e2*e3 - e1*e0)],
                         [2*(e1*e3 - e2*e0), 2*(e2*e3 + e1*e0), e3**2 + e0**2 - e1**2 - e2**2]])

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
        self.gamma = np.arctan2(-self._state.item(5), self._state.item(3)) # is this right atan2(-w, u)
        self.chi = np.arctan2(self._state.item(4), self._state.item(3)) + psi # atan2(v, u)

    def updateVelocityData(self, wind=np.zeros((6, 1))):
        Rb_v = Quaternion2Rotation(self._state[6:10])
        #wind in body frame
        self._wind = Rb_v @ wind[0:3] + wind[3:]
        V = self._state[3:6]

        #Compute Va
        Vr = V - self._wind
        self._Va = np.linalg.norm(Vr)

        #Compute alpha
        self._alpha = np.arctan2(Vr.item(2), Vr.item(0))

        #Compute beta
        self._beta = asin(Vr.item(1)/self._Va)

    def calcForcesAndMoments(self, delta):
        # Calculate gravitational forces in the body frame
        fb_grav = Quaternion2Rotation(self._state[6:10]).T @ np.array([[0, 0, MAV.mass * MAV.gravity]]).T

        # Calculating longitudinal forces and moments
        fx, fz, m = self.calcLongitudinalForcesAndMoments(delta.item(0))
        fx += fb_grav[0]
        fz += fb_grav[2]

        # Calculating lateral forces and moments
        fy, l, n = self.calcLateralForcesAndMoments(delta.item(2), delta.item(3))
        fy += fb_grav[1]

        # Propeller force and moments
        #These may act a little fast
        fp, mp = self.calcThrustForceAndMoment(delta.item(1))
        print(fp)
        # fx += fp
        # l += mp

        return np.array([fx, fy, fz, l, m, n])

    def calcThrustForceAndMoment(self, dt):
        rho = MAV.rho
        D = MAV.D_prop
        Va = self._Va

        V_in = MAV.V_max * dt

        a = (rho * D**5) / ((2 * np.pi)**2) * MAV.C_Q0
        b = (rho * (D**4) * MAV.C_Q1 * Va)/(2 * np.pi)  + (MAV.KQ **2)/MAV.R_motor
        c = rho * (D**3) * MAV.C_Q2 * (Va**2) - (MAV.KQ * V_in)/MAV.R_motor + MAV.KQ * MAV.i0

        Omega_op = (-b + np.sqrt((b**2) - 4 * a * c)) / (2 * a)
        J_op = (2 * np.pi * Va) / (Omega_op * D)

        CT = MAV.C_T2 * (J_op**2) + MAV.C_T1 * J_op + MAV.C_T0

        Qp = MAV.KQ * ((V_in - MAV.K_V * Omega_op)/MAV.R_motor - MAV.i0)
        Fp = CT * (rho * (Omega_op**2) * (D**4)) / ((2 * np.pi)**2)

        # S_prop = .2027
        # C_prop = 1.0
        # km = 80
        # kTp = 0
        # k_omega = 0
        #
        # Fp = 0.5 * rho * S_prop * C_prop * (km * dt)**2 - Va**2
        # Qp = -kTp * ((k_omega * dt)**2)

        return Fp, Qp

    def calcLateralForcesAndMoments(self, da, dr):
        b = MAV.b
        Va = self._Va
        beta = self.msg_true_state.beta
        p = self.msg_true_state.p
        r = self.msg_true_state.r
        rho = MAV.rho
        S = MAV.S_wing

        # Calculating fy
        fy = 1/2.0 * rho * (Va**2) * S * (MAV.C_Y_0 + MAV.C_Y_beta * beta + MAV.C_Y_p * (b / (2*Va)) * p +\
             MAV.C_Y_r * (b / (2 * Va)) * r + MAV.C_Y_delta_a * da + MAV.C_Y_delta_r * dr)

        # Calculating l
        l = 1/2.0 * rho * (Va**2) * S * b * (MAV.C_ell_0 + MAV.C_ell_beta * beta + MAV.C_ell_p * (b/(2*Va)) * p +\
            MAV.C_ell_r * (b/(2*Va)) * r + MAV.C_ell_delta_a * da + MAV.C_ell_delta_r * dr)

        # Calculating n
        n = 1/2.0 * rho * (Va**2) * S * b * (MAV.C_n_0 + MAV.C_n_beta * beta + MAV.C_n_p * (b/(2*Va)) * p +\
            MAV.C_n_r * (b/(2*Va)) * r + MAV.C_n_delta_a * da + MAV.C_n_delta_r * dr)

        return fy, l, n

    def calcLongitudinalForcesAndMoments(self, de):
        M = MAV.M
        alpha = self._alpha
        alpha0 = MAV.alpha0
        rho = MAV.rho
        Va = self._Va
        S = MAV.S_wing
        q = self.msg_true_state.q
        c = MAV.c

        sigma_alpha = (1 + exp(-M * (alpha - alpha0)) + exp(M * (alpha + alpha0))) /\
                      ((1 + exp(-M * (alpha - alpha0)))*(1 + exp(M * (alpha + alpha0))))
        CL_alpha = (1 - sigma_alpha) * (MAV.C_L_0 + MAV.C_L_alpha * alpha) + \
                    sigma_alpha * (2 * np.sign(alpha) * (np.sin(alpha)**2) * np.cos(alpha))

        F_lift = 0.5 * rho * (Va**2) * S * (CL_alpha + MAV.C_L_q * (c / (2 * Va)) * q \
                 + MAV.C_L_delta_e * de)

        CD_alpha = MAV.C_D_p + ((MAV.C_L_0 + MAV.C_L_alpha * alpha)**2) / (np.pi * MAV.e * MAV.AR)

        F_drag = 0.5 * rho * (Va**2) * S * (CD_alpha + MAV.C_D_q * (c / (2 * Va)) * q \
                 + MAV.C_D_delta_e * de)

        Rb_s = np.array([[np.cos(alpha), -np.sin(alpha)],
                         [np.sin(alpha), np.cos(alpha)]])
        fx_fz = Rb_s @ np.array([[-F_drag, -F_lift]]).T

        m = 0.5 * rho * (Va**2) * S * c * (MAV.C_m_0 + MAV.C_m_alpha * alpha + \
            MAV.C_m_q * (c / (2 * Va)) * q + MAV.C_m_delta_e * de)

        return fx_fz.item(0), fx_fz.item(1), m
