"""
mav_dynamics
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state
    
part of mavsimPy
    - Beard & McLain, PUP, 2012
    - Update history:  
        12/17/2018 - RWB
        1/14/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np

# load message types
from messages.state_msg import StateMsg 

import parameters.aerosonde_parameters as MAV
from tools.tools import Quaternion2Euler

class mav_dynamics:
    def __init__(self, Ts):
        self.ts_simulation = Ts
        # set initial states based on parameter file
        # _state is the 13x1 internal state of the aircraft that is being propagated:
        # _state = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]
        self._state = np.array([
                                ])
        self.msg_true_state = msg_state()

    ###################################
    # public functions
    def update_state(self, forces_moments):
        '''

            Integrate the differential equations defining dynamics. 
            Inputs are the forces and moments on the aircraft.
            Ts is the time step between function calls.
        '''

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
        Rb_v = np.array([[e1**2 + e0**2 - e2**2 - e3**2, 2*(e1*e2 - e3*e0), 2*(e1*e3 + e2*e0)],
                         [2*(e1*e2 + e3*e0), e2**2 + e0**2 - e1**2 -e3**2, 2*(e2*e3 - e1*e0)],
                         [2*(e1*e3 - e2*e0), 2*(e2*e3 + e1*e0), e3**2 + e0**2 - e1**2 - e2**2]])

        pos_dot = Rb_v @ np.array([u, v, w]).T
        pn_dot = pos_dot[0]
        pe_dot = pos_dot[1]
        pd_dot = pos_dot[2]

        # position dynamics
        u_dot = r*v - q*w + 1/m * fx
        v_dot = p*w - r*u + 1/m * fy
        w_dot = q*u - p*v + 1/m * fz

        # rotational kinematics
        e0_dot = (-p * e1 -q * e2 -r * e3) * 0.5
        e1_dot = (p * e0 + r * e2 - q * e3) * 0.5
        e2_dot = (q * e0 -r * e1 + p * e3) * 0.5
        e3_dot = (r * e0 + q * e1 - p * e2) * 0.5

        # rotatonal dynamics
        gammas = self._getGammaValues(Jx, Jy, Jx, Jxz)
        p_dot = gammas[0] * p * q - gammas[1] * q * r + gammas[2] * l + gammas[3] * n
        q_dot = gammas[4] * p * r - gammas[5] * (p**2 - r**2) + 1/Jy * m
        r_dot = gammas[6] * p * q - gammas[0] * q * r + gammas[4] * l + gammas[7] * n

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

    def _getGammaValues(self, Jx, Jy, Jz, Jxz):
        gamma = Jx * Jz - Jxz**2
        gamma1 = (Jxz * (Jx - Jy + Jz))/gamma
        gamma2 = (Jz * (Jz - Jy) + Jxz**2)/gamma
        gamma3 = Jz/gamma
        gamma4 = Jxz/gamma
        gamma5 = (Jz - Jx)/Jy
        gamma6 = Jxz/Jy
        gamma7 = ((Jx - Jy)*Jx + Jxz**2)/gamma
        gamma8 = Jx/gamma

        return np.array([gamma1, gamma2, gamma3, gamma4, gamma5, gamma6, gamma7, gamma8])

