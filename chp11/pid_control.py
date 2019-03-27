"""
pid_control
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
"""
import sys
import numpy as np
sys.path.append('..')

class pid_control:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, Ts=0.01, sigma=0.05, limit_h=1.0, limit_l=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.Ts = Ts
        self.limit_h = limit_h
        self.limit_l = limit_l
        self.integrator = 0.0
        self.e_delay_1 = 0.0
        self.error_dot_delay_1 = 0.0
        self.y_delay_1 = 0.0
        self.y_dot = 0.0
        # gains for differentiator
        self.a1 = (2.0 * sigma - Ts) / (2.0 * sigma + Ts)
        self.a2 = 2.0 / (2.0 * sigma + Ts)

    def update(self, y_ref, y, rad_flag=False):
        error = y_ref - y
        if(rad_flag):
            while(error >= np.pi):
                error -= 2 * np.pi
            while(error < -np.pi):
                error += 2 * np.pi
            # print(error)

        self.integrateError(error)
        self.differentiate(y)

        u_unsat = self.kp * error + self.ki * self.integrator - self.kd * self.y_dot
        u_sat = self._saturate(u_unsat)

        if self.ki != 0:
            self.antiwindUp(u_unsat, u_sat)

        return u_sat

    def update_with_rate(self, y_ref, y, ydot, reset_flag=False): #This should work for PD, PI and PID
        error = y_ref - y
        self.integrateError(error)

        u_unsat = self.kp * error + self.ki * self.integrator - self.kd * ydot
        u_sat = self._saturate(u_unsat)

        if self.ki != 0:
            self.antiwindUp(u_unsat, u_sat)

        return u_sat

    def integrateError(self, e):
        self.integrator += self.Ts / 2.0 * (e + self.e_delay_1)
        self.e_delay_1 = e

    def differentiate(self, y):
        self.y_dot = self.a1 * self.y_dot + self.a2 * (y - self.y_delay_1)
        self.y_delay_1 = y

    def antiwindUp(self, u_unsat, u):
        self.integrator += self.Ts/self.ki * (u - u_unsat)

    def _saturate(self, u):
        # saturate u at +- self.limit
        # modify for upper and lower limit
        if u >= self.limit_h:
            u_sat = self.limit_h
        elif u <= self.limit_l:
            u_sat = self.limit_l
        else:
            u_sat = u
        return u_sat
