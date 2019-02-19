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
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, Ts=0.01, sigma=0.05, limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.Ts = Ts
        self.limit = limit
        self.integrator = 0.0
        self.error_delay_1 = 0.0 # Do I need to differentiate error?
        self.error_dot_delay_1 = 0.0 # Do we need to differentiate error?
        self.y_delay_1 = 0.0
        self.y_dot = 0.0
        # gains for differentiator
        self.a1 = (2.0 * sigma - Ts) / (2.0 * sigma + Ts)
        self.a2 = 2.0 / (2.0 * sigma + Ts)

    def update(self, y_ref, y, reset_flag=False):
        error = y_ref - y
        self.integrateError(error)
        self.differentiate(y)

        u_unsat = self.kP * error + self.ki * integrator - self.kd * self.y_dot
        u_sat = self.saturate(u_unsat)

        if self.ki != 0:
            self.antiwindUp(u_unsat, u)

        return u_sat

    def update_with_rate(self, y_ref, y, ydot, reset_flag=False): #This should work for PD, PI and PID
        error = y_ref - y
        self.integrateError(error)

        u_unsat = self.kP * error + self.ki * integrator - self.kd * ydot
        u_sat = self.saturate(u_unsat)

        if self.ki != 0:
            self.antiwindUp(u_unsat, u)

        return u_sat

    def integrateError(self, e):
        self.integrator += self.Ts / 2.0 * (e - self.e_delay_1)
        self.e_delay_1 = e

    def differentiate(self, y):
        self.y_dot = self.a1 * self.y_dot + self.a2 * (y - self.y_delay_1)
        self.y_delay_1 = y

    def antiwindUp(self, u_unsat, u):
        self.integrator += self.Ts/self.ki * (u - u_unsat)

    def _saturate(self, u):
        # saturate u at +- self.limit
        if u >= self.limit:
            u_sat = self.limit
        elif u <= -self.limit:
            u_sat = -self.limit
        else:
            u_sat = u
        return u_sat

# class pi_control: # I don't think I need this
#     def __init__(self, kp=0.0, ki=0.0, Ts=0.01, limit=1.0):
#         self.kp = kp
#         self.ki = ki
#         self.Ts = Ts
#         self.limit = limit
#         self.integrator = 0.0
#         self.error_delay_1 = 0.0
#
#     def update(self, y_ref, y):
#         return u_sat
#
#     def _saturate(self, u):
#         # saturate u at +- self.limit
#         if u >= self.limit:
#             u_sat = self.limit
#         elif u <= -self.limit:
#             u_sat = -self.limit
#         else:
#             u_sat = u
#         return u_sat

# class pd_control_with_rate: # Not sure I need this either
#     # PD control with rate information
#     # u = kp*(yref-y) - kd*ydot
#     def __init__(self, kp=0.0, kd=0.0, limit=1.0):
#         self.kp = kp
#         self.kd = kd
#         self.limit = limit
#
#     def update(self, y_ref, y, ydot):
#         return u_sat
#
#     def _saturate(self, u):
#         # saturate u at +- self.limit
#         if u >= self.limit:
#             u_sat = self.limit
#         elif u <= -self.limit:
#             u_sat = -self.limit
#         else:
#             u_sat = u
#         return u_sat
