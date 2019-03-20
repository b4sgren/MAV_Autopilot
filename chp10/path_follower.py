import numpy as np
from math import sin, cos, atan, atan2
import sys

sys.path.append('..')
from messages.msg_autopilot import msg_autopilot

class path_follower:
    def __init__(self):
        self.chi_inf = np.radians(80)   # approach angle for large distance from straight-line path
        self.k_path = 0.05  # proportional gain for straight-line path following
        self.k_orbit =  0.05 # proportional gain for orbit following
        self.gravity = 9.8
        self.autopilot_commands = msg_autopilot()  # message sent to autopilot

    def update(self, path, state):
        if path.flag=='line':
            self._follow_straight_line(path, state)
        elif path.flag=='orbit':
            self._follow_orbit(path, state)
        return self.autopilot_commands

    def _follow_straight_line(self, path, state):
        Vg = state.Vg
        chi = state.chi
        qn = path.line_direction.item(0)
        qe = path.line_direction.item(1)
        qd = path.line_direction.item(2)
        chi_q = atan2(qe, qn)

        R_i2p = np.array([[cos(chi_q), sin(chi_q), 0],
                          [-sin(chi_q), cos(chi_q), 0],
                          [0, 0, 1]])
        r = path.line_origin
        p = np.array([[state.pn, state.pe, -state.h]]).T

        # chi_command
        e = R_i2p @ (p - r)
        epy = e.item(1)

        chi_d = self.chi_inf * (2./np.pi) * atan(self.k_path * epy)
        chi_d = self._wrap(chi_d, chi_q)
        chi_c = chi_q - chi_d

        # Altitude command
        s = np.sqrt(e.item(0)**2 + e.item(1)**2)
        q = np.sqrt(qn**2 + qe**2)

        sd = qd / q * s

        hc = -path.line_origin.item(2) + sd

        self.autopilot_commands.airspeed_command = 25.0 #Is there a way to calculate this?
        self.autopilot_commands.course_command = chi_c
        self.autopilot_commands.altitude_command = hc
        self.autopilot_commands.phi_feedforward = np.radians(0.0)

    def _follow_orbit(self, path, state):
        self.autopilot_commands.airspeed_command = 0.0
        self.autopilot_commands.course_command = 0.0
        self.autopilot_commands.altitude_command = 0.0
        self.autopilot_commands.phi_feedforward = np.radians(0.0)

    def _wrap(self, chi_c, chi):
        while chi_c-chi > np.pi:
            chi_c = chi_c - 2.0 * np.pi
        while chi_c-chi < -np.pi:
            chi_c = chi_c + 2.0 * np.pi
        return chi_c
