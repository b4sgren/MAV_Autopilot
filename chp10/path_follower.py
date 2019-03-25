import numpy as np
from math import sin, cos, atan, atan2
import sys

sys.path.append('..')
from messages.msg_autopilot import msg_autopilot

class path_follower:
    def __init__(self):
        self.chi_inf = np.radians(80)   # approach angle for large distance from straight-line path
        self.k_path = 0.002  # proportional gain for straight-line path following 0.02
        self.k_orbit =  1.5 # proportional gain for orbit following 2.5
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
        q = np.array([[path.line_direction.item(0), path.line_direction.item(1), path.line_direction.item(2)]]).T
        chi_q = atan2(q.item(1), q.item(0))
        chi_q = self._wrap(chi_q, chi)

        R_i2p = np.array([[cos(chi_q), sin(chi_q), 0],
                          [-sin(chi_q), cos(chi_q), 0],
                          [0, 0, 1]])
        r = path.line_origin
        p = np.array([[state.pn, state.pe, -state.h]]).T

        # chi_command
        e = p - r
        ep = R_i2p @ e
        epy = ep.item(1)

        chi_d = self.chi_inf * (2./np.pi) * atan(self.k_path * epy)
        chi_c = chi_q - chi_d

        # Altitude command
        temp = np.cross(q.reshape(3), np.array([0, 0, 1])).reshape((3,1))
        n = temp / np.linalg.norm(temp)
        s = e - (e.T @ n) * n # s is a 3x3 matrix

        ss = np.sqrt(s.item(0)**2 + s.item(1)**2)
        qs = np.sqrt(q.item(0)**2 + q.item(1)**2)
        sd = q.item(2) / qs * ss

        hc = -path.line_origin.item(2) - sd

        self.autopilot_commands.airspeed_command = 25.0 #Is there a way to calculate this?
        self.autopilot_commands.course_command = chi_c
        self.autopilot_commands.altitude_command = hc
        self.autopilot_commands.phi_feedforward = np.radians(0.0)

    def _follow_orbit(self, path, state):
        Vg = state.Vg
        psi = state.psi
        chi = state.chi
        p = np.array([[state.pn, state.pe, -state.h]]).T
        d = p - path.orbit_center
        d_norm = np.linalg.norm(d)
        R = path.orbit_radius
        if path.orbit_direction == 'CW':
            dir = 1
        else:
            dir = -1

        #Calculate chi_commanded
        var_phi = atan2(d.item(1), d.item(0))
        chi0 = var_phi + dir * np.pi/2.0
        chi_c = chi0 + dir * atan(self.k_orbit * (d_norm - R)/R)

        #Calculate phi_ff
        phi_ff = atan(Vg**2 / (self.gravity * R * cos(chi - psi)))

        self.autopilot_commands.airspeed_command = 25.0
        self.autopilot_commands.course_command = chi_c
        self.autopilot_commands.altitude_command = -path.orbit_center.item(2)
        self.autopilot_commands.phi_feedforward = phi_ff

    def _wrap(self, chi_c, chi):
        while chi_c-chi > np.pi:
            chi_c = chi_c - 2.0 * np.pi
        while chi_c-chi < -np.pi:
            chi_c = chi_c + 2.0 * np.pi
        return chi_c
