"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
"""
import sys
import numpy as np
sys.path.append('..')
import parameters.control_parameters as AP
from pid_control import pid_control  # , pi_control, pd_control_with_rate
from messages.state_msg import StateMsg
from tools.tools import Quaternion2Euler
from tools.transfer_function import transfer_function


class autopilot:
    def __init__(self, ts_control):
        #Can i just use the PID control class with 0 for certain gain?
        # instantiate lateral controllers
        self.roll_from_aileron = pid_control( #was pd_control_with_rate
                        kp=AP.roll_kp,
                        kd=AP.roll_kd,
                        limit_h=np.radians(45),
                        limit_l=-np.radians(45))
        self.course_from_roll = pid_control( # was pi
                        kp=AP.course_kp,
                        ki=AP.course_ki,
                        Ts=ts_control,
                        limit_h=np.radians(30),
                        limit_l=-np.radians(30))
        self.sideslip_from_rudder = pid_control( # was pi
                        kp=AP.sideslip_kp,
                        ki=AP.sideslip_ki,
                        Ts=ts_control,
                        limit_h=np.radians(45),
                        limit_l=-np.radians(45))
        self.yaw_damper = transfer_function(
                        num=np.array([[AP.yaw_damper_kp, 0]]),
                        den=np.array([[1, 1/AP.yaw_damper_tau_r]]),
                        Ts=ts_control)

        # instantiate longitudinal controllers
        self.pitch_from_elevator = pid_control( # was pd_control with rate
                        kp=AP.pitch_kp,
                        kd=AP.pitch_kd,
                        limit_h=np.radians(45),
                        limit_l=-np.radians(45))
        self.altitude_from_pitch = pid_control( # was pi
                        kp=AP.altitude_kp,
                        ki=AP.altitude_ki,
                        Ts=ts_control,
                        limit_h=np.radians(30),
                        limit_l=-np.radians(30))
        self.airspeed_from_throttle = pid_control( # was pi
                        kp=AP.airspeed_throttle_kp,
                        ki=AP.airspeed_throttle_ki,
                        Ts=ts_control,
                        limit_h=1.0,
                        limit_l=0.0)
        self.commanded_state = StateMsg()

    def update(self, cmd, state):
        # lateral autopilot
        #add phi_feedforward to phi_c?
        psi_c = cmd.course_command

        phi_c = self.course_from_roll.update(psi_c, state.chi, rad_flag=True) + cmd.phi_feedforward
        delta_a = self.roll_from_aileron.update_with_rate(phi_c, state.phi, state.p)
        delta_r = self.yaw_damper.update(state.r)

        # longitudinal autopilot
        h_c = cmd.altitude_command
        theta_c = self.altitude_from_pitch.update(h_c, state.h)
        delta_e =  self.pitch_from_elevator.update_with_rate(theta_c, state.theta, state.q)
        delta_t = self.airspeed_from_throttle.update(cmd.airspeed_command, state.Va)

        # construct output and commanded states
        delta = np.array([[delta_e], [delta_t], [delta_a], [delta_r]])
        self.commanded_state.h = cmd.altitude_command
        self.commanded_state.Va = cmd.airspeed_command
        self.commanded_state.phi = phi_c
        self.commanded_state.theta = theta_c
        self.commanded_state.chi = cmd.course_command
        return delta, self.commanded_state

    def saturate(self, input, low_limit, up_limit):
        if input <= low_limit:
            output = low_limit
        elif input >= up_limit:
            output = up_limit
        else:
            output = input
        return output
