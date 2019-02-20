import sys
sys.path.append('..')
import numpy as np
import pickle as pkl

data = []
with open("../trim_conditions.pkl", 'rb') as f:
    data = pkl.load(f)

trim_state = data[0]
trim_input = data[1]
a_phi1 = data[2]
a_phi2 = data[3]
a_beta1 = data[4]
a_beta2 = data[5]
a_theta1 = data[6]
a_theta2 = data[7]
a_theta3 = data[8]
a_V1 = data[9]
a_V2 = data[10]
a_V3 = data[11]

gravity = 9.8
sigma = 0.05
Va0 = np.linalg.norm(trim_state[3:6])

#----------roll loop-------------
zeta_phi = 0.707 # tuning parameters
wn_phi = 10.0

roll_kp = wn_phi ** 2 / a_phi2
roll_kd = (2 * zeta_phi * wn_phi - a_phi1) / a_phi2

#----------course loop-------------
zeta_chi = 0.707
wn_chi = 1.0/10.0 * wn_phi  # 10.0 is a tuning parameter

course_kp = (2 * zeta_chi * wn_chi * Va0) / gravity
course_ki = (Va0 * wn_chi**2) / gravity

#----------sideslip loop-------------
sideslip_ki = 0
sideslip_kp = 0

#----------yaw damper-------------
yaw_damper_tau_r = 0
yaw_damper_kp = 0

#----------pitch loop-------------
zeta_theta = 0.707
wn_theta = 5.0

pitch_kp = (wn_theta**2 - a_theta2)/a_theta3
pitch_kd = (2 * zeta_theta * wn_theta - a_theta1) / a_theta3
K_theta_DC = (pitch_kp * a_theta3) / (wn_theta**2)

#----------altitude loop-------------
zeta_h = 0.707
wn_h = 1.0/10.0 * wn_theta #10.0 is a tuning parameter

altitude_kp = (2 * zeta_h * wn_h) / (K_theta_DC * Va0)
altitude_ki = (wn_h**2) / (K_theta_DC  * Va0)
altitude_zone = 5.0  # This is in meters

#---------airspeed hold using throttle---------------
zeta_V = 0.707
wn_V = 2.0

airspeed_throttle_kp = (wn_V**2) / a_V2
airspeed_throttle_ki = (2 * zeta_V * wn_V - a_V1) / a_V2