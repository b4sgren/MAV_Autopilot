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

gravity = 9.8
sigma =
Va0 = np.linalg.norm(trim_state[3:6])

#----------roll loop-------------
zeta_phi = 0.707 # tuning parameters
wn_phi = 10.0

roll_kp = wn_phi ** 2 / a_phi2
roll_kd = (2 * zeta_phi * wn_phi - a_phi1) / a_phi2

#----------course loop-------------
zeta_chi = 0.707
wn_chi = 1.0

course_kp = (2 * zeta_chi * wn_chi * Va0) / gravity
course_ki = (Va0 * wn_chi**2) / gravity

#----------sideslip loop-------------
sideslip_ki =
sideslip_kp =

#----------yaw damper-------------
yaw_damper_tau_r =
yaw_damper_kp =

#----------pitch loop-------------
pitch_kp =
pitch_kd =
K_theta_DC =

#----------altitude loop-------------
altitude_kp =
altitude_ki =
altitude_zone =

#---------airspeed hold using throttle---------------
airspeed_throttle_kp =
airspeed_throttle_ki =
