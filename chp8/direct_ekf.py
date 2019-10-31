''' Full State Direct EKF instead of the cascaded EKF. See the supplement online for the details'''

import sys
sys.path.append("..")
import numpy as np 
import parameters.control_parameters as CTRL 
import parameters.sim_params as SIM 
import parameters.sensor_parameters as SENSOR 
import parameters.aerosonde_parameters as MAV
from tools.tools import Euler2Rotation
from messages.state_msg import StateMsg

class observer:
    def __init__(self, ts_control):
        self.estimated_state = StateMsg()
        self.xhat = np.zeros(12) # Do I need both this and est. state?. I think so

        # Covariance Matrices. Q and P (a little) are tuning parameters
        self.P = np.eye(12) * 0.5
        self.Q = np.diag(np.ones(12)) * 0.01
        self.R_gyro = np.eye(3) * SENSOR.gyro_sigma**2
        self.R_accel = np.eye(3) * SENSOR.accel_sigma**2
        self.R_gps = np.diag([SENSOR.gps_n_sigma**2, SENSOR.gps_e_sigma**2,
                            SENSOR.gps_Vg_sigma**2, SENSOR.gps_course_sigma**2])
        self.R_pseudo = np.diag([0.1, 0.1]) # This is for the pseudo wind triangle measurement
        self.R_static_p = SENSOR.static_pres_sigma**2
        self.R_diff_p = SENSOR.diff_pres_sigma**2

        self.N = 10  # Number of propagation steps in between measurement samples
        self.ts = SIM.ts_control/self.N
    
    def propagateState(self, measurement):
        phi = self.estimated_state.phi
        theta = self.estimated_state.theta 
        psi = self.estimated_state.psi
        R_bfromv = Euler2Rotation(phi, theta, psi)


    def update(self, measurement):
        debug = 1

        #propagation step

        #Measurement Updates

