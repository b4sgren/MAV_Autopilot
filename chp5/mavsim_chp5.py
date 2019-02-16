"""
mavsim
    - Chapter 2 assignment for Beard & McLain, PUP, 2012
    - Update history:
        1/10/2019 - RWB
"""
import sys
sys.path.append('..')

from mav_viewer import MAV_Viewer
from mav_dynamics import mav_dynamics as Dynamics
import parameters.sim_params as SIM
from messages.state_msg import StateMsg
from data_viewer import data_viewer
from wind_simulation import wind_simulation
from trim import compute_trim
import numpy as np
from tools.tools import Quaternion2Euler

# initialize dynamics object
dyn = Dynamics(SIM.ts_sim)
wind = wind_simulation(SIM.ts_sim)

mav_view = MAV_Viewer()
data_view = data_viewer()

# initialize the simulation time
sim_time = SIM.t0

#get trim input and states
Va_star = 25.
gamma_star = 0.
trim_state, trim_input = compute_trim(dyn, Va_star, gamma_star)
delta = np.copy(trim_input)

# main simulation loop
print("Press Ctrl-Q to exit...")
while sim_time < SIM.t_end:
    #---Get the wind here
    if sim_time >= 10.0 and sim_time <=10.2:
        delta[3,0] = -0.75
    elif sim_time > 10.2 and sim_time <= 10.4:
        delta[3,0] = 0.75
    else:
        delta[3, 0] = trim_input.item(3)
    current_wind = np.zeros((6, 1)) # wind.update(dyn._Va)
    dyn.update_state(delta, current_wind)
    #-------update viewer---------------
    mav_view.update(dyn.msg_true_state)
    data_view.update(dyn.msg_true_state,
                    dyn.msg_true_state,
                    dyn.msg_true_state,
                    SIM.ts_sim)

    #-------increment time-------------
    sim_time += SIM.ts_sim
input("Press Enter to Close")
