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
import numpy as np

# initialize dynamics object
dyn = Dynamics(SIM.ts_sim)

mav_view = MAV_Viewer()
data_view = data_viewer()

# initialize the simulation time
sim_time = SIM.t0

# main simulation loop
while sim_time < SIM.t_end:
    # Will need to set the initial state to check stuff
    #-------vary forces to check viewer-------------
    fx = 0
    fy = 0
    fz = 0
    l = 0
    m = 0
    n = 0
    if sim_time < 8.0:
        fx = 50
    elif sim_time < 16.0:
        fy = 50
        dyn._state[3] = 0
    elif sim_time < 24.0:
        fz = 50
        dyn._state[4] = 0
    elif sim_time < 32.0:
        l = 0.01
        dyn._state[5] = 0
    elif sim_time < 40.0:
        m = 0.05
        dyn._state[10] = 0
        dyn._state[12] = 0
        if sim_time <= 32.02:
            dyn._state[6:10] = np.array([1, 0, 0, 0]).reshape((4, 1))
    elif sim_time < 48.0:
        n = 0.01
        dyn._state[11] = 0
        if sim_time <= 40.02:
            dyn._state[6:10] = np.array([1, 0, 0, 0]).reshape((4, 1))

    U = np.array([fx, fy, fz, l, m, n])

    dyn.update_state(U)
    #-------update viewer---------------
   mav_view.update(dyn.msg_true_state)
   data_view.update(dyn.msg_true_state,
                    dyn.msg_true_state,
                    dyn.msg_true_state,
                    SIM.tx_simulation)

    #-------increment time-------------
    sim_time += SIM.ts_sim

print("Press Ctrl-Q to exit...")
