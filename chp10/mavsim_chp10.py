"""
mavsim
    - Chapter 2 assignment for Beard & McLain, PUP, 2012
    - Update history:
        1/10/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np

import parameters.sim_params as SIM

from messages.msg_autopilot import msg_autopilot
from messages.state_msg import StateMsg
from messages.msg_path import msg_path

from mav_dynamics import mav_dynamics as Dynamics
from data_viewer import data_viewer
from wind_simulation import wind_simulation
from autopilot import autopilot
from observer import observer
from tools.signals import signals
from path_viewer import path_viewer
from path_follower import path_follower

# initialize dynamics object
dyn = Dynamics(SIM.ts_sim)
wind = wind_simulation(SIM.ts_sim)
ctrl = autopilot(SIM.ts_sim)
obsv = observer(SIM.ts_sim)
path_follow = path_follower()

#path definition
path = msg_path()
path.flag = 'line'
# path.flag = 'orbit'

if path.flag == 'line':
    path.line_origin = np.array([[0.0, 0.0, -100.0]]).T
    path.line_direction = np.array([[0.5, 1.0, -0.5]]).T
    path.line_direction = path.line_direction / np.linalg.norm(path.line_direction)
else:
    path.orbit_center = np.array([[0.0, 0.0, -100.0]]).T
    path.orbit_radius = 300.0
    path.orbit_direction = 'CCW'

path_view = path_viewer()
data_view = data_viewer()

# initialize the simulation time
sim_time = SIM.t0

# main simulation loop
print("Press Ctrl-Q to exit...")
while sim_time < SIM.t_end:
    #-----------observer---------------------
    measurements = dyn.sensors
    estimated_state = obsv.update(measurements)

    #-----------path follower-----------------
    commands = path_follow.update(path, estimated_state)

    #-----------controller--------------------
    delta, commanded_state = ctrl.update(commands, estimated_state)

    #------------Physical System----------------------
    current_wind = wind.update(dyn._Va)
    dyn.update_state(delta, current_wind)
    dyn.updateSensors()

    #-------update viewer---------------
    path_view.update(path, dyn.msg_true_state)
    data_view.update(dyn.msg_true_state,
                    estimated_state,
                    commanded_state,
                    SIM.ts_sim)

    #-------increment time-------------
    sim_time += SIM.ts_sim
input("Press Enter to Close")
