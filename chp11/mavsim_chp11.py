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
import parameters.planner_parameters as PLAN

from messages.msg_waypoints import msg_waypoints

from mav_dynamics import mav_dynamics as Dynamics
from data_viewer import data_viewer
from wind_simulation import wind_simulation
from autopilot import autopilot
from observer import observer
from tools.signals import signals
from waypoint_viewer import waypoint_viewer
from path_follower import path_follower
from path_manager import path_manager

# initialize dynamics object
dyn = Dynamics(SIM.ts_sim)
wind = wind_simulation(SIM.ts_sim)
ctrl = autopilot(SIM.ts_sim)
obsv = observer(SIM.ts_sim)
path_follow = path_follower()
path_manage = path_manager()

# waypoint definition
waypoints = msg_waypoints()
waypoints.type = 'straight_line'
waypoints.type = 'fillet'
#waypoints.type = 'dubins'
waypoints.num_waypoints = 5
Va = PLAN.Va0
waypoints.ned[:,0:waypoints.num_waypoints] = np.array([[0, 0, -100],
                                                       [1000, 0, -100],
                                                       [0, 1000, -100],
                                                       [1000, 1000, -100],
                                                       [0, 0, -100]]).T
waypoints.airspeed[:, 0:waypoints.num_waypoints] = np.array([[Va, Va, Va, Va]])
waypoints.course[:, 0:waypoints.num_waypoints] = np.array([[0,
                                                            np.radians(45),
                                                            np.radians(45),
                                                            np.radians(-135),
                                                            0]])

waypoint_view = waypoint_viewer()
data_view = data_viewer()

# initialize the simulation time
sim_time = SIM.t0

# main simulation loop
print("Press Ctrl-Q to exit...")
while sim_time < SIM.t_end:
    #-----------observer---------------------
    measurements = dyn.sensors
    estimated_state = obsv.update(measurements)

    #-----------path manager------------------
    path = path_manage.update(waypoints, PLAN.R_min, estimated_state)
    # print(path.line_origin)

    #-----------path follower-----------------
    commands = path_follow.update(path, estimated_state)

    #-----------controller--------------------
    delta, commanded_state = ctrl.update(commands, estimated_state)

    #------------Physical System----------------------
    current_wind = np.zeros((6,1)) # wind.update(dyn._Va)
    dyn.update_state(delta, current_wind)
    dyn.updateSensors()

    #-------update viewer---------------
    waypoint_view.update(waypoints, path, dyn.msg_true_state)
    data_view.update(dyn.msg_true_state,
                    estimated_state,
                    commanded_state,
                    SIM.ts_sim)

    #-------increment time-------------
    sim_time += SIM.ts_sim
input("Press Enter to Close")
