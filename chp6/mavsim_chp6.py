"""
mavsim
    - Chapter 2 assignment for Beard & McLain, PUP, 2012
    - Update history:
        1/10/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np

from mav_viewer import MAV_Viewer
from mav_dynamics import mav_dynamics as Dynamics
import parameters.sim_params as SIM
from messages.state_msg import StateMsg
from messages.msg_autopilot import msg_autopilot
from data_viewer import data_viewer
from wind_simulation import wind_simulation
from tools.tools import Quaternion2Euler
from autopilot import autopilot
from tools.signals import signals

# initialize dynamics object
dyn = Dynamics(SIM.ts_sim)
wind = wind_simulation(SIM.ts_sim)
ctrl = autopilot(SIM.ts_sim)

# autopilot commands
commands = msg_autopilot()
Va_command = signals(dc_offset=25.0, amplitude=3.0, start_time=2.0, frequency = 0.01)
h_command = signals(dc_offset=100.0, amplitude=10.0, start_time=0.0, frequency = 0.02)
chi_command = signals(dc_offset=np.radians(180), amplitude=np.radians(45), start_time=5.0, frequency = 0.015)

mav_view = MAV_Viewer()
data_view = data_viewer()

# initialize the simulation time
sim_time = SIM.t0

# main simulation loop
print("Press Ctrl-Q to exit...")
while sim_time < SIM.t_end:
    #-------controller-------------
    estimated_state = dyn._state  # uses true states in the control
    commands.airspeed_command = Va_command.square(sim_time)
    commands.course_command = chi_command.square(sim_time)
    commands.altitude_command = h_command.square(sim_time)
    delta, commanded_state = ctrl.update(commands, estimated_state)

    #---Get the wind here
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
