import sys
sys.path.append("..")

from messages.msg_waypoints import msg_waypoints
import numpy as np

class path_planner:
    def __init__(self):
        self.waypoints = msg_waypoints()

    def update(self, map, state):
        # planner_flag = 1  # return simple fillet path
        planner_flag = 2  # return dubins waypoint path
        # planner_flag = 3  # do straight line RRT through city
        # planner_flag = 4  # do dubins RRT through city

        if planner_flag == 1:
            self.waypoints.type = 'fillet'
            self.waypoints.num_waypoints = 4
            self.waypoints.ned[:,0:self.waypoints.num_waypoints] = np.array([[0, 0, -100],
                                                                   [1000, 0, -100],
                                                                   [0, 1000, -100],
                                                                   [1000, 1000, -100]]).T
            Va = 25
            self.waypoints.airspeed[:, 0:self.waypoints.num_waypoints] = np.array([[Va, Va, Va, Va]])
        elif planner_flag == 2:
            self.waypoints.type = 'dubins'
            self.waypoints.num_waypoints = 4
            self.waypoints.ned[:,0:self.waypoints.num_waypoints] = np.array([[0, 0, -100],
                                                                   [1000, 0, -100],
                                                                   [0, 1000, -100],
                                                                   [1000, 1000, -100]]).T
            Va = 25
            self.waypoints.airspeed[:, 0:self.waypoints.num_waypoints] = np.array([[Va, Va, Va, Va]])
            self.waypoints.course[:, 0:self.waypoints.num_waypoints] = np.array([[0,
                                                                        np.radians(45),
                                                                        np.radians(45),
                                                                        np.radians(-135)]])
        elif planner_flag == 3:
            debug = 1
        elif planner_flag == 4:
            debug = 1
        else:
            print("Unknown planner type")

        return self.waypoints
