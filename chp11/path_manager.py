import numpy as np
import sys
sys.path.append('..')
# from dubins_parameters import dubins_parameters
from messages.msg_path import msg_path

class path_manager:
    def __init__(self):
        # message sent to path follower
        self.path = msg_path()
        # pointers to previous, current, and next waypoints
        self.ptr_previous = 0
        self.ptr_current = 1
        self.ptr_next = 2
        # flag that request new waypoints from path planner
        self.flag_need_new_waypoints = True
        self.num_waypoints = 0
        self.halfspace_n = np.inf * np.ones((3,1))
        self.halfspace_r = np.inf * np.ones((3,1))
        # state of the manager state machine
        self.manager_state = 1
        # dubins path parameters
        # self.dubins_path = dubins_parameters()

    def update(self, waypoints, radius, state):
        if waypoints.type == 'straight_line':
            self.line_manager(waypoints, state)
        elif waypoints.type == 'fillet':
            self.fillet_manager(waypoints, radius, state)
        elif waypoints.type == 'dubins':
            self.dubins_manager(waypoints, radius, state)
        else:
            print('Error in Path Manager: Undefined waypoint type.')
        return self.path

    def line_manager(self, waypoints, state):
        qi = waypoints.ned[:, self.ptr_next] - waypoints.ned[:, self.ptr_current]
        qi = qi / np.linalg.norm(qi)
        q_prev = waypoints.ned[:, self.ptr_current] - waypoints.ned[:, self.ptr_previous]
        q_prev = q_prev / np.linalg.norm(q_prev)

        n = q_prev + qi
        self.halfspace_n = n / np.linalg.norm(n).reshape((3,1))
        self.halfspace_r = waypoints.ned[:, self.ptr_previous].reshape((3,1))
        p = np.array([[state.pn, state.pe, -state.h]]).T

        crossed = self.inHalfSpace(p)

        if crossed:
            self.path.flag = 'line'
            self.path.airspeed = waypoints.airspeed.item(self.ptr_next)
            self.line_origin = waypoints.nedp[:, self.ptr_current].reshape((3,1))
            self.line_direction = qi.reshape((3,1))

            if not self.ptr_current == self.num_waypoints-1:
                self.ptr_previous += 1
                self.ptr_current += 1
                slef.ptr_next += 1



    def fillet_manager(self, waypoints, radius, state):
        debug = 1

    def dubins_manager(self, waypoints, radius, state):
        debug = 1

    def initialize_pointers(self):
        debug = 1

    def increment_pointers(self):
        debug = 1

    def inHalfSpace(self, pos):
        if (pos-self.halfspace_r).T @ self.halfspace_n >= 0:
            return True
        else:
            return False
