import numpy as np
import sys
sys.path.append('..')
# from dubins_parameters import dubins_parameters
from messages.msg_path import msg_path
import parameters.planner_parameters as PLAN

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
        #check if waypoints change and reinitialize
        if waypoints.flag_waypoints_changed:
            self.num_waypoints = waypoints.num_waypoints
            self.flag_need_new_waypoints = False
            self.initialize_pointers()
            waypoints.flag_waypoints_changed = False
            self.manager_state = 1

        # if self.path.flag_path_changed:
        #     self.path.flag_path_changed = False
        if waypoints.num_waypoints == 0:
            waypoints.flag_manager_requests_waypoints = True
        else:
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
        qi = qi / np.linalg.norm(qi) # issue here when not a next waypoint
        q_prev = waypoints.ned[:, self.ptr_current] - waypoints.ned[:, self.ptr_previous]
        q_prev = q_prev / np.linalg.norm(q_prev)

        n = q_prev + qi
        self.halfspace_n = (n / np.linalg.norm(n)).reshape((3,1))
        self.halfspace_r = waypoints.ned[:, self.ptr_current].reshape((3,1))
        p = np.array([[state.pn, state.pe, -state.h]]).T

        # self.path.flag_path_changed = False
        self.path.flag = 'line'
        self.path.airspeed = waypoints.airspeed.item(self.ptr_current)
        self.path.line_origin = waypoints.ned[:, self.ptr_previous].reshape((3,1))
        self.path.line_direction = q_prev.reshape((3,1))

        crossed = self.inHalfSpace(p)

        if crossed:
            self.path.flag_path_changed = True
            self.increment_pointers()
        else:
            self.path.flag_path_changed = False


    def fillet_manager(self, waypoints, radius, state):
        w_next = waypoints.ned[:, self.ptr_next].reshape((3,1))
        w_current = waypoints.ned[:, self.ptr_current].reshape((3,1))
        w_prev = waypoints.ned[:, self.ptr_previous].reshape((3,1))
        qi = w_next - w_current
        qi = (qi / np.linalg.norm(qi))
        q_prev = w_current - w_prev
        q_prev = (q_prev / np.linalg.norm(q_prev))

        var_theta = np.arccos(-q_prev.T @ qi)
        p = np.array([[state.pn, state.pe, -state.h]]).T

        if self.manager_state == 1:  #straight line part
            z = w_current - (radius/np.tan(var_theta/2.)) * q_prev
            self.halfspace_r = z #algorithm says previous
            self.halfspace_n = q_prev

            self.path.flag = 'line'
            self.path.airspeed = waypoints.airspeed.item(self.ptr_current)
            self.path.line_origin = w_prev
            self.path.line_direction = q_prev

            crossed = self.inHalfSpace(p)
            if crossed:
                self.manager_state = 2
                self.path.flag_path_changed = True
            else:
                self.path.flag_path_changed = False
        else:
            q_temp = q_prev - qi
            q_temp = q_temp / np.linalg.norm(q_temp)
            c = w_current - (radius/np.sin(var_theta/2.)) * q_temp
            # print(c)

            z = w_current + (radius/np.tan(var_theta/2.)) * q_temp # was qi
            dir = np.sign(q_prev.item(0)*qi.item(1) - q_prev.item(1) * qi.item(0))

            self.path.flag = 'orbit'
            self.path.airspeed = waypoints.airspeed.item(self.ptr_current)
            self.path.orbit_center = c
            self.path.orbit_radius = radius
            if dir > 0:
                self.path.orbit_direction = 'CW'
            else:
                self.path.orbit_direction = 'CCW'

            self.halfspace_r = z
            self.halfspace_n = qi
            crossed = self.inHalfSpace(p)
            if crossed:
                self.increment_pointers()
                self.manager_state = 1
                self.path.flag_path_changed = True
            else:
                self.path.flag_path_changed = False

    def dubins_manager(self, waypoints, radius, state):
        p = np.array([[state.pn, state.pe, -state.h]]).T
        chi = state.chi

        # First circle centers
        cr = p + radius * np.array([[np.cos(chi + np.pi/2), np.sin(chi + np.pi/2), 0]]).T
        cl = p + radius * np.array([[np.cos(chi - np.pi/2), np.sin(chi - np.pi/2), 0]]).T 

    def initialize_pointers(self):
        self.ptr_previous = 0
        self.ptr_current = 1
        self.ptr_next = 2

    def increment_pointers(self):
        if  self.ptr_current < self.num_waypoints-1:
            self.ptr_previous += 1
            self.ptr_current += 1
            self.ptr_next += 1
        else:
            self.flag_need_new_waypoints = True

    def inHalfSpace(self, pos):
        if (pos-self.halfspace_r).T @ self.halfspace_n >= 0:
            return True
        else:
            return False
