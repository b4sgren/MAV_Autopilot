import numpy as np
from messages.msg_waypoints import msg_waypoints
from dubins_parameters import dubins_parameters

from IPython.core.debugger import Pdb

class planDubinsRRT():
    def __init__(self):
        self.waypoints = msg_waypoints()
        self.segmentLength = 500 # standard length of path segments
        self.dubins_params = dubins_parameters()

    def planPath(self, wpp_start, wpp_end, map, R):
        # desired down position is down position of end node
        pd = wpp_end.item(2)
        self.segmentLength = 3.5 * R

        # specify start and end nodes from wpp_start and wpp_end
        # format: N, E, D, cost, parentIndex, connectsToGoalFlag, chi
        start_node = np.array([wpp_start.item(0), wpp_start.item(1), pd, 0, -1, 0, mod(wpp_start.item(4))])
        v = wpp_end[0:2] - wpp_start[0:2]
        chi = np.arctan2(v.item(1), v.item(0))
        end_node = np.array([wpp_end.item(0), wpp_end.item(1), pd, 0, 0, 0, mod(chi)])

        # establish tree starting with the start node
        tree = np.empty((1, 7))
        tree[0,:] = start_node

        # check to see if start_node connects directly to end_node
        if ((np.linalg.norm(start_node[0:3] - end_node[0:3]) < self.segmentLength ) and not self.collision(start_node, end_node, map, R)
            and np.linalg.norm(start_node[0:3] - end_node[0:3]) >= 3*R):
            self.waypoints.ned = end_node[0:3] #This seems kinda dumb b/c only 1 waypoint. Need 3 for our thing
        else:
            numPaths = 0
            while numPaths < 1: #see if we get 3
                tree, flag = self.extendTree(tree, end_node, self.segmentLength, map, pd, R)
                numPaths = numPaths + flag

        # find path with minimum cost to end_node
        path = self.findMinimumPath(tree, end_node)
        self.smoothPath(path, map, R)
        return self.waypoints

    def generateRandomNode(self, map, pd):
        min = 0
        max = map.city_width
        p = np.random.uniform(min, max, 3)
        p[2] = pd

        return p

    def findClosestNode(self, tree, pt):
        n = pt.item(0)
        e = pt.item(1)

        dist = (n - tree[:,0])**2 + (e - tree[:,1])**2
        index = np.argmin(dist)

        return tree[index,:], index

    def planSegment(self, v_star, p):
        v = p[0:3] - v_star[0:3]
        v = v / np.linalg.norm(v)
        v_plus = v_star[0:3] + v * self.segmentLength
        chi = np.arctan2(v.item(1), v.item(0))

        # N, E, D, cost, parent node, connect to goal, chi
        return np.array([v_plus.item(0), v_plus.item(1), p.item(2), 0, 0, 0, chi])

    def collision(self, start_node, end_node, map, R):
        #need to edit this function
        delta = 10
        d_ang = np.radians(1)
        buf = 5

        # all pts on dubins path between 2 nodes
        # Pdb().set_trace()
        self.dubins_params.update(start_node[0:3].reshape((3,1)), start_node[-1], end_node[0:3].reshape((3,1)), end_node[-1], R)
        pts = self.pointsAlongPath(d_ang, delta)
        crashed = self.detectCrash(pts, map)

        return crashed

    def detectCrash(self, pts, map):
        buf = 5
        w = map.building_width
        for i in range(pts.shape[0]):
            pt = pts[i,:]
            #find the closes building in NE plane
            dst_n = (pt[0] - map.building_north)**2
            dst_e = (pt[1] - map.building_east)**2
            index_n = np.argmin(dst_n)
            index_e = np.argmin(dst_e)

            dn = map.building_north[index_n] - w/2
            de = map.building_east[index_e] - w/2
            if (pt[0] > dn - buf and pt[0] < dn + w + buf) \
               and (pt[1] > de - buf and pt[1] < de + w + buf)\
               and (pt[2] < map.building_height[index_e, index_n] + buf):
               return True
        return False

    def pointsAlongPath(self, Del_th, Del_lin):
        # points along start circle
        initialize_points = True
        th1 = np.arctan2(self.dubins_params.p_s.item(1) - self.dubins_params.center_s.item(1),
                        self.dubins_params.p_s.item(0) - self.dubins_params.center_s.item(0))
        th1 = mod(th1)
        th2 = np.arctan2(self.dubins_params.r1.item(1) - self.dubins_params.center_s.item(1),
                         self.dubins_params.r1.item(0) - self.dubins_params.center_s.item(0))
        th2 = mod(th2)
        th = th1
        theta_list = [th]
        if self.dubins_params.dir_s > 0:
            if th1 >= th2:
                while th < th2 + 2*np.pi:
                    th += Del_th
                    theta_list.append(th)
            else:
                while th < th2:
                    th += Del_th
                    theta_list.append(th)
        else:
            if th1 <= th2:
                while th > th2 - 2*np.pi:
                     th -= Del_th
                     theta_list.append(th)
            else:
                while th > th2:
                    th -= Del_th
                    theta_list.append(th)

        if initialize_points:
            points = np.array([[self.dubins_params.center_s.item(0) + self.dubins_params.radius * np.cos(theta_list[0]),
                                self.dubins_params.center_s.item(1) + self.dubins_params.radius * np.sin(theta_list[0]),
                                self.dubins_params.center_s.item(2)]])
            initialize_points = False
        for angle in theta_list:
            new_point = np.array([[self.dubins_params.center_s.item(0) + self.dubins_params.radius * np.cos(angle),
                                   self.dubins_params.center_s.item(1) + self.dubins_params.radius * np.sin(angle),
                                   self.dubins_params.center_s.item(2)]])
            points = np.concatenate((points, new_point), axis=0)

        # points along straight line
        vec = self.dubins_params.r2 - self.dubins_params.r1
        v = np.linalg.norm(vec)
        num_pts = np.ceil(v/Del_lin)
        # Pdb().set_trace()
        vec = vec/v
        for i in range(int(num_pts)):
            new_point = self.dubins_params.r1 + i * Del_lin * vec
            points = np.concatenate((points, new_point.T), axis=0)

        # points along end circle
        th2 = np.arctan2(self.dubins_params.p_e.item(1) - self.dubins_params.center_e.item(1),
                         self.dubins_params.p_e.item(0) - self.dubins_params.center_e.item(0))
        th2 = mod(th2)
        th1 = np.arctan2(self.dubins_params.r2.item(1) - self.dubins_params.center_e.item(1),
                         self.dubins_params.r2.item(0) - self.dubins_params.center_e.item(0))
        th1 = mod(th1)
        th = th1
        theta_list = [th]
        if self.dubins_params.dir_e > 0:
            if th1 >= th2:
                while th < th2 + 2 * np.pi:
                    th += Del_th
                    theta_list.append(th)
            else:
                while th < th2:
                    th += Del_th
                    theta_list.append(th)
        else:
            if th1 <= th2:
                while th > th2 - 2 * np.pi:
                    th -= Del_th
                    theta_list.append(th)
            else:
                while th > th2:
                    th -= Del_th
                    theta_list.append(th)
        for angle in theta_list:
            new_point = np.array([[self.dubins_params.center_e.item(0) + self.dubins_params.radius * np.cos(angle),
                                   self.dubins_params.center_e.item(1) + self.dubins_params.radius * np.sin(angle),
                                   self.dubins_params.center_e.item(2)]])
            points = np.concatenate((points, new_point), axis=0)

        return points

    def extendTree(self, tree, end_node, segmentLength, map, pd, R):
        pt = self.generateRandomNode(map, pd)
        v_star, index = self.findClosestNode(tree, pt)
        v_plus = self.planSegment(v_star, pt)
        flag = 0
        if not self.collision(v_star, v_plus, map, R):
            #append to tree
            cost = self.segmentLength + tree[index, 3]
            temp = np.array([v_plus[0], v_plus[1], v_plus[2], cost, index, flag, v_plus[-1]])
            tree = np.vstack((tree, temp))
            if not self.collision(v_plus, end_node, map, R):
                flag = 1
                v = end_node[0:2] - v_plus[0:2]
                chi = np.arctan2(v.item(1), v.item(0))
                cost +=  np.linalg.norm(v_plus[0:3])
                temp = np.array([end_node[0], end_node[1], end_node[2], cost, tree.shape[0]-1, flag, chi])
                tree = np.vstack((tree, temp))

        return tree, flag

    def findMinimumPath(self, tree, end_node):
        indices = np.nonzero(tree[:,-2])
        index = indices[0][np.argmin(tree[indices, 3])]
        #get the list of all points
        waypoints = [tree[index]]
        while tree[index,4] != -1:
            index = int(tree[index,4])
            waypoints.append(tree[index])
        return waypoints[::-1] # this reverses the list

    def smoothPath(self, path, map, R):
        i = 0
        j = 1
        smooth = [path[0]]

        while j < len(path)-1:
            node = path[i]
            next_node = path[j+1]
            if self.collision(node, next_node, map, R):
                last_node = path[j]
                v = last_node[0:2] - node[0:2]
                chi = np.arctan2(v.item(1), v.item(0))
                last_node[-1] = chi
                smooth.append(last_node)
                i = j
            j += 1
        smooth.append(path[-1])
        smooth = np.array(smooth)
        self.waypoints.ned = smooth[:, 0:3].T
        self.waypoints.num_waypoints = smooth.shape[0]
        self.waypoints.course = smooth[:,-1].T

def mod(x):
    # force x to be between 0 and 2*pi
    while x < 0:
        x += 2*np.pi
    while x > 2*np.pi:
        x -= 2*np.pi
    return x
