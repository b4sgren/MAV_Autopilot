import numpy as np
from messages.msg_waypoints import msg_waypoints
from dubins_parameters import dubins_parameters

from IPython.core.debugger import Pdb

class planDubinsRRT():
    def __init__(self):
        self.waypoints = msg_waypoints()
        self.segmentLength = 300 # standard length of path segments
        self.dubins_params = dubins_parameters()

    def planPath(self, wpp_start, wpp_end, map):
        # desired down position is down position of end node
        pd = wpp_end.item(2)

        # specify start and end nodes from wpp_start and wpp_end
        # format: N, E, D, cost, parentIndex, connectsToGoalFlag, chi
        start_node = np.array([wpp_start.item(0), wpp_start.item(1), pd, 0, -1, 0, wpp_start.item(4)])
        v = wpp_end[0:2] - wpp_start[0:2]
        chi = np.arctan2(v.item(1), v.item(0))
        end_node = np.array([wpp_end.item(0), wpp_end.item(1), pd, 0, 0, 0, chi])

        # establish tree starting with the start node
        tree = np.empty((1, 7))
        tree[0,:] = start_node

        # check to see if start_node connects directly to end_node
        if ((np.linalg.norm(start_node[0:3] - end_node[0:3]) < self.segmentLength ) and not self.collision(start_node, end_node, map)):
            self.waypoints.ned = end_node[0:3] #This seems kinda dumb b/c only 1 waypoint. Need 3 for our thing
        else:
            numPaths = 0
            while numPaths < 3:
                tree, flag = self.extendTree(tree, end_node, self.segmentLength, map, pd)
                numPaths = numPaths + flag

        # find path with minimum cost to end_node
        path = self.findMinimumPath(tree, end_node)
        self.smoothPath(path, map)
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
        chi = np.arctan2(v.item(1), v.item(2))

        # N, E, D, cost, parent node, connect to goal, chi
        return np.array([v_plus.item(0), v_plus.item(1), p.item(2), 0, 0, 0, chi])

    def collision(self, start_node, end_node, map):
        #need to edit this function
        R = 150
        delta = 10
        d_ang = np.radians(1)
        buf = 5

        # collisions on first circle
        self.dubins_params.update(start_node[0:3], start_node[-1], end_node[0:3], end_node[-1], R)
        chi_s = start_node[-1]
        d2 = np.linalg.norm(self.dubins_params.r1 - start_node[0:3])
        alpha = np.arccos((2 * R**2 - d2)/(2* R**2))
        chi_e1 = chi_s - alpha
        pts = self.pointsAlongOrbit(self.dubins_params.center_s, d_ang * -self.dubins_params.dir_s, # CHECK I think I need the negative in multiplication
                                     chi_s, chi_e1, R)
        crashed = self.detectCrash(pts, map)
        if crashed:
            return crashed

        #collisions on straight line
        pts = self.pointsAlongPath(self.dubins_params.r2, self.dubins_params.r3, delta)
        crashed = self.detectCrash(pts, map)
        if crashed:
            return crashed

        #collisions on second circle
        d2 = np.linalg.norm(self.dubins_params.r3 - self.dubins_params.r2)
        alpha = np.arccos((2 * R**2 - d2)/(2* R**2))
        chi_e2 = chi_e1 - alpha
        pts = self.pointsAlongOrbit(self.dubins_params.center_e, d_ang * -self.dubins_params.dir_e,  # CHECK I think I need the negative in multiplication
                                     chi_e1, chi_e2, R)
        crashed = self.detectCrash(pts, map)
        if crashed:
            return crashed

        return False

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

    def pointsAlongOrbit(self, center, Del, chi_s, chi_e, R):
        pts = np.empty((1, 3))
        pd = center.item(2)

        while abs(chi_s - chi_e) > 1:
            n = center.item(0) + R * np.sin(chi_s)
            e = center.item(1) + R * np.cos(chi_s)
            chi_s += Del
            pts = np.vstack((pts, np.array([[n, e, pd]])))
            if chi_s > np.pi:
                chi_s -= 2 * np.pi
            if chi_s < -np.pi:
                chi_s += 2 * np.pi
        return pts

    def pointsAlongPath(self, start_node, end_node, Del):
        vec = end_node[0:3] - start_node[0:3]
        v = np.linalg.norm(vec)
        num_pts = np.ceil(v/Del)
        vec = vec/v

        points = np.empty((1,3))
        # Pdb().set_trace()
        points = np.vstack((points, start_node[0:3]))
        for i in range(int(num_pts)):
            temp = points[-1,:] + vec * Del
            # temp = temp.reshape((1,3))
            points = np.vstack((points, temp))

        return points

    def extendTree(self, tree, end_node, segmentLength, map, pd):
        pt = self.generateRandomNode(map, pd)
        v_star, index = self.findClosestNode(tree, pt)
        v_plus = self.planSegment(v_star, pt)
        flag = 0
        if not self.collision(v_star, v_plus, map):
            #append to tree
            cost = self.segmentLength + tree[index, 3]
            temp = np.array([v_plus[0], v_plus[1], v_plus[2], cost, index, flag, v_plus[-1]])
            tree = np.vstack((tree, temp))
            if not self.collision(v_plus, end_node, map):
                flag = 1
                v = end_node[0:2] - v_plus[0:2]
                chi = np.arctan2(v.item(1), v.item(0))
                cost +=  np.linalg.norm(v_plus[0:3])
                temp = np.array([end_node[0], end_node[1], end_node[2], cost, tree.shape[0]-1, flag, chi])
                tree = np.vstack((tree, temp))

        return tree, flag

    def findMinimumPath(self, tree, end_node):
        indices = np.nonzero(tree[:,-1])
        index = indices[0][np.argmin(tree[indices, 3])]
        #get the list of all points
        waypoints = [tree[index]]
        while tree[index,4] != -1:
            index = int(tree[index,4])
            waypoints.append(tree[index])
        return waypoints[::-1] # this reverses the list

    def smoothPath(self, path, map):
        i = 0
        j = 1
        smooth = [path[0]]

        while j < len(path)-1:
            node = path[i]
            next_node = path[j+1]
            if self.collision(node, next_node, map):
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
