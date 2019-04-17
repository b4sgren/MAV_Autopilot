import numpy as np
from messages.msg_waypoints import msg_waypoints

from IPython.core.debugger import Pdb

class planRRT():
    def __init__(self):
        self.waypoints = msg_waypoints()
        self.segmentLength = 300 # standard length of path segments

    def planPath(self, wpp_start, wpp_end, map):
        # desired down position is down position of end node
        pd = wpp_end.item(2)

        # specify start and end nodes from wpp_start and wpp_end
        # format: N, E, D, cost, parentIndex, connectsToGoalFlag,
        start_node = np.array([wpp_start.item(0), wpp_start.item(1), pd, 0, -1, 0])
        end_node = np.array([wpp_end.item(0), wpp_end.item(1), pd, 0, 0, 0])

        # establish tree starting with the start node
        tree = np.empty((1, 6))
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

        # N, E, D, cost, parent node, connect to goal
        return np.array([v_plus.item(0), v_plus.item(1), p.item(2), 0, 0, 0])

    def collision(self, start_node, end_node, map):
        delta = 10
        pts = self.pointsAlongPath(start_node, end_node, delta)

        for i in range(pts.shape[0]):
            pt = pts[i,:]
            #find the closes building in NE plane
            dst_n = (pt[0] - map.building_north)**2
            dst_e = (pt[1] - map.building_east)**2
            index_n = np.argmin(dst_n)
            index_e = np.argmin(dst_e)

            buf = 5
            w = map.building_width
            dn = map.building_north[index_n] - w/2
            de = map.building_east[index_e] - w/2
            # if (pt[0] > dst_n[index_n] - buf and pt[0] < dst_n[index_n] + map.building_width + buf) \
            #    and (pt[1] > dst_e[index_e] - buf and pt[1] < dst_e[index_e] + map.building_width + buf) \
            #    and (pt[2] < map.building_height[index_n, index_e] + buf):
            if (pt[0] > dn - buf and pt[0] < dn + w + buf) \
               and (pt[1] > de - buf and pt[1] < de + w + buf):
               return True
        return False

    def pointsAlongPath(self, start_node, end_node, Del):
        vec = end_node[0:3] - start_node[0:3]
        v = np.linalg.norm(vec)
        num_pts = np.ceil(v/Del)
        vec = vec/v

        points = np.empty((1,3))
        points[0] = start_node[0:3]
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
            temp = np.array([v_plus[0], v_plus[1], v_plus[2], cost, index, flag])
            tree = np.vstack((tree, temp))
            if not self.collision(v_plus, end_node, map):
                flag = 1
                cost +=  np.linalg.norm(v_plus[0:3])
                temp = np.array([end_node[0], end_node[1], end_node[2], cost, tree.shape[0]-1, flag])
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
                smooth.append(last_node)
                i = j
            j += 1
        smooth.append(path[-1])
        smooth = np.array(smooth)
        self.waypoints.ned = smooth[:, 0:3].T
