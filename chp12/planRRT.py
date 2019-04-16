import numpy as np
from messages.msg_waypoints import msg_waypoints


class planRRT():
    def __init__(self, map):
        self.waypoints = msg_waypoints()
        self.segmentLength = 300 # standard length of path segments

    def planPath(self, wpp_start, wpp_end, map):
        # desired down position is down position of end node
        pd = wpp_end.item(2)

        # specify start and end nodes from wpp_start and wpp_end
        # format: N, E, D, cost, parentIndex, connectsToGoalFlag,
        start_node = np.array([wpp_start.item(0), wpp_start.item(1), pd, 0, 0, 0])
        end_node = np.array([wpp_end.item(0), wpp_end.item(1), pd, 0, 0, 0])

        # establish tree starting with the start node
        tree = start_node

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
        return self.smoothPath(path, map)

    def generateRandomNode(self, map, pd, chi):
        min = 0
        max = map.city_width

        p = np.array([np.random.uniform(min, max), np.random.uniform(min, max), pd, chi])  # N, E, D, heading
        return p

    def findClosestNode(self, tree, pt):
        n = pt.item(0)
        e = pt.item(1)

        dist = (n - tree[:,0])**2 + (e - tree[:,1])**2
        index = np.argmin(dist)

        return tree[index,:], index

    def planSegment(self, v_star, p):
        v = v_star[0:3] - p[0:3]
        v = v / np.linalg.norm(v)
        v_plus = v_star[0:3] + v * self.segmentLength

        return np.array([v_plus.item(0), v_plus.item(1), p.item(2), p.item(3)])

    def collision(self, start_node, end_node, map):
        delta = self.segmentLength/10
        pts = self.pointsAlongPath(start_node, end_node, delta)

        for i in range(pts.shape[0]):
            pt = pts[i,:]
            #find the closes building in NE plane
            dst_n = (pt[0] - map.building_north)**2
            dst_e = (pt[1] - map.building_east)**2
            index_n = np.argmin(dst_n)
            index_e = np.argmin(dst_e)

            buf = 5
            if (pt[0] > dst_n[index_n] - buf and pt[0] < dst_n[index_n] + map.building_width + buf) \
               and (pt[1] > dst_e[index_e] - buf and pt[1] < dst_e[index_e] + map.building_width + buf):
               return True
        return False


    def pointsAlongPath(self, start_node, end_node, Del):
        vec = end_node[0:3] - start_node[0:3]
        vec = vec/np.linalg.norm(vec)

        points = start_node[0:3]
        for i in range(10):
            temp = points[-1,:] + vec * Del
            points = np.hstack((points, temp))

        return points

    def downAtNE(self, map, n, e):
        debug = 0

    def extendTree(self, tree, end_node, segmentLength, map, pd):
        pt = self.generateRandomNode(map, pd, 0) #figure out what chi is later
        v_star, index = self.findClosestNode(tree, pt)
        v_plus = self.planSegment(v_star, p)
        flag = 0
        if !self.collision(v_star, v_plus, map):
            #append to tree
            cost = self.segmentLength + tree[index, 3]
            temp = np.array(v_plus[0], v_plus[1], v_plus[2], cost, index, flag)
            np.hstack((tree, temp))
        if !self.collision(v_plus, end_node, map):
            flag = 1
            cost +=  np.linalg.norm(v_plus[0:3])
            temp = np.array(end_node[0], end_node[1], end_node[2], cost, tree.shape[0]-1, flag)
            np.hstack((tree, temp))

        return flag

    def findMinimumPath(self, tree, end_node):
        debug = 0

    def smoothPath(self, path, map):
        debug = 0
