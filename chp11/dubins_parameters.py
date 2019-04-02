# dubins_parameters
#   - Dubins parameters that define path between two configurations
#
# mavsim_matlab
#     - Beard & McLain, PUP, 2012
#     - Update history:
#         3/26/2019 - RWB

import numpy as np
import sys
sys.path.append('..')


class dubins_parameters:
    def __init__(self):
        self.p_s = np.inf*np.ones((3,1))  # the start position in re^3
        self.chi_s = np.inf  # the start course angle
        self.p_e = np.inf*np.ones((3,1))  # the end position in re^3
        self.chi_e = np.inf  # the end course angle
        self.radius = np.inf  # turn radius
        self.length = np.inf  # length of the Dubins path
        self.center_s = np.inf*np.ones((3,1))  # center of the start circle
        self.dir_s = np.inf  # direction of the start circle
        self.center_e = np.inf*np.ones((3,1))  # center of the end circle
        self.dir_e = np.inf  # direction of the end circle
        self.r1 = np.inf*np.ones((3,1))  # vector in re^3 defining half plane H1
        self.r2 = np.inf*np.ones((3,1))  # vector in re^3 defining position of half plane H2
        self.r3 = np.inf*np.ones((3,1))  # vector in re^3 defining position of half plane H3
        self.n1 = np.inf*np.ones((3,1))  # unit vector in re^3 along straight line path
        self.n3 = np.inf*np.ones((3,1))  # unit vector defining direction of half plane H3

    def update(self, ps, chis, pe, chie, R):
        ell = np.linalg.norm(ps[0:2] - pe[0:2])
        if ell < 2 * R:
            print('Error in Dubins Parameters: The distance between nodes must be larger than 2R.')
        else:
            crs = ps + R * rotz(np.pi/2) * np.array([[np.cos(chis), np.sin(chis), 0]]).T
            cls = ps + R * rotz(-np.pi/2) * np.array([[np.cos(chis), np.sin(chis), 0]]).T
            cre = pe + R * rotz(np.pi/2) * np.array([[np.cos(chie), np.sin(chie), 0]]).T
            cle = pe + R * rotz(-np.pi/2) * np.array([[np.cos(chie), np.sin(chie), 0]]).T


            L_rsr = self.calcL_rsr(R, chis, chie, ps, pe)
            L_rsl = self.calcL_rsl(R, chis, chie, ps, pe)
            L_lsr = self.calcL_lsr(R, chis, chie, ps, pe)
            L_lsl = self.calcL_lsl(R, chis, chie, ps, pe)

            L = [L_rsr, L_rsl, L_lsr, L_lsl]
            index = np.argmin(L) #determine which path is shortest

            e1 = np.array([[1, 0, 0]]).T
            z3 = pe
            q3 = rotz(chie) @ e1
            l = np.inf
            if index == 0:
                cs = crs
                ce = cre
                dir_s = 1
                dir_e = 1
                q1 = (ce - cs)/np.linalg.norm(ce - cs)
                z1 = cs + R * rotz(-np.pi/2) @ q1
                z2 = ce + R * rotz(-np.pi/2) @ q1
            elif index == 1:
                cs = crs
                ce = cre
                dir_s = 1
                dir_e = -1
                l = np.linalg.norm(ce - cs)
                var_theta = self.calc_varTheta(ps, pe) #not sure this is the correct calculation
                ang2 = var_theta - np.pi/2 + np.arcsin(2 * R/l)
                q1 = rotz(ang2 + np.pi/2) @ e1
                z1 = cs + R * rotz(ang2) @ e1
                z2 = ce + R * rotz(ang2 + np.pi) @ e1
            elif index == 2:
                cs = cls
                ce = cre
                dir_s = -1
                dir_e = 1
                l = np.linalg.norm(ce - cs) # not sure if this is the correct calculation
                var_theta = self.calc_varTheta(ps, pe)
                ang2 = np.arccos(2 * R /l)
                q1 = rotz(var_theta + ang2 - np.pi/2) @ e1
                z1 = cs + R * rotz(var_theta + ang2) @ e1
                z2 = ce + R * rotz(var_theta + ang2 - np.pi) @ e1
            else:
                cs = cls
                ce = cle
                dir_s = -1
                dir_e = -1
                q1 = (ce - cs)/np.linalg.norm(ce - cs)
                z1 = cs + R * rotz(np.pi/2) @ q1
                z2 = ce + R * rotz(np.pi/2) @ q1

            self.p_s = ps
            self.chi_s = chis
            self.p_e = pe
            self.chi_e = chie
            self.radius = R
            self.length = l
            self.center_s = cs
            self.dir_s = dir_s
            self.center_e = ce
            self.dir_e = dir_e
            self.r1 = z1
            self.n1 = q1
            self.r2 = z2
            self.r3 = z3
            self.n3 = q3

        def calcL_rsr(self, R, chis, chie, ps, pe):
            crs = ps + R * rotz(np.pi/2) * np.array([[np.cos(chis), np.sin(chis), 0]]).T
            cre = ps + R * rotz(np.pi/2) * np.array([[np.cos(chie), np.sin(chie), 0]]).T
            l1 = np.linalg.norm(crs-cre)

            e1 = np.array([[1, 0, 0]]).T
            var_theta1 = mod(chis - np.pi/2)
            var_theta2 = self.calc_varTheta(ps, pe)
            var_theta3 = mod(var_theta2 - np.pi/2)
            l2 = R * mod(2 * np.pi + var_theta3 - var_theta1)

            var_theta4 = mod(chie - np.pi/2)
            l3 = R * mod(2 * np.pi + var_theta4 - var_theta3)

            return l1 + l2 + l3

        def calcL_rsl(self, R, chis, chie, ps, pe):
            p = pe - ps
            l = np.linalg.norm(p)

            l1 = np.sqrt(l**2 + 4 * R**2)

            e1 = np.array([[1, 0, 0]]).T
            var_theta = self.calc_varTheta(ps, pe)
            theta2 = var_theta - np.pi/2.0 + np.arcsin(2*R/l)
            ang1 = mod(chis - np.pi/2.0)
            l2 = R * mod(2 * np.pi + mod(var_theta - theta2) - ang1)

            ang2 = mod(chie + np.pi/2.0)
            l3 = R * mod(2 * np.pi + mod(theta2 + np.pi) - ang2)

            return l1 + l2 + l3

        def calcL_lsr(self, R, chis, chie, ps, pe):
            p = pe - ps
            l = np.linalg.norm(p)

            l1 = np.sqrt(l**2 + 4 * R**2)

            e1 = np.array([[1, 0, 0]]).T
            var_theta = self.calc_varTheta(ps, pe)
            theta2 = np.arccos(2*R/l)
            ang1 = mod(chis + np.pi/2.0)
            l2 = R * mod(2 * np.pi + ang1 - mod(var_theta + theta2))

            ang2 = mod(chie - np.pi/2.0)
            l3 = R * mod(2 * np.pi + ang2 - mod(var_theta + theta2 - np.pi))

            return l1 + l2 + l3

        def calcL_lsl(self, R, chis, chie, ps, pe):
            p = pe - ps
            cls = p + R * rotz(-np.pi/2) * np.array([[np.cos(chis), np.sin(chis), 0]]).T
            cle = p + R * rotz(-np.pi/2) * np.array([[np.cos(chie), np.sin(chie), 0]]).T

            l1 = np.linalg.norm(cls - cle)

            e1 = np.array([[1, 0, 0]]).T
            ang1 = mod(chis + np.pi/2)
            var_theta = self.calc_varTheta(ps, pe)
            l2 = R * mod(2 * np.pi + ang1 - mod(var_theta + np.pi/2))

            ang2 = mod(chie + np.pi/2)
            l3 = mod(2 * np.pi + mod(var_theta + np.pi/2) - ang2)

            return l1 + l2 + l3

        def calc_varTheta(self, ps, pe):
            e1 = np.array([[1, 0, 0]]).T
            p = pe - ps
            var_theta = mod(np.arccos(np.dot(e1, p)/(np.linalg.norm(pe) * np.linalg.norm(ps)))) #check this if not working

            return var_theta

def rotz(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])

def mod(x):
    return x % (2*np.pi)
