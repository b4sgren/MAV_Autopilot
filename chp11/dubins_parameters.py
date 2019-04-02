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
            c_xs = np.cos(chis)
            s_xs = np.sin(chis)
            c_xe = np.cos(chie)
            s_xe = np.sin(chie)
            crs = ps + R * rotz(np.pi/2) @ np.array([[c_xs, s_xs, 0]]).T
            cls = ps + R * rotz(-np.pi/2) @ np.array([[c_xs, s_xs, 0]]).T
            cre = pe + R * rotz(np.pi/2) @ np.array([[c_xe, s_xe, 0]]).T
            cle = pe + R * rotz(-np.pi/2) @ np.array([[c_xe, s_xe, 0]]).T

            theta = np.arctan2(ps.item(1)-pe.item(1),ps.item(0)-pe.item(0))
            theta2 = theta - np.arccos(2*R/ell)
            pi = np.pi
            sqrt = np.sqrt(ell**2 - 4*R**2)

            # compute L1,L2,L3,L4
            L_rsr = np.linalg.norm(crs-cre)+R*mod(2*pi+mod(theta-pi/2)-\
                    mod(chis-pi/2))+R*mod(2*pi+mod(chie-pi/2)-mod(theta-pi/2))
            L_rsl = sqrt*mod(2*pi+mod(theta-theta2)-mod(chis-pi/2))+\
                    R*mod(2*pi+mod(theta2+pi)-mod(chie+pi/2))
            L_lsr = sqrt+R*mod(2*pi+mod(chis+pi/2)-mod(theta-theta2))+R*mod(\
                    2*pi+mod(chie-pi/2)-mod(theta+theta2-pi))
            L_lsl = np.linalg.norm(cls-cle)+R*mod(2*pi+mod(chis+pi/2)-\
                    mod(theta+pi/2))+R*mod(2*pi+mod(theta+pi/2)-mod(chie-pi/2))
            L = [L_rsr, L_rsl, L_lsr, L_lsl]
            index = np.argmin(L) #determine which path is shortest

            e1 = np.array([[1, 0, 0]]).T
            z3 = pe
            q3 = rotz(chie) @ e1
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
                ce = cle
                dir_s = 1
                dir_e = -1
                diff = ce - cs
                ell = np.linalg.norm(diff)
                var_theta = np.arctan2(diff.item(1), diff.item(0)) #switches variables in mats code
                ang2 = var_theta - np.pi/2 + np.arcsin(2 * R/ell)
                q1 = rotz(ang2 + np.pi/2) @ e1
                z1 = cs + R * rotz(ang2) @ e1
                z2 = ce + R * rotz(ang2 + np.pi) @ e1
            elif index == 2:
                cs = cls
                ce = cre
                dir_s = -1
                dir_e = 1
                diff = ce - cs
                ell = np.linalg.norm(diff)
                var_theta = np.arctan2(diff.item(1), diff.item(0)) # switches order in mats code
                ang2 = np.arccos(2 * R /ell)
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
            self.length = ell
            self.center_s = cs
            self.dir_s = dir_s
            self.center_e = ce
            self.dir_e = dir_e
            self.r1 = z1
            self.n1 = q1
            self.r2 = z2
            self.r3 = z3
            self.n3 = q3

def rotz(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])

def mod(x):
    return x % (2*np.pi)
