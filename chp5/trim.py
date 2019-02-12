"""
compute_trim
    - Chapter 5 assignment for Beard & McLain, PUP, 2012
    - Update history:
        2/5/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
from scipy.optimize import minimize
from tools.tools import Euler2Quaternion
from mav_dynamics import mav_dynamics as Dynamics

def compute_trim(mav, Va, gamma):
    # define initial state and input
    e = Euler2Quaternion(0, gamma, 0)
    state0 = np.array([[0., 0., 0., Va, 0., 0., e.item(0), e.item(1), e.item(2), e.item(3), 0., 0., 0.]]).T  # I need another 0 for the extra quaternion state
    delta0 = np.array([[0., 0.5, 0., 0.]]).T
    x0 = np.concatenate((state0, delta0), axis=0)
    # define equality constraints
    cons = ({'type': 'eq',
             'fun': lambda x: np.array([
                                x[3]**2 + x[4]**2 + x[5]**2 - Va**2,  # magnitude of velocity vector is Va
                                x[4],  # v=0, force side velocity to be zero
                                x[6]**2 + x[7]**2 + x[8]**2 + x[9]**2 - 1.,  # force quaternion to be unit length
                                x[7], # e1=0  - forcing e1=e3=0 ensures zero roll and zero yaw in trim
                                x[9], # e3=0
                                x[10], # p=0  - angular rates should all be zero
                                x[11], # q=0
                                x[12], # r=0
                                ]),
             'jac': lambda x: np.array([
                                [0., 0., 0., 2*x[3], 2*x[4], 2*x[5], 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 2*x[6], 2*x[7], 2*x[8], 2*x[9], 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.],
                                ])
             })
    # solve the minimization problem to find the trim states and inputs
    res = minimize(trim_objective, x0, method='SLSQP', args = (mav, Va, gamma),
                   constraints=cons, options={'ftol': 1e-10, 'disp': True})
    # extract trim state and input and return
    trim_state = np.array([res.x[0:13]]).T
    trim_input = np.array([res.x[13:17]]).T
    return trim_state, trim_input

# objective function to be minimized
def trim_objective(x, mav, Va, gamma):
    # Isn't psi_dot non zero? How to represent this in quaternion? Same with rdot
    xdot_star = np.array([[0., 0., Va * np.sin(gamma), 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]]).T
    delta = np.array([[0., 0.5, 0., 0.]]).T
    forces_moments = mav.calcForcesAndMoments(delta)
    f = mav._derivatives(mav._state, forces_moments)

    error = xdot_star - f
    J = error.T @ error
    return J

if __name__ == "__main__":
    mav = Dynamics(.02)
    Va = 10.0
    gamma = 0.2

    trim_state, trim_input = compute_trim(mav, Va, gamma) # Why don't I need R??

    print("State: ", trim_state)
    print("Inputs: ", trim_input)
