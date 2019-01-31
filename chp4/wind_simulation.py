"""
Class to determine wind velocity at any given moment,
calculates a steady wind speed and uses a stochastic
process to represent wind gusts. (Follows section 4.4 in uav book)
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.aerosonde_parameters as MAV

class wind_simulation:
    def __init__(self, Ts):
        # steady state wind defined in the inertial frame
        self._steady_state = np.array([[0., 0., 0.]]).T
        # self.steady_state = np.array([[3., 1., 0.]]).T

        #   Dryden gust model parameters (pg 56 UAV book)
        # HACK:  Setting Va to a constant value is a hack.  We set a nominal airspeed for the gust model.
        # Could pass current Va into the gust function and recalculate A and B matrices.
        Va = 17 
        self._A = np.eye(3) # A and B are wrong but I'm not really sure what they should be
        self._B = np.eye(3) # Not sure how to fix the hack or what it means
        self._C = np.eye(3)
        self._gust_state = np.zeros((3, 1))
        self._Ts = Ts

    def update(self):
        # returns a six vector.
        #   The first three elements are the steady state wind in the inertial frame
        #   The second three elements are the gust in the body frame
        return np.concatenate(( self._steady_state, self._gust() ))

    def _gust(self):
        # calculate wind gust using Dryden model.  Gust is defined in the body frame
        w = np.random.randn(3, 1)  # zero mean unit variance Gaussian (white noise)
        # propagate Dryden model (Euler method): x[k+1] = x[k] + Ts*( A x[k] + B w[k] )
        self._gust_state += self._Ts * (self._A @ self._gust_state + self._B @ w)
        # output the current gust: y[k] = C x[k]
        return np.zeros((3, 1)) # self._C @ self._gust_state

