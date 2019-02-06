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
        # self._steady_state = np.array([[3., 1., 0.]]).T

        #Dryden Parameters:
        self.Lu = 200.0
        self.Lv = self.Lu
        self.Lw = 50.0
        self.sigma_u = 2.12
        self.sigma_v = self.sigma_u
        self.sigma_w = 1.4

        #   Dryden gust model parameters (pg 56 UAV book)
        # HACK:  Setting Va to a constant value is a hack.  We set a nominal airspeed for the gust model.
        # Could pass current Va into the gust function and recalculate A and C matrices.
        Va = MAV.u0
        self._A = np.array([[-Va/self.Lu, 0, 0, 0, 0],
                            [0, -2*(Va/self.Lv), -(Va/self.Lv)**2, 0, 0],
                            [0, 1, 0, 0, 0],
                            [0, 0, 0, -2*(Va/self.Lw), -(Va/self.Lw)**2],
                            [0, 0, 0, 1, 0]])
        self._B = np.array([[1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 0],
                            [0, 0, 1],
                            [0, 0, 0]])
        self._C = np.array([[self.sigma_u * np.sqrt((2*Va)/self.Lu), 0, 0, 0, 0],
                            [0, self.sigma_v * np.sqrt((3*Va)/self.Lv), np.sqrt((Va/self.Lv)**3), 0, 0],
                            [0, 0, 0, self.sigma_w * np.sqrt((3*Va)/self.Lv), np.sqrt((Va/self.Lw)**3)]])
        self._gust_state = np.zeros((5, 1))
        self._Ts = Ts

    def update(self, Va):
        # returns a six vector.
        #   The first three elements are the steady state wind in the inertial frame
        #   The second three elements are the gust in the body frame
        return np.concatenate(( self._steady_state, self._gust(Va) ))
        # return np.concatenate(( self._steady_state, np.zeros((3, 1)) ))

    def _gust(self, Va):
        self._A = np.array([[-Va/self.Lu, 0, 0, 0, 0],
                            [0, -2*(Va/self.Lv), -(Va/self.Lv)**2, 0, 0],
                            [0, 1, 0, 0, 0],
                            [0, 0, 0, -2*(Va/self.Lw), -(Va/self.Lw)**2],
                            [0, 0, 0, 1, 0]])
        self._C = np.array([[self.sigma_u * np.sqrt((2*Va)/self.Lu), 0, 0, 0, 0],
                            [0, self.sigma_v * np.sqrt((3*Va)/self.Lv), np.sqrt((Va/self.Lv)**3), 0, 0],
                            [0, 0, 0, self.sigma_w * np.sqrt((3*Va)/self.Lv), np.sqrt((Va/self.Lw)**3)]])
        # calculate wind gust using Dryden model.  Gust is defined in the body frame
        w = np.random.randn(3, 1)  # zero mean unit variance Gaussian (white noise)
        # propagate Dryden model (Euler method): x[k+1] = x[k] + Ts*( A x[k] + B w[k] )
        self._gust_state += self._Ts * (self._A @ self._gust_state + self._B @ w)
        # output the current gust: y[k] = C x[k]
        return self._C @ self._gust_state
