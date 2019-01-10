#message for the state. Will be used to pass state between different classes

class StateMsg:
    def __init__(self):
        self.pn = 0.0       #inertial North position
        self.pe = 0.0       #inertial East position
        self.h = 0.0        #inertial altitude. (Inertial down is negative of this)
        self.phi = 0.0      #roll angle
        self.theta = 0.0    #pitch angle
        self.psi = 0.0      #yaw angle
