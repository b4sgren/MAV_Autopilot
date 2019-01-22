#message for the state. Will be used to pass state between different classes

class StateMsg:
    def __init__(self):
        self.pn = 0.0       #inertial North position
        self.pe = 0.0       #inertial East position
        self.h = 0.0        #inertial altitude. (Inertial down is negative of this)
        self.phi = 0.0      #roll angle
        self.theta = 0.0    #pitch angle
        self.psi = 0.0      #yaw angle
        self.Va = 0.0       # airspeed in m/s
        self.alpha = 0.0    #angle of attack in rad
        self.beta = 0.0     #sideslip angle in radians
        self.p = 0.0        #roll rate in rad/s
        self.q = 0.0        #pitch rate in rad/s
        self.r = 0.0        #yaw rate in rad/s
        self.Vg = 0.0       #groundspeed in m/s
        self.gamma = 0.0    #flgith path angle in rad
        self.chi = 0.0      # course angle in radians
        self.wn = 0.0       #inertial windspeed in north direction m/s
        self.we = 0.0       #inertial windspeed in east direction m/s
        self.bx = 0.0       #gyro bias along roll axis in rad/s
        self.by = 0.0       #gyro bias along pitch axis in rad/s
        self.bz = 0.0       #gyro bias along yaw axis in rad/s
