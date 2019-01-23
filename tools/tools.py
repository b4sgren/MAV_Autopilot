import numpy as np
import math

def Quaternion2Euler(quat):
    e0 = quat[0]
    ex = quat[1]
    ey = quat[2]
    ez = quat[3]

    euler = np.empty(3)
    
    euler[0] = math.atan2(2 * (e0*ex + ey*ez), e0**2 + ez**2 - ex**2 - ey**2) # phi
    euler[1] = math.asin(2 * (e0*ey - ex*ez)) # theta
    euler[2] = math.atan2(2*(e0*ez + ex*ey), e0**2 + ex**2 - ey**2 - ez**2) # psi

    return euler

def Euler2Quaternion(phi, theta, psi):
    c_phi2 = np.cos(phi/2.0)
    s_phi2 = np.sin(phi/2.0)
    c_theta2 = np.cos(theta/2.0)
    s_theta2 = np.sin(theta/2.0)
    c_psi2 = np.cos(psi/2.0)
    s_psi2 = np.sin(psi/2.0)

    quat = np.empty(4)
    quat[0] = c_psi2 * c_theta2 * c_phi2 + s_psi2 * s_theta2 * s_phi2  # e0
    quat[1] = c_psi2 * c_theta2 * s_phi2 - s_psi2 * s_theta2 * c_phi2  # ex
    quat[2] = c_psi2 * s_theta2 * c_phi2 + s_psi2 * c_theta2 * s_phi2  # ey
    quat[3] = s_psi2 * c_theta2 * c_phi2 - c_psi2 * s_theta2 * s_phi2  # ez

    return quat

if __name__ == "__main__":
    phi = np.pi/2.0
    theta = np.pi/6.0
    psi = 0.0

    euler = np.array([phi, theta, psi])
    quat = Euler2Quaternion(euler)
    print(Quaternion2Euler(quat))
