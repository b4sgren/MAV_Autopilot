"""
compute_ss_model
    - Chapter 5 assignment for Beard & McLain, PUP, 2012
    - Update history:
        2/4/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
from scipy.optimize import minimize
from tools.tools import Euler2Quaternion, Quaternion2Euler
from control import TransferFunction as TF
import parameters.aerosonde_parameters as MAV
from parameters.sim_params import ts_sim as Ts
from mav_dynamics import mav_dynamics as Dynamics

from mav_dynamics import mav_dynamics as Dynamics
from trim import compute_trim

def compute_tf_model(mav, trim_state, trim_input):
    rho = MAV.rho
    S = MAV.S_wing
    Va = mav._Va
    b = MAV.b
    beta = mav._beta
    alpha = mav._alpha
    c = MAV.c
    Jy = MAV.Jy

    # Transfer Function Models
    b2Va = b/(2 * Va)
    a_phi_1 = 0.5 * rho * (Va**2) * S * b * MAV.C_p_p * b2Va
    a_phi_2 = 0.5 * rho * (Va**2) * S * b * MAV.C_p_delta_a
    T_phi_delta_a = TF(np.array([a_phi_2]), np.array([1, a_phi_1, 0]))

    T_chi_phi = TF(np.array([MAV.gravity/Va]), np.array([1, 0]))  #Va should be Vg

    betadr = (-rho * Va * S) / (2. * MAV.mass * np.cos(beta))
    a_beta1 = betadr * MAV.C_Y_beta
    a_beta2 = betadr * MAV.C_Y_delta_r
    T_beta_delta_r = TF(np.array([a_beta2]), np.array([1, a_beta1]))

    thetade = (rho * (Va**2) * c * S) / (2. * Jy)
    a_theta1 = -thetade * MAV.C_m_q * (c / (2. * Va))
    a_theta2 = - thetade * MAV.C_m_alpha
    a_theta3 = thetade * MAV.C_m_delta_e
    T_theta_delta_e = TF(np.array([a_theta3]), np.array([1, a_theta1, a_theta2]))

    T_h_theta = TF(np.array([Va]), np.array([1, 0]))

    _, theta, _ = Quaternion2Euler(trim_state[6:10])
    T_h_Va = TF(np.array([theta]), np.array([1, 0]))

    C_vals = (MAV.C_D_0 + MAV.C_D_alpha * alpha + MAV.C_D_delta_e * trim_input.item(0))
    a_V1 = ((rho * Va * S * C_vals) - dT_dVa(mav, Va, trim_input.item(1))) / MAV.mass
    a_V2 = dT_ddelta_t(mav, Va, trim_input.item(1)) / MAV.mass
    a_V3 = MAV.gravity * np.cos(theta - alpha)
    T_Va_delta_t = TF(np.array([a_V2]), np.array([1, a_V1]))
    T_Va_theta = TF(np.array([-a_V3]), np.array([1, a_V1]))

    return [T_phi_delta_a, T_chi_phi, T_beta_delta_r, T_theta_delta_e, T_h_theta, T_h_Va, T_Va_delta_t, T_Va_theta]

def compute_ss_model(mav, trim_state, trim_input):
    x_euler = euler_state(trim_state)
    A = df_dx(mav, x_euler, trim_input)
    B = df_du(mav, x_euler, trim_input)

    #indexing each state
    h = 2
    u = 3
    v = 4
    w = 5
    phi = 6
    theta = 7
    psi = 8
    p = 9
    q = 10
    r = 11

    #indexing each input
    de = 0
    dt = 1
    da = 2
    dr = 3

    A_lat = np.array([[A[v,v], A[v,p], A[v,r], A[v,phi], A[v,psi]],
                      [A[p,v], A[p,p], A[p,r], A[p,phi], A[p,psi]],
                      [A[r,v], A[r,p], A[r,r], A[r,phi], A[r,psi]],
                      [A[phi,v], A[phi,p], A[phi,r], A[phi,phi], A[phi,psi]],
                      [A[psi,v], A[psi,p], A[psi,r], A[psi,phi], A[psi,psi]]])
    B_lat = np.array([[B[v,da], B[v,dr]],
                      [B[p,da], B[p,dr]],
                      [B[r,da], B[r,dr]],
                      [B[phi,da], B[phi,dr]],
                      [B[psi,da], B[psi,dr]]])

    A_lon = np.array([[A[u,u], A[u,w], A[u,q], A[u,theta], A[u,h]],
                      [A[w,u], A[w,w], A[w,q], A[w,theta], A[w,h]],
                      [A[q,u], A[q,w], A[q,q], A[q,theta], A[q,h]],
                      [A[theta,u], A[theta,w], A[theta,q], A[theta,theta], A[theta,h]],
                      [A[h,u], A[h,w], A[h,q], A[h,theta], A[h,h]]])
    B_lon = np.array([[B[u,de], B[u,dt]],
                      [B[w,de], B[w,dt]],
                      [B[q,de], B[q,dt]],
                      [B[theta,de], B[theta,dt]],
                      [B[h,de], B[h,dt]]])

    return A_lon, B_lon, A_lat, B_lat

def euler_state(x_quat):
    # convert state x with attitude represented by quaternion
    # to x_euler with attitude represented by Euler angles
    phi, theta, psi = Quaternion2Euler(x_quat[6:10])

    x_euler = np.zeros((12, 1))
    x_euler[:6] = x_quat[:6]
    x_euler[6:9] = np.array([[phi, theta, psi]]).T
    x_euler[9:] = x_quat[10:]

    return x_euler

def quaternion_state(x_euler):
    # convert state x_euler with attitude represented by Euler angles
    # to x_quat with attitude represented by quaternions
    phi = x_euler.item(6)
    theta = x_euler.item(7)
    psi = x_euler.item(8)
    e = Euler2Quaternion(phi, theta, psi)

    x_quat = np.zeros((13, 1))
    x_quat[:6] = x_euler[:6]
    x_quat[6:10] = e
    x_quat[10:] = x_euler[9:]

    return x_quat

def f_euler(mav, x_euler, input):
    # return 12x1 dynamics (as if state were Euler state)
    return f_euler_

def df_dx(mav, x_euler, input):
    # take partial of f_euler with respect to x_euler
    dT = dT_dxquat(x_euler)
    forces_moments = mav.calcForcesAndMoments(input)
    eps = 0.01
    A = np.zeros((12, 12))
    A_quat = np.zeros((13, 12))
    fxu = mav._derivatives(quaternion_state(x_euler), forces_moments)

    for i in range(12):
        x_eps = np.copy(x_euler)
        x_eps[i][0] += eps
        f_eps = mav._derivatives(quaternion_state(x_eps), forces_moments)
        dfdx = (f_eps - fxu) / eps
        A_quat[:,i] = dfdx[:,0]

    A = dT @ A_quat
    return A

def df_du(mav, x_euler, delta):
    # take partial of f_euler with respect to delta
    dT = dT_dxquat(x_euler)
    forces_moments = mav.calcForcesAndMoments(delta)
    eps = 0.005
    B = np.zeros((12, 4))
    B_quat = np.zeros((13, 4))
    fxu = mav._derivatives(quaternion_state(x_euler), forces_moments)

    for i in range(4):
        d_eps = np.copy(delta)
        d_eps[i][0] +=  eps
        forces_moments = mav.calcForcesAndMoments(d_eps)
        f_eps = mav._derivatives(quaternion_state(x_euler), forces_moments)
        dfdu = (f_eps - fxu) / eps
        B_quat[:, i] = dfdu[:,0]

    B = dT @ B_quat
    return B

def dT_dVa(mav, Va, delta_t):
    # returns the derivative of motor thrust with respect to Va
    epsilon = 0.01
    Tp1, _ = mav.calcThrustForceAndMoment(delta_t, Va - epsilon)
    Tp2, _ = mav.calcThrustForceAndMoment(delta_t, Va + epsilon)

    dThrust = (Tp2 - Tp1) / (2. * epsilon)
    return dThrust

def dT_ddelta_t(mav, Va, delta_t):
    # returns the derivative of motor thrust with respect to delta_t
    epsilon = 0.001
    Tp1, _ = mav.calcThrustForceAndMoment(delta_t - epsilon, Va)
    Tp2, _ = mav.calcThrustForceAndMoment(delta_t + epsilon, Va)

    dThrust = (Tp2 - Tp1) / (2. * epsilon)
    return dThrust

def dT_dxquat(x_euler):
    dT = np.zeros((12, 13))
    dT[0:6, 0:6] = np.eye(6)
    dT[9:, 10:] = np.eye(3)

    dTheta = dTheta_dq(quaternion_state(x_euler))

    dT[6:9, 6:10] = dTheta
    return dT

def dTheta_dq(x):
    q = x[6:10]
    phi, theta, psi = Quaternion2Euler(q)
    eps = 0.01

    phi1, th1, psi1 = Quaternion2Euler(q + np.array([[eps, 0, 0, 0]]).T)
    phi2, th2, psi2 = Quaternion2Euler(q + np.array([[0, eps, 0, 0]]).T)
    phi3, th3, psi3 = Quaternion2Euler(q + np.array([[0, 0, eps, 0]]).T)
    phi4, th4, psi4 = Quaternion2Euler(q + np.array([[0, 0, 0, eps]]).T)

    dphi1 = (phi1 - phi)/eps
    dphi2 = (phi2 - phi)/eps
    dphi3 = (phi3 - phi)/eps
    dphi4 = (phi4 - phi)/eps
    dth1 = (th1 - theta)/eps
    dth2 = (th2 - theta)/eps
    dth3 = (th3 - theta)/eps
    dth4 = (th4 - theta)/eps
    dpsi1 = (psi1 - psi)/eps
    dpsi2 = (psi2 - psi)/eps
    dpsi3 = (psi3 - psi)/eps
    dpsi4 = (psi4 - psi)/eps

    dTheta = np.array([[dphi1, dphi2, dphi3, dphi4],
                       [dth1, dth2, dth3, dth4],
                       [dpsi1, dpsi2, dpsi3, dpsi4]])
    return dTheta

if __name__ == "__main__":
    mav = Dynamics(0.02)
    Va = 25.0
    gamma = 0.0
    mav._Va = Va

    trim_state, trim_input = compute_trim(mav, Va, gamma)
    tf_list = compute_tf_model(mav, trim_state, trim_input)

    print('T_phi_delta_a\n', tf_list[0])
    print('T_chi_phi\n', tf_list[1])
    print('T_beta_delta_r\n', tf_list[2])
    print('T_theta_delta_e\n', tf_list[3])
    print('T_h_theta\n', tf_list[4])
    print('T_h_Va\n', tf_list[5])
    print('T_Va_delta_t\n',tf_list[6])
    print('T_Va_theta\n', tf_list[7])

    x_e = np.array([[10., 10., 0., 1., 2., 3., 0., np.pi/6, 0., 1., 2., 3.]]).T
    x_q = quaternion_state(x_e)
    # print('xq:\n', x_q)
    x_e = euler_state(x_q)
    # print('xe:\n', x_e)

    x_e = np.array([[1, 1, -10, 25, 0, 0, 0, 0, 0, 0, 0, 0]]).T
    dT = dT_dxquat(x_e)
    # print(dT)
    # print(dT.shape)

    A = df_dx(mav, euler_state(trim_state), trim_input)
    # print('A:\n', A)
    B = df_du(mav, euler_state(trim_state), trim_input)
    # print('B:\n', B)

    A_lon, B_lon, A_lat, B_lat = compute_ss_model(mav, trim_state, trim_input)
    # print('A_lon:\n', A_lon)
    # print('B_lon:\n', B_lon)
    # print('A_lat:\n', A_lat)
    # print('B_lat:\n', B_lat)
    #
    # eig_lon, _ = np.linalg.eig(A_lon)
    # eig_lat, _ = np.linalg.eig(A_lat)
    # print('Eig A_lon:\n', eig_lon)
    # print('Eig A_lat:\n', eig_lat)
