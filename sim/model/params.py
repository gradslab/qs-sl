"""Parameters of the slung payload system, the path and the controller, as in Section VI of the paper.
SI units. The inertial frame is north-east-down, so heights are negative."""
import numpy as np

# slung payload system
M_Q = 5.0                      # quadrotor mass [kg]
M_P = 0.2                      # payload mass [kg]
L = 3.0                        # pendulum length [m]
G = 9.81                       # gravity [m/s^2]
J = np.diag([0.03, 0.03, 0.05])  # quadrotor inertia [kg m^2]
ELL = 0.2                      # rotor-to-center-of-mass distance [m]
C_D = 0.01                     # drag-to-thrust ratio of the propellers
C1 = ELL / 2.0                 # tau_1 = C1 u_t - C2 tau_3 under the failure
C2 = ELL / (2.0 * C_D)
K_OM = 3.0                     # damping gain k_Omega [1/s], the same on every axis

# path: horizontal circle of radius D_STAR at height R_STAR, traversed at V_D
D_STAR = 5.0                   # [m]
R_STAR = -3.0                  # [m], 3 m above the ground in north-east-down
V_D = 0.8                      # [rad/s] of the polar angle, 4 m/s along the circle

# poles of the transverse chains of h1 (order 6) and h2 (order 2) and of the speed chain of h3 (order 5)
POLES_1 = [-1.0, -1.5, -2.0, -2.5, -3.0, -3.5]
POLES_2 = [-1.0, -1.2]
POLES_3 = [-1.0, -1.5, -2.0, -2.5, -3.0]


def chain_gains(poles):
    """gain row K with nu = -K z for a Brunovsky chain, so that the characteristic polynomial is prod (s - pole)"""
    return np.poly(poles)[1:][::-1]


K1 = chain_gains(POLES_1)      # [78.75, 250.875, 319, 208.125, 73.75, 13.5]
K2 = chain_gains(POLES_2)      # [1.2, 2.2]
K3 = chain_gains(POLES_3)      # [22.5, 65.25, 72.5, 38.75, 10]

# initial condition of the simulation
P0 = np.array([-2.0, 8.0, -11.0])                 # payload position [m]
ALPHA0, BETA0 = np.radians(-30.0), np.radians(20.0)   # pendulum angles
THETA0, PHI0, PSI0 = np.radians(45.0), np.radians(20.0), np.radians(-60.0)  # 2-1-3 Euler angles of the quadrotor
V0 = np.array([-0.2, 0.1, 0.5])                   # payload velocity [m/s]
GAMMA0 = np.array([-0.2, 0.1])                    # pendulum angle rates [rad/s]
OMEGA_B0 = np.array([-0.1, 0.3, 0.2])             # body angular velocity [rad/s]
T_END = 20.0                                      # horizon [s]


def controller_params():
    """the parameters the generated expressions take"""
    return dict(g=G, L=L, m__q=M_Q, m__p=M_P, r1=D_STAR, r2=R_STAR, vd=V_D, k__21=K2[0], k__22=K2[1],
                J__1=J[0, 0], J__2=J[1, 1], J__3=J[2, 2], c__1=C1, c__2=C2, k__Om=K_OM)
