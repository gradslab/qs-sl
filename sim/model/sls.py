"""The slung payload system under single-rotor failure and its quasi-static feedback, in Python.

Coordinate-free model (1) of the paper with the applied torque (2), state
    x = (p, q, R row-major, v, omega, Omega) in R^24,
in the north-east-down frame. The controller is the quasi-static feedback (17), (26): it reads the
state, projects q onto the sphere and R onto SO(3), computes the 2-1-3 Euler angles and the pendulum
angles, evaluates the generated output chains and returns the thrust and the three applied torques.
This mirrors matlab/SLS_quasi.m and matlab/controller.m line by line.
"""
import os, sys, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params as P
from qsf_gen import qsf_terms

E3 = np.array([0.0, 0.0, 1.0])


def hat(w):
    return np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_x(a):
    c, s = np.cos(a), np.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_z(a):
    c, s = np.cos(a), np.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def R_of_euler(theta, phi, psi):
    """2-1-3 Euler angles, R = Ry(theta) Rx(phi) Rz(psi), equation (3a)"""
    return rot_y(theta) @ rot_x(phi) @ rot_z(psi)


def euler_of_R(R):
    """inverse of R_of_euler away from phi = +-pi/2 (MATLAB rotm2eul with the YXZ sequence)"""
    phi = np.arcsin(np.clip(-R[1, 2], -1.0, 1.0))
    theta = np.arctan2(R[0, 2], R[2, 2])
    psi = np.arctan2(R[1, 0], R[1, 1])
    return theta, phi, psi


def q_of_angles(alpha, beta):
    """equation (3b)"""
    return np.array([np.sin(beta), -np.sin(alpha) * np.cos(beta), np.cos(alpha) * np.cos(beta)])


def angles_of_q(q):
    beta = np.arctan2(q[0], np.hypot(q[1], q[2]))
    alpha = np.arctan2(-q[1], q[2])
    return alpha, beta


def W_gamma(alpha, beta):
    """omega = W_gamma (alpha, beta) gamma, equation (5)"""
    ca, sa, cb, s2b = np.cos(alpha), np.sin(alpha), np.cos(beta), np.sin(2 * beta)
    return np.array([[cb ** 2, 0.0], [sa * s2b / 2.0, ca], [-ca * s2b / 2.0, sa]])


def project_so3(R):
    U, _, Vt = np.linalg.svd(R)
    return U @ Vt


def initial_state():
    R0 = R_of_euler(P.THETA0, P.PHI0, P.PSI0)
    q0 = q_of_angles(P.ALPHA0, P.BETA0)
    om0 = W_gamma(P.ALPHA0, P.BETA0) @ P.GAMMA0
    return np.concatenate([P.P0, q0, R0.ravel(), P.V0, om0, P.OMEGA_B0])


def unpack(x):
    p = x[0:3]; q = x[3:6]; R = x[6:15].reshape(3, 3); v = x[15:18]; om = x[18:21]; Om = x[21:24]
    return p, q, R, v, om, Om


def controller(x, cp=None):
    """quasi-static feedback (17), (26) with the damping (2). Returns the thrust, the applied torque
    tau_a = tau - k_Omega J Omega, and the intermediate quantities."""
    cp = cp or P.controller_params()
    p, q, R, v, om, Om = unpack(x)
    q = q / np.linalg.norm(q)
    R = project_so3(R)
    theta, phi, psi = euler_of_R(R)
    alpha, beta = angles_of_q(q)
    W = W_gamma(alpha, beta)
    gamma = np.linalg.solve(W.T @ W, W.T @ om)
    T = qsf_terms(p[0], p[1], p[2], v[0], v[1], v[2], alpha, beta, gamma[0], gamma[1], theta, phi, psi, Om[0], Om[1], Om[2], **cp)
    z1 = np.array([T['h1'], T['h1__dt'], T['h1__ddt'], T['h1__dddt'], T['h1__ddddt'], T['h1__dddddt']])
    z2 = np.array([T['h2'], T['h2__dt']])
    e3 = np.array([T['h3__dt'], T['h3__ddt'], T['h3__dddt'], T['h3__ddddt'], T['h3__dddddt']])
    nu1 = -P.K1 @ z1; nu2 = -P.K2 @ z2; nu3 = -P.K3 @ e3
    ca, cb = np.cos(alpha), np.cos(beta)
    b3 = R[:, 2]
    b3q = float(b3 @ q)
    om2 = (cb ** 2) * gamma[0] ** 2 + gamma[1] ** 2            # ||omega||^2 in the pendulum angles
    Lf2h2 = P.G - P.L * P.M_Q / (P.M_Q + P.M_P) * om2 * ca * cb
    u_t = (P.M_Q + P.M_P) * (Lf2h2 - nu2) / (b3q * ca * cb)   # equation (17)
    sigma = -u_t * (P.G - nu2) / (P.M_Q * P.L * b3q * (ca * cb) ** 2)   # equation (19)
    dh1, dh2, dh3 = np.array(T['dh_1']), np.array(T['dh_2']), np.array(T['dh_3'])
    left = np.vstack([np.cross(dh1, dh2), np.cross(dh3, dh2)])
    right = np.column_stack([np.cross(np.cross(R[:, 1], q), q) / P.J[1, 1], -P.C2 * np.cross(np.cross(R[:, 0], q), q) / P.J[0, 0]])
    A = sigma * left @ right
    tau23 = np.linalg.solve(A, np.array([nu1 - T['b1'], nu3 - T['b3']]))   # equation (26)
    tau = np.array([P.C1 * u_t - P.C2 * tau23[1], tau23[0], tau23[1]])
    tau_a = tau - P.K_OM * (P.J @ Om)                                          # equation (2)
    return dict(u_t=u_t, tau_a=tau_a, tau=tau, nu=np.array([nu1, nu2, nu3]), detA=np.linalg.det(A), b3q=b3q,
                h=np.array([T['h1'], T['h2'], T['h3']]), h3_dt=T['h3__dt'] + P.V_D, h1_dt=T['h1__dt'], h2_dt=T['h2__dt'],
                alpha=alpha, beta=beta, euler=np.array([theta, phi, psi]), gamma=gamma, z1=z1, z2=z2, e3=e3)


def dynamics(x, u_t, tau_a):
    """coordinate-free model (1) with the applied torque tau_a"""
    p, q, R, v, om, Om = unpack(x)
    q = q / np.linalg.norm(q)
    R = project_so3(R)
    M = P.M_Q + P.M_P
    b3 = R @ E3
    dp = v
    dq = np.cross(om, q)
    dR = R @ hat(Om)
    dv = -(P.M_Q * P.L / M) * float(om @ om) * q + P.G * E3 - u_t * float(b3 @ q) * q / M
    dom = u_t / (P.M_Q * P.L) * np.cross(q, b3)
    dOm = np.linalg.solve(P.J, tau_a - np.cross(Om, P.J @ Om))
    return np.concatenate([dp, dq, dR.ravel(), dv, dom, dOm])


def closed_loop(t, x, cp=None):
    c = controller(x, cp)
    return dynamics(x, c['u_t'], c['tau_a'])


def outputs_along(t, X):
    """controller quantities at every logged state"""
    cp = P.controller_params()
    keys = ('u_t', 'h3_dt', 'detA', 'b3q', 'alpha', 'beta')
    out = {k: [] for k in keys}; out.update(tau_a=[], tau=[], h=[], euler=[], nu=[])
    for x in X:
        c = controller(x, cp)
        for k in keys: out[k].append(c[k])
        for k in ('tau_a', 'tau', 'h', 'euler', 'nu'): out[k].append(c[k])
    return {k: np.array(v) for k, v in out.items()}
