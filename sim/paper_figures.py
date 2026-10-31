"""Figures of the paper (Figs. 2 to 7) and of the project page, from a logged run.

    python3 paper_figures.py                       paper style from results/drake_run.npz, PDF and PNG in results/figs
    python3 paper_figures.py --web                 page style, results/figs/W_*.png
    python3 paper_figures.py --log port_tight      another log

Every quantity is read from the log: the state of the paper x = (p, q, R, v, omega, Omega), the
thrust, the applied torques, the outputs and the 2-1-3 Euler angles.
"""
import os, sys, argparse, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, 'model')); sys.path.insert(0, HERE)
import params as P
from metrics import load, path_distance, yaw_unwrapped
RES = os.path.join(HERE, 'results'); FIG = os.path.join(RES, 'figs'); os.makedirs(FIG, exist_ok=True)
C_MAIN, C_2, C_3, C_K = '#1f4e9c', '#d9480f', '#2a9d8f', '#333333'
WEB = False


def style(web):
    global WEB
    WEB = web
    if web:
        rcParams.update({'font.size': 11, 'font.family': 'sans-serif', 'mathtext.fontset': 'dejavusans', 'axes.labelsize': 11,
                         'legend.fontsize': 10, 'xtick.labelsize': 10, 'ytick.labelsize': 10, 'lines.linewidth': 1.4,
                         'axes.grid': True, 'grid.alpha': 0.3, 'figure.dpi': 160, 'savefig.dpi': 160, 'axes.titlesize': 11.5})
    else:
        rcParams.update({'font.size': 8, 'font.family': 'serif', 'mathtext.fontset': 'cm', 'axes.labelsize': 8, 'legend.fontsize': 7.5,
                         'xtick.labelsize': 7.5, 'ytick.labelsize': 7.5, 'lines.linewidth': 1.0, 'axes.grid': True, 'grid.alpha': 0.3})


def save(fig, name):
    if WEB:
        out = os.path.join(FIG, 'W_' + name + '.png'); fig.savefig(out, bbox_inches='tight', pad_inches=0.05, facecolor='white')
    else:
        out = os.path.join(FIG, 'fig_' + name + '.pdf'); fig.savefig(out, bbox_inches='tight', pad_inches=0.02)
        fig.savefig(os.path.join(FIG, 'fig_' + name + '.png'), bbox_inches='tight', pad_inches=0.02, dpi=300)
    plt.close(fig); print('wrote', out)


def size(w, h):
    return (w, h) if WEB else (w * 3.45 / 10.5, h * 3.45 / 10.5 * 1.15)


def grid(rows, cols, w=10.5, h=None, **kw):
    h = h or 2.9 * rows
    fig, ax = plt.subplots(rows, cols, figsize=size(w, h), **kw)
    return fig, np.atleast_1d(ax).ravel()


def fig_snapshots(L, times=(0, 1, 1.5, 2, 5, 6.5, 8, 10.5)):
    """Fig. 2: the path, the payload trajectory and the system at the given times, north-east-down axes"""
    t = L['t']; x = L['x']; p = x[:, 0:3]; q = x[:, 3:6]; R = x[:, 6:15].reshape(-1, 3, 3)
    fig = plt.figure(figsize=size(8.5, 6.5)); ax = fig.add_subplot(111, projection='3d')
    a = np.linspace(0, 2 * np.pi, 400)
    ax.plot(P.D_STAR * np.cos(a), P.D_STAR * np.sin(a), np.full_like(a, P.R_STAR), color='#1a6633', lw=2.5, label=r'the path $\mathcal{C}$')
    ax.plot(*p.T, color='0.5', lw=1.0)
    arm, rp = P.ELL * 3.0, P.ELL * 1.2     # drawn larger than life so the quadrotor is visible at this scale
    for tt in times:
        k = int(np.searchsorted(t, tt)); c = p[k] - P.L * q[k]; Rk = R[k]
        for e in (np.array([1.0, 0, 0]), np.array([0, 1.0, 0])):
            seg = np.c_[c - arm * Rk @ e, c + arm * Rk @ e]; ax.plot(*seg, color='#1f4e9c', lw=1.5)
        th = np.linspace(0, 2 * np.pi, 40)
        for i, e in enumerate((np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([-1.0, 0, 0]), np.array([0, -1.0, 0]))):
            ctr = c + arm * Rk @ e
            ring = ctr[:, None] + rp * (np.outer(Rk[:, 0], np.cos(th)) + np.outer(Rk[:, 1], np.sin(th)))
            ax.plot(*ring, color='#b11226' if i == 1 else '#1f4e9c', lw=2.0 if i == 1 else 1.5)
        ax.plot(*np.c_[c, p[k]], color='0.35', lw=1.2); ax.plot(*p[k], 'o', color='k', ms=4)
        ax.text(c[0] + 0.5, c[1] + 0.5, c[2] - 1.5, f't = {tt:g} s', fontsize=8 if WEB else 6)
    ax.set_xlim(-10, 10); ax.set_ylim(-10, 10); ax.set_zlim(-15, 0)
    ax.invert_yaxis(); ax.invert_zaxis()
    ax.set_xlabel(r'$n_1$ [m]'); ax.set_ylabel(r'$n_2$ [m]'); ax.set_zlabel(r'$n_3$ [m]')
    ax.view_init(elev=25, azim=110); ax.legend(loc='upper right', frameon=False)
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane): pane.fill = False
    save(fig, 'snapshots')


def fig_inputs(L):
    """Fig. 3: the thrust and the three applied torques"""
    t = L['t']; fig, ax = grid(2, 2, h=5.6)
    ax[0].plot(t, L['u_t'], color=C_MAIN); ax[0].set_ylabel(r'$u_t$ [N]')
    for i in range(3):
        ax[i + 1].plot(t, L['tau_a'][:, i], color=C_MAIN); ax[i + 1].set_ylabel(fr'$\tau_{i + 1}$ [Nm]'); ax[i + 1].set_ylim(-0.05, 0.05)
    for a in ax: a.set_xlabel('t [s]'); a.set_xlim(0, t[-1])
    fig.tight_layout(); save(fig, 'inputs')


def fig_outputs(L):
    """Fig. 4: the outputs and the speed error"""
    t = L['t']; h = L['h']; fig, ax = grid(2, 2, h=5.6)
    ax[0].plot(t, h[:, 0], color=C_MAIN); ax[0].set_ylabel(r'$h_1$ [m$^2$]')
    ax[1].plot(t, h[:, 1], color=C_MAIN); ax[1].set_ylabel(r'$h_2$ [m]')
    ax[2].plot(t, np.unwrap(h[:, 2]), color=C_MAIN); ax[2].set_ylabel(r'$h_3$ [rad]')
    ax[3].plot(t, L['h3_dt'] - P.V_D, color=C_MAIN); ax[3].set_ylabel(r'$\dot h_3 - v_d$ [rad/s]')
    for a in ax: a.set_xlabel('t [s]'); a.set_xlim(0, t[-1])
    fig.tight_layout(); save(fig, 'outputs')


def fig_config(L):
    """Fig. 5: configuration variables in coordinates"""
    t = L['t']; x = L['x']; eu = L['euler']; fig, ax = grid(2, 4, h=5.2)
    for i in range(3):
        ax[i].plot(t, x[:, i], color=C_MAIN); ax[i].set_ylabel(fr'$p_{i + 1}$ [m]')
    ax[3].plot(t, L['alpha'], color=C_MAIN); ax[3].set_ylabel(r'$\alpha$ [rad]')
    ax[4].plot(t, L['beta'], color=C_MAIN); ax[4].set_ylabel(r'$\beta$ [rad]')
    ax[5].plot(t, eu[:, 0], color=C_MAIN); ax[5].set_ylabel(r'$\theta$ [rad]')
    ax[6].plot(t, eu[:, 1], color=C_MAIN); ax[6].set_ylabel(r'$\phi$ [rad]')
    ax[7].plot(t, yaw_unwrapped(L), color=C_MAIN); ax[7].set_ylabel(r'$\psi$ [rad]')
    for a in ax: a.set_xlabel('t [s]'); a.set_xlim(0, t[-1])
    fig.tight_layout(); save(fig, 'config')


def fig_pendulum(L):
    """Fig. 6: the pendulum direction and its angular velocity"""
    t = L['t']; x = L['x']; fig, ax = grid(2, 3, h=5.4)
    for i in range(3):
        ax[i].plot(t, x[:, 3 + i], color=C_MAIN); ax[i].set_ylabel(fr'$q_{i + 1}$'); ax[i].set_ylim(-1.05, 1.05)
        ax[3 + i].plot(t, x[:, 18 + i], color=C_MAIN); ax[3 + i].set_ylabel(fr'$\omega_{i + 1}$ [rad/s]')
    for a in ax: a.set_xlabel('t [s]'); a.set_xlim(0, t[-1])
    fig.tight_layout(); save(fig, 'pendulum')


def fig_attitude(L):
    """Fig. 7: the nine entries of the rotation matrix"""
    t = L['t']; R = L['x'][:, 6:15]; fig, ax = grid(3, 3, h=6.6)
    for i in range(9):
        ax[i].plot(t, R[:, i], color=C_MAIN); ax[i].set_ylabel(fr'$R_{{{i // 3 + 1}{i % 3 + 1}}}$'); ax[i].set_ylim(-1.05, 1.05)
    for a in ax: a.set_xlabel('t [s]'); a.set_xlim(0, t[-1])
    fig.tight_layout(); save(fig, 'attitude')


def fig_yaw(L):
    """page: yaw angle, yaw rate, the internal state xi_2 and the two functions that must stay positive on X_d"""
    t = L['t']; x = L['x']; Om = x[:, 21:24]; psi = yaw_unwrapped(L)
    fig, ax = grid(1, 3, h=3.3)
    ax[0].plot(t, np.gradient(psi, t), color=C_MAIN, label=r'$\dot\psi$ [rad/s]'); ax[0].plot(t, Om[:, 2], color=C_2, lw=1.0, label=r'$\Omega_3$ [rad/s]')
    ax[0].set_ylabel('yaw rate [rad/s]'); ax[0].legend(frameon=False)
    ax[1].plot(t, P.J[0, 0] * Om[:, 0] + P.J[2, 2] * P.C2 * Om[:, 2], color=C_MAIN); ax[1].set_ylabel(r'$\xi_2 = J_1\Omega_1 + J_3 c_2 \Omega_3$ [Nms]')
    ax[2].plot(t, L['b3q'], color=C_MAIN, label=r'$\langle b_3, q\rangle$'); ax[2].plot(t, np.cos(L['alpha']) * np.cos(L['beta']), color=C_2, label=r'$\cos\alpha\cos\beta$')
    ax[2].axhline(0, color='k', lw=0.8); ax[2].set_ylim(-0.1, 1.05); ax[2].legend(frameon=False, loc='lower right')
    for a in ax: a.set_xlabel('t [s]'); a.set_xlim(0, t[-1])
    fig.tight_layout(); save(fig, 'yaw')


def fig_distance(L, Lon=None):
    """page: distance of the payload to the path and the speed error, log scale, with the on-path start if given"""
    fig, ax = grid(1, 2, h=3.4)
    for Lk, lab, c in ((L, 'start of the paper', C_MAIN), (Lon, 'start on the path at rest', C_2)):
        if Lk is None: continue
        t = Lk['t']; d = path_distance(Lk)
        ax[0].semilogy(t, d + 1e-13, color=c, label=lab); ax[1].semilogy(t, np.abs(Lk['h3_dt'] - P.V_D) + 1e-13, color=c, label=lab)
    ax[0].set_ylabel('distance of the payload to the path [m]'); ax[1].set_ylabel(r'$|\dot h_3 - v_d|$ [rad/s]')
    for a in ax: a.set_xlabel('t [s]'); a.set_xlim(0, L['t'][-1]); a.legend(frameon=False)
    fig.tight_layout(); save(fig, 'distance')


def fig_compare(Ld, Lp, Lm=None):
    """page: the Drake run against the port on the point-mass model and the MATLAB reference"""
    fig, ax = grid(1, 3, h=3.4)
    t = Ld['t']
    def at(Lk):  # interpolate a log onto the Drake times
        return np.array([np.interp(t, Lk['t'], Lk['x'][:, i]) for i in range(3)]).T, np.interp(t, Lk['t'], Lk['u_t'])
    pp, up = at(Lp)
    ax[0].plot(t, 1e3 * np.linalg.norm(Ld['x'][:, 0:3] - pp, axis=1), color=C_MAIN, label='Drake against the port')
    ax[1].plot(t, Ld['u_t'] - up, color=C_MAIN)
    if Lm is not None:
        pm, um = at(Lm)
        ax[0].plot(t, 1e3 * np.linalg.norm(Ld['x'][:, 0:3] - pm, axis=1), color=C_2, label='Drake against MATLAB')
        ax[1].plot(t, Ld['u_t'] - um, color=C_2)
    ax[2].plot(t, path_distance(Ld) * 1e3, color=C_MAIN, label='Drake'); ax[2].plot(Lp['t'], path_distance(Lp) * 1e3, '--', color=C_2, label='port')
    ax[0].set_ylabel('payload position difference [mm]'); ax[1].set_ylabel('thrust difference [N]'); ax[2].set_ylabel('distance to the path [mm]')
    ax[2].set_yscale('log'); ax[2].set_ylim(1e-3, 2e4)
    for a in ax: a.set_xlabel('t [s]'); a.set_xlim(0, t[-1])
    ax[0].legend(frameon=False); ax[2].legend(frameon=False)
    fig.tight_layout(); save(fig, 'compare')


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--web', action='store_true'); ap.add_argument('--log', default='drake_run'); a = ap.parse_args()
    style(a.web); L = load(a.log)
    fig_snapshots(L); fig_inputs(L); fig_outputs(L); fig_config(L); fig_pendulum(L); fig_attitude(L)
    if a.web:
        Lon = load('drake_run_onpath') if os.path.exists(os.path.join(RES, 'drake_run_onpath.npz')) else None
        fig_yaw(L); fig_distance(L, Lon)
        Lm = load('matlab/reference_sim_tight') if os.path.exists(os.path.join(RES, 'matlab', 'reference_sim_tight.npz')) else None
        if Lm is not None:
            Lm = dict(t=Lm['t'], x=Lm['x'], u_t=Lm['out'][:, 0])
        fig_compare(L, load('port_tight'), Lm)


if __name__ == '__main__':
    main()
