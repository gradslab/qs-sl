"""Closed-loop simulation of the Python port on the coordinate-free model, the scenario of Section VI.

    python3 model/simulate_port.py            default tolerances of ode45 (rtol 1e-3, atol 1e-6), as matlab/main.m
    python3 model/simulate_port.py --tight    rtol 1e-9, atol 1e-11

Writes results/port_<tag>.npz with the state and the controller quantities on the grid 0:0.01:20 s.
"""
import os, sys, time, argparse, numpy as np
from scipy.integrate import solve_ivp
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import params as P, sls
RES = os.path.abspath(os.path.join(HERE, '..', 'results')); os.makedirs(RES, exist_ok=True)


def run(tight=False, T=P.T_END, dt=0.01):
    cp = P.controller_params()
    t_eval = np.arange(0.0, T + dt / 2, dt)
    x0 = sls.initial_state()
    kw = dict(method='DOP853', rtol=1e-9, atol=1e-11) if tight else dict(method='RK45', rtol=1e-3, atol=1e-6)
    t0 = time.time()
    sol = solve_ivp(lambda t, y: sls.closed_loop(t, y, cp), (0.0, T), x0, t_eval=t_eval, **kw)
    el = time.time() - t0
    assert sol.success, sol.message
    X = sol.y.T
    out = sls.outputs_along(sol.t, X)
    tag = 'tight' if tight else 'default'
    np.savez(os.path.join(RES, f'port_{tag}.npz'), t=sol.t, x=X, nfev=sol.nfev, **out)
    print(f'port_{tag}: {sol.nfev} evaluations in {el:.0f} s; final h = {out["h"][-1]}, h3dot = {out["h3_dt"][-1]:.4f}, '
          f'yaw rate ~ {np.diff(np.unwrap(out["euler"][-200:, 2])).mean() / dt:.3f} rad/s')


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--tight', action='store_true'); a = ap.parse_args()
    run(a.tight)
