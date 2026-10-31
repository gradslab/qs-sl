"""Compare the Python port with the MATLAB implementation.

results/matlab/ holds what matlab/reference_run.m writes, converted by model/matlab_csv_to_npz.py: the controller and the vector field at 60
states (the initial condition and 59 random states) and the closed loop on 0:0.01:20 s with the
default tolerances of ode45 and with tight tolerances. The output columns are
    u_t, tau_1, tau_2, tau_3 (applied), h1, h2, h3, dh3/dt - v_d, h3^(4), h3^(5), residual of (20).

    python3 model/compare_matlab.py
"""
import os, sys, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import params as P, sls
RES = os.path.abspath(os.path.join(HERE, '..', 'results'))


def rel(a, b):
    a, b = np.atleast_1d(a), np.atleast_1d(b)
    return np.abs(a - b).max() / max(np.abs(b).max(), 1.0)


def states():
    D = np.load(os.path.join(RES, 'matlab', 'reference_states.npz')); Z, O = D['states'], D['outputs']
    cp = P.controller_params(); worst = {}
    for z, o in zip(Z, O):
        c = sls.controller(z, cp); xd = sls.closed_loop(0.0, z, cp)
        pairs = dict(vector_field=(xd, o[:24]), u_t=(c['u_t'], o[24]), tau=(c['tau_a'], o[25:28]), h=(c['h'], o[28:31]),
                     h3_speed_error=(c['h3_dt'] - P.V_D, o[31]), h3_4=(c['e3'][3], o[32]), h3_5=(c['e3'][4], o[33]))
        for k, (a, b) in pairs.items():
            worst[k] = max(worst.get(k, 0.0), rel(a, b))
    print(f'{len(Z)} states, largest relative difference per quantity:')
    for k, v in worst.items():
        print(f'    {k:16s} {v:.1e}')
    return max(worst.values())


def runs():
    out = {}
    for tag in ('default', 'tight'):
        fm = os.path.join(RES, 'matlab', f'reference_sim_{tag}.npz'); fp = os.path.join(RES, f'port_{tag}.npz')
        if not (os.path.exists(fm) and os.path.exists(fp)):
            print(f'{tag}: missing', fm if not os.path.exists(fm) else fp); continue
        M = np.load(fm); D = np.load(fp)
        assert np.allclose(M['t'], D['t'])
        dp = np.linalg.norm(D['x'][:, 0:3] - M['x'][:, 0:3], axis=1)
        du = np.abs(D['u_t'] - M['out'][:, 0]); dtau = np.abs(D['tau_a'] - M['out'][:, 1:4]).max(axis=1)
        out[tag] = dict(max_dp=float(dp.max()), max_du=float(du.max()), max_dtau=float(dtau.max()), ok=bool(tag == 'default' or dp.max() < 1e-8))
        print(f'{tag}: largest payload position difference {dp.max():.2e} m, thrust {du.max():.2e} N, torque {dtau.max():.2e} Nm')
    return out


def record(name, value):
    """keep the worst value of a gate in results/web/gates.json for the page"""
    import json
    os.makedirs(os.path.join(RES, 'web'), exist_ok=True)
    f = os.path.join(RES, 'web', 'gates.json')
    G = json.load(open(f)) if os.path.exists(f) else {}
    G[name] = value
    json.dump(G, open(f, 'w'), indent=1)


if __name__ == '__main__':
    w = states(); r = runs()
    ok = w < 1e-9 and all(v['ok'] for v in r.values())
    record('matlab_states', w)
    if 'tight' in r:
        record('matlab_tight_dp', r['tight']['max_dp']); record('matlab_tight_du', r['tight']['max_du'])
    print('ALL PASS' if ok else 'SOME FAIL')
    sys.exit(0 if ok else 1)
