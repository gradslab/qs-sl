"""Numbers of the runs, computed from the logs in results/.

    python3 metrics.py                      prints them
    import metrics; metrics.all_metrics()   returns them as a dictionary (used by the project page)
"""
import os, sys, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, 'model'))
import params as P
RES = os.path.join(HERE, 'results')


def load(name):
    return dict(np.load(os.path.join(RES, name + '.npz')))


def path_distance(L):
    """distance of the payload to the circle: radial error and height error"""
    p = L['x'][:, 0:3]
    return np.hypot(np.hypot(p[:, 0], p[:, 1]) - P.D_STAR, p[:, 2] - P.R_STAR)


def yaw_unwrapped(L):
    return np.unwrap(L['euler'][:, 2])


def _first_after(t, mask):
    """first time after which mask holds for the rest of the run"""
    bad = np.where(~mask)[0]
    return float(t[bad[-1] + 1]) if len(bad) and bad[-1] + 1 < len(t) else (float(t[0]) if not len(bad) else None)


def run_metrics(name):
    L = load(name); t = L['t']; x = L['x']; d = path_distance(L); e = np.abs(L['h3_dt'] - P.V_D)
    Om = x[:, 21:24]; psi = yaw_unwrapped(L); psid = np.gradient(psi, t)
    late = t >= 10.0; after5 = t >= 5.0
    m = dict(T=float(t[-1]), hz=float(L['hz']) if 'hz' in L else None, d0=float(d[0]),
             t_d_below_10cm=_first_after(t, d < 0.10), t_d_below_1cm=_first_after(t, d < 0.01), t_d_below_1mm=_first_after(t, d < 1e-3),
             d_final=float(d[-1]), d_after_10s_max=float(d[late].max()), d_max=float(d.max()), d_after_1s_max=float(d[t >= 1.0].max()),
             t_speed_err_below_1pct=_first_after(t, e < 0.01 * P.V_D), speed_err_after_10s_max=float(e[late].max()),
             h1_final=float(L['h'][-1, 0]), h2_final=float(L['h'][-1, 1]),
             thrust_range=[float(L['u_t'].min()), float(L['u_t'].max())], thrust_after_5s=[float(L['u_t'][after5].min()), float(L['u_t'][after5].max())],
             tau_peak=[float(v) for v in np.abs(L['tau_a']).max(axis=0)], tau_after_5s_peak=[float(v) for v in np.abs(L['tau_a'][after5]).max(axis=0)],
             yaw_rate_mean_after_10s=float(psid[late].mean()), yaw_rate_max=float(np.abs(psid).max()), Omega3_final=float(Om[-1, 2]),
             yaw_total=float(psi[-1] - psi[0]), xi2_max=float(np.abs(P.J[0, 0] * Om[:, 0] + P.J[2, 2] * P.C2 * Om[:, 2]).max()),
             min_b3q=float(L['b3q'].min()), min_cacb=float((np.cos(L['alpha']) * np.cos(L['beta'])).min()),
             max_tilt_deg=float(np.degrees(np.abs(L['euler'][:, :2])).max()), max_pendulum_deg=float(np.degrees(np.abs(np.c_[L['alpha'], L['beta']])).max()),
             min_abs_detA=float(np.abs(L['detA']).min()), laps=float((np.unwrap(L['h'][:, 2])[-1] - np.unwrap(L['h'][:, 2])[0]) / (2 * np.pi)),
             tilt0_deg=float(np.degrees(np.arccos(np.clip(x[0, 14], -1.0, 1.0)))), t_yaw_rate_above_3=_first_after(t, psid > 3.0),
             yaw_rate_predicted=float(P.C_D * L['u_t'][t >= 15.0].mean() / (P.K_OM * P.J[2, 2])), Omega3_mean_after_15s=float(Om[t >= 15.0, 2].mean()),
             R33_after_10s=[float(x[late, 14].min()), float(x[late, 14].max())])
    return m


def compare(a, b):
    """largest differences of two logs at the times of the first"""
    A, B = load(a), load(b); t = A['t']
    pb = np.array([np.interp(t, B['t'], B['x'][:, i]) for i in range(3)]).T
    ub = np.interp(t, B['t'], B['u_t']); tb = np.array([np.interp(t, B['t'], B['tau_a'][:, i]) for i in range(3)]).T
    hb = np.interp(t, B['t'], B['h3_dt'])
    dp = np.linalg.norm(A['x'][:, 0:3] - pb, axis=1)
    pa = A['x'][:, 0:3]
    arc = P.D_STAR * np.abs(np.angle(np.exp(1j * (np.arctan2(pa[:, 1], pa[:, 0]) - np.arctan2(pb[:, 1], pb[:, 0])))))   # along the circle
    da = np.hypot(np.hypot(pa[:, 0], pa[:, 1]) - P.D_STAR, pa[:, 2] - P.R_STAR); db = np.hypot(np.hypot(pb[:, 0], pb[:, 1]) - P.D_STAR, pb[:, 2] - P.R_STAR)
    late = t >= 10.0
    return dict(max_dp_mm=float(1e3 * dp.max()), dp_after_5s_mm=float(1e3 * dp[t >= 5].max()), max_du=float(np.abs(A['u_t'] - ub).max()),
                max_dtau=float(np.abs(A['tau_a'] - tb).max()), dp_final_mm=float(1e3 * dp[-1]), arc_final_mm=float(1e3 * arc[-1]),
                transverse_after_10s_mm=float(1e3 * np.abs(da - db)[late].max()), du_after_5s=float(np.abs(A['u_t'] - ub)[t >= 5].max()),
                dtau_after_5s=float(np.abs(A['tau_a'] - tb)[t >= 5].max()), speed_bias_after_10s=float((A['h3_dt'] - hb)[late].mean()))


def matlab_compare():
    out = {}
    for tag in ('default', 'tight'):
        f = os.path.join(RES, 'matlab', f'reference_sim_{tag}.npz')
        if not os.path.exists(f): continue
        M = np.load(f); D = load('port_' + tag); dp = np.linalg.norm(D['x'][:, 0:3] - M['x'][:, 0:3], axis=1)
        out[tag] = dict(max_dp_mm=float(1e3 * dp.max()), max_du=float(np.abs(D['u_t'] - M['out'][:, 0]).max()))
    return out


def all_metrics():
    out = dict(drake=run_metrics('drake_run'), port=run_metrics('port_tight'))
    for name, key in (('drake_run_onpath', 'onpath'), ('drake_run_400hz', 'drake_400hz'), ('drake_run_pointmass', 'drake_pointmass')):
        if os.path.exists(os.path.join(RES, name + '.npz')): out[key] = run_metrics(name)
    out['drake_vs_port'] = compare('drake_run', 'port_tight')
    for key, name in (('drake_400hz', 'drake_run_400hz'), ('drake_pointmass', 'drake_run_pointmass')):
        if key in out: out[key + '_vs_port'] = compare(name, 'port_tight')
    out['port_vs_matlab'] = matlab_compare()
    return out


if __name__ == '__main__':
    print(json.dumps(all_metrics(), indent=1))
