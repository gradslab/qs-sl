"""Verification gate of the controller port.

Along the closed loop, the time derivative of each generated term must equal the next term of its
chain: d/dt h1^(k) = h1^(k+1) for k = 0..4, and likewise for h3. At the top of each chain the torques
are chosen so that d/dt h1^(5) = nu_1 and d/dt h3^(5) = nu_3, which tests the sixth-order drift terms
b1, b3 and the matrix A together. The second output has d/dt h2 = v_3 and d/dt v_3 = nu_2, which
tests the thrust. The generated h3__dt is the speed error dh3/dt - v_d, as in the MATLAB code. The derivatives are taken by a central difference of the terms along a short,
tightly integrated piece of the closed-loop solution. The test runs at the initial condition and at
states sampled from the closed-loop solution.

    python3 model/test_chains.py
"""
import os, sys, numpy as np
from scipy.integrate import solve_ivp
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params as P, sls
from qsf_gen import qsf_terms

CHAIN1 = ['h1', 'h1__dt', 'h1__ddt', 'h1__dddt', 'h1__ddddt', 'h1__dddddt']
CHAIN3 = ['h3', 'h3__dt', 'h3__ddt', 'h3__dddt', 'h3__ddddt', 'h3__dddddt']


def terms_at(x, cp):
    c = sls.controller(x, cp)
    p, q, R, v, om, Om = sls.unpack(x)
    T = qsf_terms(*p, *v, c['alpha'], c['beta'], *c['gamma'], *c['euler'], *Om, **cp)
    return T, c


def flow(x, h, cp):
    """state at time h along the closed loop from x"""
    s = solve_ivp(lambda t, y: sls.closed_loop(t, y, cp), (0.0, h), x, method='DOP853', rtol=1e-12, atol=1e-14)
    return s.y[:, -1]


def check_state(x, cp, h=2e-4):
    """largest relative mismatch over both chains, the thrust row and the top rows"""
    xp = flow(x, h, cp); xm = flow(x, -h, cp)
    xpp = flow(x, 2 * h, cp); xmm = flow(x, -2 * h, cp)
    Tp, _ = terms_at(xp, cp); Tm, _ = terms_at(xm, cp); Tpp, _ = terms_at(xpp, cp); Tmm, _ = terms_at(xmm, cp)
    T0, c0 = terms_at(x, cp)
    def ddt(name):  # fourth-order central difference
        return (-Tpp[name] + 8 * Tp[name] - 8 * Tm[name] + Tmm[name]) / (12 * h)
    rows = []
    for chain in (CHAIN1, CHAIN3):
        for k in range(5):
            num, ana = ddt(chain[k]), T0[chain[k + 1]]
            if chain[k] == 'h3':
                ana = ana + P.V_D      # the generated h3__dt is the speed error dh3/dt - v_d
            rows.append((f'd/dt {chain[k]} = {chain[k + 1]}', num, ana))
    rows.append(('d/dt h1^(5) = nu_1', ddt('h1__dddddt'), c0['nu'][0]))
    rows.append(('d/dt h3^(5) = nu_3', ddt('h3__dddddt'), c0['nu'][2]))
    rows.append(('d/dt h2 = v_3', ddt('h2'), T0['h2__dt']))
    rows.append(('d/dt v_3 = nu_2', ddt('h2__dt'), c0['nu'][1]))
    worst = 0.0
    for name, num, ana in rows:
        rel = abs(num - ana) / max(abs(ana), 1.0)
        worst = max(worst, rel)
    return worst, rows


def main():
    cp = P.controller_params()
    x0 = sls.initial_state()
    sol = solve_ivp(lambda t, y: sls.closed_loop(t, y, cp), (0.0, 12.0), x0, method='DOP853', rtol=1e-10, atol=1e-12,
                    t_eval=[0.0, 0.3, 1.0, 2.5, 5.0, 8.0, 12.0])
    ok = True; worst_all = 0.0
    for t, x in zip(sol.t, sol.y.T):
        worst, rows = check_state(x, cp)
        flag = 'PASS' if worst < 1e-7 else 'FAIL'
        ok &= worst < 1e-7; worst_all = max(worst_all, worst)
        print(f't = {t:5.2f} s  largest relative mismatch {worst:.1e}  {flag}')
        if flag == 'FAIL':
            for name, num, ana in rows:
                print(f'    {name:34s} numeric {num: .6e}  generated {ana: .6e}')
    from compare_matlab import record
    record('chains', worst_all)
    print('ALL PASS' if ok else 'SOME FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
