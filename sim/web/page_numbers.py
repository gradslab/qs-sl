"""Collect every number the project page quotes into results/web/numbers.json.

    python3 web/page_numbers.py

The page is rendered from this file, so a rerun that changes a result changes the page.
"""
import os, sys, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); SIM = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, SIM); sys.path.insert(0, os.path.join(SIM, 'model')); sys.path.insert(0, os.path.join(SIM, 'drake'))
import metrics as M
import params as P
OUT = os.path.join(SIM, 'results', 'web'); os.makedirs(OUT, exist_ok=True)


def parameters():
    from sls_plant import PAYLOAD_RADIUS, MESH_SCALE
    return dict(m_q=P.M_Q, m_p=P.M_P, L=P.L, g=P.G, J=[P.J[0, 0], P.J[1, 1], P.J[2, 2]], ell=P.ELL, c_d=P.C_D, c1=P.C1, c2=P.C2, k_Om=P.K_OM,
                d_star=P.D_STAR, r_star=P.R_STAR, v_d=P.V_D, poles_1=P.POLES_1, poles_2=P.POLES_2, poles_3=P.POLES_3,
                K1=list(P.K1), K2=list(P.K2), K3=list(P.K3), p0=list(P.P0), alpha0_deg=-30.0, beta0_deg=20.0, theta0_deg=45.0, phi0_deg=20.0, psi0_deg=-60.0,
                v0=list(P.V0), gamma0=list(P.GAMMA0), Omega0=list(P.OMEGA_B0), T=P.T_END, ctrl_hz=1000, plant_hz=4000,
                payload_radius=PAYLOAD_RADIUS, payload_inertia=0.4 * P.M_P * PAYLOAD_RADIUS ** 2, pendulum_inertia=P.M_P * P.L ** 2,
                mesh_scale=MESH_SCALE)


def main():
    N = dict(metrics=M.all_metrics(), params=parameters())
    gates = os.path.join(OUT, 'gates.json')
    N['gates'] = json.load(open(gates)) if os.path.exists(gates) else {}
    sizes = os.path.join(OUT, 'anim_sizes.json')
    N['anim_mb'] = json.load(open(sizes)) if os.path.exists(sizes) else {}
    json.dump(N, open(os.path.join(OUT, 'numbers.json'), 'w'), indent=1)
    print('wrote', os.path.join(OUT, 'numbers.json'))


if __name__ == '__main__':
    main()
