"""Verification gate of the Drake plant against the model (1) of the paper.

At random states and inputs the accelerations of the payload, of the pendulum direction and of the
quadrotor body computed by Drake must equal those of the coordinate-free model. The payload is a
point mass here, so that both models are the same. The state conversion is checked by a round trip.

    python3 drake/verify_plant.py
"""
import os, sys, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..', 'model'))
import params as P, sls
from sls_plant import SLSPlant, F


def random_state(rng):
    rr, ang = rng.uniform(2, 9), rng.uniform(0, 2 * np.pi)
    p = np.array([rr * np.cos(ang), rr * np.sin(ang), rng.uniform(-12, -1)])
    al, be = np.radians(rng.uniform(-50, 50, 2)); th, ph = np.radians(rng.uniform(-50, 50, 2)); ps = rng.uniform(-np.pi, np.pi)
    R = sls.R_of_euler(th, ph, ps); q = sls.q_of_angles(al, be)
    v = rng.uniform(-1, 1, 3); gam = rng.uniform(-0.5, 0.5, 2); om = sls.W_gamma(al, be) @ gam; Om = rng.uniform(-1, 1, 3)
    return np.concatenate([p, q, R.ravel(), v, om, Om])


def main():
    rng = np.random.default_rng(3)
    m = SLSPlant(point_mass=True); pl, ctx = m.plant, m.ctx
    port = pl.get_applied_generalized_force_input_port()
    derivs = pl.AllocateTimeDerivatives()
    worst_state = worst_acc = 0.0
    for _ in range(40):
        x = random_state(rng); u_t = rng.uniform(5, 80); tau_a = rng.uniform(-2, 2, 3)
        xp = m.plant_state_from_ned(x)
        m.set_plant_state(xp)
        x_back = m.ned_state()
        worst_state = max(worst_state, np.abs(x_back - x).max())
        R_WB = pl.EvalBodyPoseInWorld(ctx, m.quad).rotation().matrix()
        port.FixValue(ctx, m.generalized_force(R_WB, u_t, tau_a))
        # accelerations in Drake
        A_B = pl.EvalBodySpatialAccelerationInWorld(ctx, m.quad); A_P = pl.EvalBodySpatialAccelerationInWorld(ctx, m.pend)
        V_P = pl.EvalBodySpatialVelocityInWorld(ctx, m.pend); R_WP = pl.EvalBodyPoseInWorld(ctx, m.pend).rotation().matrix()
        qw = -R_WP[:, 2]; wP = V_P.rotational(); alP = A_P.rotational()
        a_payload_w = A_B.translational() + np.cross(alP, P.L * qw) + np.cross(wP, np.cross(wP, P.L * qw))
        qdd_w = np.cross(alP, qw) + np.cross(wP, np.cross(wP, qw))
        q = F @ qw; dv_drake = F @ a_payload_w; dom_drake = np.cross(q, F @ qdd_w)
        R = F @ R_WB @ F; dOm_drake = R.T @ (F @ A_B.rotational())
        # accelerations of the model
        xd = sls.dynamics(x, u_t, tau_a)
        dv, dom, dOm = xd[15:18], xd[18:21], xd[21:24]
        err = max(np.abs(dv - dv_drake).max() / max(1.0, np.abs(dv).max()), np.abs(dom - dom_drake).max() / max(1.0, np.abs(dom).max()),
                  np.abs(dOm - dOm_drake).max() / max(1.0, np.abs(dOm).max()))
        worst_acc = max(worst_acc, err)
    print(f'state round trip, largest error            {worst_state:.1e}  {"PASS" if worst_state < 1e-10 else "FAIL"}')
    print(f'accelerations Drake vs model, largest rel. {worst_acc:.1e}  {"PASS" if worst_acc < 1e-9 else "FAIL"}')
    ok = worst_state < 1e-10 and worst_acc < 1e-9
    from compare_matlab import record
    record('plant', worst_acc); record('plant_round_trip', worst_state)
    print('ALL PASS' if ok else 'SOME FAIL'); return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
