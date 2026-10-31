"""Closed-loop run of the quasi-static feedback on the Drake plant, the scenario of Section VI.

The plant is a continuous MultibodyPlant integrated with a third-order Runge-Kutta scheme at 4 kHz.
The controller runs at a fixed rate (1 kHz by default) with a zero-order hold on the body-frame
command (thrust and applied torque). The payload is a solid sphere, so the plant differs from the
point-mass model of the paper by the rotational inertia of the sphere.

    python3 drake/sim_drake.py                 results/drake_run.npz
    python3 drake/sim_drake.py --hz 400 --tag _400hz
    python3 drake/sim_drake.py --start on-path --tag _onpath    payload on the path at rest, the invariance case
    python3 drake/sim_drake.py --point-mass --tag _pointmass    payload without rotational inertia
"""
import os, sys, time, argparse, numpy as np
from pydrake.all import (DiagramBuilder, AddMultibodyPlantSceneGraph, LeafSystem, Simulator, ResetIntegratorFromFlags)
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..', 'model'))
import params as P, sls
from sls_plant import add_sls, SLSPlant
RES = os.path.abspath(os.path.join(HERE, '..', 'results')); os.makedirs(RES, exist_ok=True)


class ZOHController(LeafSystem):
    """reads the plant state at period dt, evaluates the quasi-static feedback and holds the command"""
    def __init__(self, model, dt):
        super().__init__()
        self.model, self.dt, self.cp = model, dt, P.controller_params()
        self.DeclareVectorInputPort('x', model.nq + model.nv)
        self.istate = self.DeclareDiscreteState(np.zeros(4))
        self.DeclareVectorOutputPort('cmd', 4, lambda c, o: o.SetFromVector(c.get_discrete_state(self.istate).get_value()))
        self.DeclarePeriodicDiscreteUpdateEvent(dt, 0.0, self.Update)
        self.log = []

    def Update(self, context, discrete_state):
        xp = self.get_input_port(0).Eval(context)
        self.model.set_plant_state(xp)
        x = self.model.ned_state()
        c = sls.controller(x, self.cp)
        self.log.append(dict(t=context.get_time(), x=x, xp=xp, u_t=c['u_t'], tau_a=c['tau_a'], tau=c['tau'], h=c['h'], h3_dt=c['h3_dt'],
                             euler=c['euler'], alpha=c['alpha'], beta=c['beta'], nu=c['nu'], detA=c['detA'], b3q=c['b3q']))
        discrete_state.get_mutable_vector(self.istate).SetFromVector(np.r_[c['u_t'], c['tau_a']])


class BodyFrameActuator(LeafSystem):
    """maps the held body-frame command to the generalized force at the current attitude"""
    def __init__(self, model):
        super().__init__()
        self.model = model
        self.DeclareVectorInputPort('x', model.nq + model.nv)
        self.DeclareVectorInputPort('cmd', 4)
        self.DeclareVectorOutputPort('qf', model.nv, self.CalcOutput)

    def CalcOutput(self, context, output):
        xp = self.get_input_port(0).Eval(context); cmd = self.get_input_port(1).Eval(context)
        output.SetFromVector(self.model.generalized_force(self.model.rotation(xp), cmd[0], cmd[1:4]))


def on_path_state(a0=0.0, psi=0.0):
    """hover with the payload on the path: at rest, pendulum vertical, quadrotor level with heading psi. Every
    transverse coordinate is zero at this state, so it lies on the path-following manifold."""
    p = np.array([P.D_STAR * np.cos(a0), P.D_STAR * np.sin(a0), P.R_STAR])
    return np.concatenate([p, [0.0, 0.0, 1.0], sls.rot_z(psi).ravel(), np.zeros(3), np.zeros(3), np.zeros(3)])


def run(name, T=P.T_END, hz=1000.0, x0=None, plant_hz=4000.0, point_mass=False, verbose=True):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    add_sls(plant, point_mass=point_mass); plant.Finalize()
    model = SLSPlant(plant=plant)
    zoh = builder.AddSystem(ZOHController(model, 1.0 / hz)); act = builder.AddSystem(BodyFrameActuator(model))
    builder.Connect(plant.get_state_output_port(), zoh.get_input_port(0))
    builder.Connect(plant.get_state_output_port(), act.get_input_port(0))
    builder.Connect(zoh.get_output_port(0), act.get_input_port(1))
    builder.Connect(act.get_output_port(0), plant.get_applied_generalized_force_input_port())
    diagram = builder.Build()
    sim = Simulator(diagram)
    ResetIntegratorFromFlags(sim, 'runge_kutta3', 1.0 / plant_hz)
    sim.get_mutable_integrator().set_fixed_step_mode(True)
    ctx = sim.get_mutable_context(); pctx = plant.GetMyMutableContextFromRoot(ctx)
    x0 = sls.initial_state() if x0 is None else x0
    plant.SetPositionsAndVelocities(pctx, model.plant_state_from_ned(x0))
    t0 = time.time()
    for t_stop in np.append(np.arange(2.0, T, 2.0), T):
        sim.AdvanceTo(t_stop)
        L = zoh.log[-1]
        if verbose:
            print(f'  t={L["t"]:5.2f}s h1={L["h"][0]: .2e} h2={L["h"][1]: .2e} h3dot-vd={L["h3_dt"] - P.V_D: .2e} u_t={L["u_t"]:6.2f} '
                  f'|tau|={np.abs(L["tau_a"]).max():.3f} yaw rate~{L["x"][23]:.2f} ({time.time() - t0:.0f}s)')
    log = {k: np.array([e[k] for e in zoh.log], dtype=float) for k in zoh.log[0]}
    log['hz'] = hz; log['plant_hz'] = plant_hz
    np.savez(os.path.join(RES, name + '.npz'), **log)
    print('wrote', os.path.join(RES, name + '.npz'))
    return log


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--hz', type=float, default=1000.0, help='controller rate')
    ap.add_argument('--T', type=float, default=P.T_END)
    ap.add_argument('--start', default='paper', choices=['paper', 'on-path'])
    ap.add_argument('--point-mass', action='store_true', help='payload without rotational inertia, the model of the paper exactly')
    ap.add_argument('--tag', default='')
    a = ap.parse_args()
    run('drake_run' + a.tag, T=a.T, hz=a.hz, x0=on_path_state() if a.start == 'on-path' else None, point_mass=a.point_mass)
