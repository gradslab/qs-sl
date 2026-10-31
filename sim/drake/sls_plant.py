"""The slung payload system as a Drake MultibodyPlant.

The quadrotor is a free rigid body with the mass and inertia of the paper. The pendulum is one rigid
body attached to the quadrotor center of mass by a universal joint, so it has the two degrees of
freedom of the unit vector q and no spin about its own axis. Its mass is the payload, a sphere at the
end of a massless rod. The world frame of Drake has its third axis up, while the paper works in a
north-east-down frame. The two are related by the rotation F = diag(1, -1, -1), which is also the
relation between the body frame of the paper (third axis down, thrust along -b3) and the body frame
of the Drake model (third axis up). The functions below convert between the state of the paper
    x = (p, q, R, v, omega, Omega)
and the state of the plant, and apply the thrust and torque of the paper to the plant.
"""
import os, sys, numpy as np
from pydrake.all import (MultibodyPlant, SpatialInertia, UnitInertia, RotationalInertia, UniversalJoint, RigidTransform,
                         RotationMatrix, RollPitchYaw, SpatialVelocity, Mesh, Cylinder, Sphere, GeometryInstance, IllustrationProperties,
                         PerceptionProperties)
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'model'))
import params as P

ASSETS = os.path.abspath(os.path.join(HERE, '..', '..', 'assets'))
X2_OBJ = os.path.join(ASSETS, 'skydio_x2', 'assets', 'X2_lowpoly.obj')
F = np.diag([1.0, -1.0, -1.0])          # north-east-down <-> Drake world, paper body frame <-> Drake body frame
EZ = np.array([0.0, 0.0, 1.0])

# The X2 mesh is in centimeters with its second axis up. Its rotor hubs are 0.24 m from the center at a
# scale of 0.01, so the scale below puts them at the rotor distance ELL of the paper. The mesh is turned
# by 45 degrees so that the arms lie close to the body axes, as the model of the paper assumes.
MESH_SCALE = 0.01 * P.ELL / 0.24
MESH_YAW = np.pi / 4
# The scene is twenty meters across and the airframe half a meter, so the airframe, the failed-rotor marker, the rod
# and the payload are drawn VISUAL_SCALE times larger than they are, as Fig. 2 of the paper does. The dynamics use the
# true dimensions.
VISUAL_SCALE = 3.0
_HUBS_CM = np.array([[0.146, 0.19], [0.146, -0.19], [-0.147, 0.20], [-0.147, -0.20]]) / 0.01   # mesh body frame at scale 1 cm
PAYLOAD_RADIUS = 0.06
ROD_RADIUS = 0.012


def hub_positions():
    """rotor hub positions in the Drake body frame"""
    c, s = np.cos(MESH_YAW), np.sin(MESH_YAW)
    xy = (_HUBS_CM * MESH_SCALE) @ np.array([[c, s], [-s, c]])
    return np.c_[xy, np.full(4, 0.069 * MESH_SCALE / 0.01)]


def failed_hub():
    """the failed rotor is on the +b2 arm of the paper, which is the -y arm of the Drake body frame"""
    H = hub_positions()
    return H[np.argmin(np.linalg.norm(H[:, :2] - np.array([0.0, -P.ELL]), axis=1))]


def add_sls(plant, point_mass=False, visuals=True):
    """add the quadrotor and the pendulum. point_mass=True gives the payload no rotational inertia of its own,
    which is the model of the paper exactly; otherwise it is a solid sphere of radius PAYLOAD_RADIUS."""
    Gq = UnitInertia(P.J[0, 0] / P.M_Q, P.J[1, 1] / P.M_Q, P.J[2, 2] / P.M_Q)
    quad = plant.AddRigidBody('quad', SpatialInertia(P.M_Q, np.zeros(3), Gq))
    p_cm = np.array([0.0, 0.0, -P.L])
    if point_mass:
        Mp = SpatialInertia.MakeFromCentralInertia(P.M_P, p_cm, RotationalInertia(0.0, 0.0, 0.0))
    else:
        i = 0.4 * P.M_P * PAYLOAD_RADIUS ** 2
        Mp = SpatialInertia.MakeFromCentralInertia(P.M_P, p_cm, RotationalInertia(i, i, i))
    pend = plant.AddRigidBody('pendulum', Mp)
    joint = plant.AddJoint(UniversalJoint('pivot', quad.body_frame(), pend.body_frame()))
    if visuals:
        k = VISUAL_SCALE
        X_mesh = RigidTransform(RollPitchYaw(np.pi / 2, 0.0, np.pi + MESH_YAW), [0.0, 0.0, -0.03 * k * MESH_SCALE / 0.01])
        mesh = GeometryInstance(X_mesh, Mesh(X2_OBJ, k * MESH_SCALE), 'x2')      # no diffuse color, so the texture of its .mtl is used
        plant.RegisterVisualGeometry(quad, mesh)
        h = k * failed_hub()
        plant.RegisterVisualGeometry(quad, RigidTransform(h + np.array([0, 0, 0.006 * k])), Cylinder(0.105 * k * MESH_SCALE / 0.01 * 1.25, 0.004 * k),
                                     'failed_rotor', np.array([0.85, 0.08, 0.08, 0.75]))
        plant.RegisterVisualGeometry(pend, RigidTransform([0.0, 0.0, -P.L / 2]), Cylinder(k * ROD_RADIUS, P.L), 'rod', np.array([0.22, 0.22, 0.24, 1.0]))
        plant.RegisterVisualGeometry(pend, RigidTransform(p_cm), Sphere(k * PAYLOAD_RADIUS), 'payload', np.array([0.80, 0.42, 0.10, 1.0]))
        plant.RegisterVisualGeometry(quad, RigidTransform(), Sphere(0.02 * k), 'pivot', np.array([0.1, 0.1, 0.1, 1.0]))
    return quad, pend, joint


class SLSPlant:
    """a plant of its own for kinematic evaluation, and the conversions to and from the state of the paper"""
    def __init__(self, plant=None, point_mass=False):
        if plant is None:
            plant = MultibodyPlant(time_step=0.0)
            self.quad, self.pend, self.joint = add_sls(plant, point_mass=point_mass, visuals=False)
            plant.Finalize()
        else:
            self.quad, self.pend, self.joint = plant.GetBodyByName('quad'), plant.GetBodyByName('pendulum'), plant.GetJointByName('pivot')
        self.plant = plant
        self.ctx = plant.CreateDefaultContext()
        self.nq, self.nv = plant.num_positions(), plant.num_velocities()
        assert self.nq == 9 and self.nv == 8
        assert self.quad.floating_velocities_start_in_v() == 0 and self.joint.velocity_start() == 6

    def set_plant_state(self, xp, ctx=None):
        ctx = self.ctx if ctx is None else ctx
        self.plant.SetPositionsAndVelocities(ctx, xp)

    def ned_state(self, ctx=None):
        """state of the paper from the plant context"""
        ctx = self.ctx if ctx is None else ctx
        pl = self.plant
        X_WB = pl.EvalBodyPoseInWorld(ctx, self.quad); R_WB = X_WB.rotation().matrix(); pB = X_WB.translation()
        V_WB = pl.EvalBodySpatialVelocityInWorld(ctx, self.quad); wB = V_WB.rotational(); vB = V_WB.translational()
        R_WP = pl.EvalBodyPoseInWorld(ctx, self.pend).rotation().matrix()
        wP = pl.EvalBodySpatialVelocityInWorld(ctx, self.pend).rotational()
        qw = -R_WP[:, 2]; qdw = np.cross(wP, qw)
        p = F @ (pB + P.L * qw); q = F @ qw; v = F @ (vB + P.L * qdw)
        R = F @ R_WB @ F
        om = np.cross(q, F @ qdw)
        Om = R.T @ (F @ wB)
        return np.concatenate([p, q, R.ravel(), v, om, Om])

    def rotation(self, xp):
        """rotation of the Drake body frame from the plant positions (quaternion first)"""
        self.plant.SetPositions(self.ctx, xp[:self.nq])
        return self.plant.EvalBodyPoseInWorld(self.ctx, self.quad).rotation().matrix()

    def plant_state_from_ned(self, x):
        """plant positions and velocities from the state of the paper"""
        p = x[0:3]; q = x[3:6]; R = x[6:15].reshape(3, 3); v = x[15:18]; om = x[18:21]; Om = x[21:24]
        q = q / np.linalg.norm(q)
        qd = np.cross(om, q)
        pB = F @ (p - P.L * q); vB = F @ (v - P.L * qd); R_WB = F @ R @ F; wB = F @ (R @ Om)
        qb = F @ (R.T @ q)                                   # rod direction in the Drake body frame
        th2 = np.arcsin(np.clip(-qb[0], -1.0, 1.0)); th1 = np.arctan2(qb[1], -qb[2])
        qbd = F @ (R.T @ (qd - np.cross(R @ Om, q)))         # its derivative relative to the body
        c1, s1, c2, s2 = np.cos(th1), np.sin(th1), np.cos(th2), np.sin(th2)
        Jth = np.array([[0.0, -c2], [c1 * c2, -s1 * s2], [s1 * c2, c1 * s2]])
        thd = np.linalg.lstsq(Jth, qbd, rcond=None)[0]
        pl, ctx = self.plant, self.ctx
        pl.SetFreeBodyPose(ctx, self.quad, RigidTransform(RotationMatrix(R_WB), pB))
        pl.SetFreeBodySpatialVelocity(ctx, self.quad, SpatialVelocity(wB, vB))
        self.joint.set_angles(ctx, [th1, th2]); self.joint.set_angular_rates(ctx, thd)
        return pl.GetPositionsAndVelocities(ctx).copy()

    def generalized_force(self, R_WB, u_t, tau_a):
        """thrust along the third body axis and torque, as generalized forces of the plant (world frame)"""
        qf = np.zeros(self.nv)
        qf[0:3] = R_WB @ (F @ tau_a)
        qf[3:6] = u_t * R_WB @ EZ
        return qf
