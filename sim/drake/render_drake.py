"""Offline rendering of a logged Drake run with the VTK render engine of Drake.

    python3 drake/render_drake.py drake_run --cam side
    python3 drake/render_drake.py drake_run --cam orbit
    python3 drake/render_drake.py drake_run --cam follow
    python3 drake/render_drake.py drake_run --snapshots 0,1,1.5,2,5,6.5,8,10.5    one composite image

The log carries the plant positions at every controller step, which fixes the configuration. The path
is drawn as dots and the trace of the payload as a solid line, with depth occlusion from the depth
image of the sensor. The world of the renderer has its third axis up, so the path at height r* = -3 m
of the paper is 3 m above the ground.
"""
import os, sys, argparse, numpy as np
from pydrake.all import (DiagramBuilder, AddMultibodyPlantSceneGraph, RigidTransform, RotationMatrix, MakeRenderEngineVtk,
                         RenderEngineVtkParams, RgbdSensor, DepthRenderCamera, ColorRenderCamera, RenderCameraCore, CameraInfo,
                         ClippingRange, DepthRange, LightParameter, Box, Rgba, GeometryInstance, MakePhongIllustrationProperties,
                         PerceptionProperties)
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..', 'model'))
import params as P
from sls_plant import add_sls, F
RES = os.path.abspath(os.path.join(HERE, '..', 'results')); ANIM = os.path.join(RES, 'anim'); os.makedirs(ANIM, exist_ok=True)
W, H = 1280, 720
C_PATH, C_TRACE = (30, 140, 70), (245, 245, 245)
CENTER = np.array([0.0, 0.0, -P.R_STAR])          # center of the circle in the world of the renderer


def look_at(eye, target):
    fwd = target - eye; fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, np.array([0, 0, 1.0])); right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    return RigidTransform(RotationMatrix(np.c_[right, down, fwd]), eye)


def camera_pose(cam, t, T, p_quad=None, p_load=None):
    """world pose of the camera (+z forward, +x right, +y down)"""
    if cam == 'side':
        return look_at(np.array([11.5, -9.5, 6.5]), CENTER + np.array([-0.5, 0.5, 2.5]))
    if cam == 'orbit':
        az = np.radians(-40.0 + 360.0 * t / T); el = np.radians(22.0); d = 21.0
        eye = CENTER + np.array([0.0, 0.0, 1.5]) + d * np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        return look_at(eye, CENTER + np.array([0.0, 0.0, 1.5]))
    if cam == 'follow':
        mid = 0.5 * (p_quad + p_load)
        back = mid - CENTER; back[2] = 0.0; back /= max(np.linalg.norm(back), 1e-6)
        eye = mid + 5.8 * back + 2.4 * np.array([-back[1], back[0], 0.0]); eye[2] = p_quad[2] + 0.6
        return look_at(eye, mid)
    if cam == 'wide':
        return look_at(np.array([17.0, -17.0, 12.5]), CENTER + np.array([-1.5, 0.0, 5.0]))
    raise ValueError(cam)


def build_scene():
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    add_sls(plant); plant.Finalize()
    src = scene_graph.RegisterSource('env')
    ground = GeometryInstance(RigidTransform([0, 0, -0.05]), Box(80, 80, 0.1), 'ground')
    ground.set_illustration_properties(MakePhongIllustrationProperties(np.array([0.42, 0.45, 0.47, 1.0])))
    tex = os.path.join(RES, 'checker_ground.png')
    if not os.path.exists(tex):
        from PIL import Image
        n, cell = 40, 16
        a = np.zeros((n * cell, n * cell, 3), np.uint8)
        for i in range(n):
            for j in range(n):
                a[i * cell:(i + 1) * cell, j * cell:(j + 1) * cell] = (168, 176, 182) if (i + j) % 2 == 0 else (140, 150, 156)
        Image.fromarray(a).save(tex)
    pp = PerceptionProperties(); pp.AddProperty('phong', 'diffuse', Rgba(1.0, 1.0, 1.0, 1.0)); pp.AddProperty('phong', 'diffuse_map', tex)
    ground.set_perception_properties(pp)
    scene_graph.RegisterAnchoredGeometry(src, ground)
    params = RenderEngineVtkParams()
    params.default_clear_color = [0.80, 0.88, 0.97]
    key = LightParameter(); key.type = 'directional'; key.direction = [0.18, 0.12, -1.0]; key.intensity = 1.0; key.frame = 'world'
    fill = LightParameter(); fill.type = 'directional'; fill.direction = [-0.4, -0.25, -1.0]; fill.intensity = 0.45; fill.frame = 'world'
    low = LightParameter(); low.type = 'directional'; low.direction = [-0.3, 0.6, 0.45]; low.intensity = 0.5; low.frame = 'world'
    params.lights = [key, fill, low]
    params.cast_shadows = True; params.shadow_map_size = 4096
    scene_graph.AddRenderer('vtk', MakeRenderEngineVtk(params))
    intr = CameraInfo(W, H, np.radians(40.0))
    core = RenderCameraCore('vtk', intr, ClippingRange(0.1, 200.0), RigidTransform())
    color = ColorRenderCamera(core, False); depth = DepthRenderCamera(core, DepthRange(0.5, 150.0))
    sensor = builder.AddSystem(RgbdSensor(scene_graph.world_frame_id(), RigidTransform(), color, depth))
    builder.Connect(scene_graph.get_query_output_port(), sensor.query_object_input_port())
    diagram = builder.Build()
    return diagram, plant, sensor, intr


def project(X_WC, intr, pts):
    Pc = X_WC.inverse().multiply(pts.T).T
    z = Pc[:, 2]
    return intr.focal_x() * Pc[:, 0] / z + intr.center_x(), intr.focal_y() * Pc[:, 1] / z + intr.center_y(), z


def draw_overlay(img, depth, X_WC, intr, paths, traces):
    from PIL import Image, ImageDraw
    im = Image.fromarray(img[:, :, :3]); dr = ImageDraw.Draw(im)
    def visible(u, v, z):
        ui, vi = int(round(u)), int(round(v))
        if z <= 0 or ui < 0 or vi < 0 or ui >= W or vi >= H: return False
        d = depth[vi, ui]
        return (not np.isfinite(d)) or d <= 0 or z < d + 0.05
    for pts, rgb in paths:
        u, v, z = project(X_WC, intr, pts)
        for ui, vi, zi in zip(u, v, z):
            if visible(ui, vi, zi):
                r = max(1.6, 40.0 / zi); dr.ellipse([ui - r, vi - r, ui + r, vi + r], fill=rgb)
    for pts, rgb in traces:
        if len(pts) < 2: continue
        u, v, z = project(X_WC, intr, pts)
        for i in range(len(pts) - 1):
            if visible(u[i], v[i], z[i]) and visible(u[i + 1], v[i + 1], z[i + 1]):
                dr.line([u[i], v[i], u[i + 1], v[i + 1]], fill=rgb, width=max(2, int(45.0 / z[i])))
    return np.asarray(im)


def path_points(n=360):
    a = np.linspace(-np.pi, np.pi, n, endpoint=False)
    return [(np.c_[P.D_STAR * np.cos(a), P.D_STAR * np.sin(a), np.full(n, -P.R_STAR)] * np.array([1, -1, 1]), C_PATH)]


def world_points(L):
    """payload and quadrotor positions in the world of the renderer"""
    x = L['x']; p = x[:, 0:3]; q = x[:, 3:6]
    return (p @ F.T), ((p - P.L * q) @ F.T)


def font(size=22):
    from PIL import ImageFont
    try:
        return ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', size)
    except OSError:
        return ImageFont.load_default()


def caption(frame, t, lines):
    """time stamp and the lines of text in the upper left corner"""
    from PIL import Image, ImageDraw
    im = Image.fromarray(frame); dr = ImageDraw.Draw(im, 'RGBA')
    f1, f2 = font(30), font(20)
    w = max([dr.textlength(f't = {t:5.2f} s', font=f1)] + [dr.textlength(l, font=f2) for l in lines])
    dr.rectangle([16, 16, 16 + w + 28, 16 + 44 + 26 * len(lines) + 10], fill=(255, 255, 255, 170))
    dr.text((30, 22), f't = {t:5.2f} s', fill=(15, 15, 15), font=f1)
    for i, l in enumerate(lines):
        dr.text((30, 66 + 26 * i), l, fill=(40, 40, 40), font=f2)
    return np.asarray(im)


CAPTION = ['rotor on the +b2 arm failed before t = 0, three propellers', 'airframe, rod and payload drawn at three times their size']


def render_frame(plant, pctx, sensor, sctx, intr, xp, X_WC, paths, traces):
    plant.SetPositions(pctx, xp[:9])
    if hasattr(sensor, 'SetX_PB'): sensor.SetX_PB(sctx, X_WC)
    img = sensor.color_image_output_port().Eval(sctx).data.copy()
    dep = sensor.depth_image_32F_output_port().Eval(sctx).data.copy()[:, :, 0]
    return draw_overlay(img, dep, X_WC, intr, paths, traces), img[:, :, :3].copy(), dep


def render(logname, fps=30, cam='side'):
    L = np.load(os.path.join(RES, logname + '.npz')); t = L['t']; T = float(t[-1])
    diagram, plant, sensor, intr = build_scene()
    ctx = diagram.CreateDefaultContext(); pctx = plant.GetMyMutableContextFromRoot(ctx); sctx = sensor.GetMyMutableContextFromRoot(ctx)
    p_load, p_quad = world_points(L); paths = path_points()
    frames = []; ks = list(np.searchsorted(t, np.arange(0, T, 1.0 / fps)))
    for n, k in enumerate(ks):
        X_WC = camera_pose(cam, t[k], T, p_quad[k], p_load[k])
        frame, _, _ = render_frame(plant, pctx, sensor, sctx, intr, L['xp'][k], X_WC, paths, [(p_load[:k + 1:3], C_TRACE)])
        frames.append(caption(frame, t[k], CAPTION))
        if n % 100 == 0: print(f'  frame {n}/{len(ks)}')
    import imageio
    out = os.path.join(ANIM, f'{logname}_{cam}_drake.mp4')
    imageio.mimsave(out, frames, fps=fps, codec='libx264', quality=8, macro_block_size=None); print('wrote', out)


def one_frame(logname, time_s, cam):
    from PIL import Image
    L = np.load(os.path.join(RES, logname + '.npz')); t = L['t']; T = float(t[-1]); k = int(np.searchsorted(t, time_s))
    diagram, plant, sensor, intr = build_scene()
    ctx = diagram.CreateDefaultContext(); pctx = plant.GetMyMutableContextFromRoot(ctx); sctx = sensor.GetMyMutableContextFromRoot(ctx)
    p_load, p_quad = world_points(L)
    frame, _, _ = render_frame(plant, pctx, sensor, sctx, intr, L['xp'][k], camera_pose(cam, t[k], T, p_quad[k], p_load[k]), path_points(), [(p_load[:k + 1:3], C_TRACE)])
    out = os.path.join(ANIM, f'{logname}_{cam}_t{time_s:g}.png'); Image.fromarray(caption(frame, t[k], CAPTION)).save(out); print('wrote', out)


def snapshots(logname, times, cam='wide'):
    """one image with the system at the given times, nearest surface wins, with the trace up to the last time"""
    from PIL import Image, ImageDraw
    L = np.load(os.path.join(RES, logname + '.npz')); t = L['t']
    diagram, plant, sensor, intr = build_scene()
    ctx = diagram.CreateDefaultContext(); pctx = plant.GetMyMutableContextFromRoot(ctx); sctx = sensor.GetMyMutableContextFromRoot(ctx)
    p_load, p_quad = world_points(L); paths = path_points()
    X_WC = camera_pose(cam, 0.0, 1.0)
    ks = [int(np.searchsorted(t, tt)) for tt in times]
    best = None; bestd = None; labels = []
    for k in ks:
        frame, raw, dep = render_frame(plant, pctx, sensor, sctx, intr, L['xp'][k], X_WC, [], [])
        d = np.where(np.isfinite(dep) & (dep > 0), dep, np.inf)
        if best is None:
            best, bestd = raw.copy(), d.copy()
        else:
            m = d < bestd - 1e-3; best[m] = raw[m]; bestd[m] = d[m]
        u, v, z = project(X_WC, intr, p_quad[k][None, :]); labels.append((u[0], v[0], t[k]))
    img = draw_overlay(best, np.where(np.isfinite(bestd), bestd, np.nan), X_WC, intr, paths, [(p_load[:ks[-1] + 1:3], C_TRACE)])
    im = Image.fromarray(img); dr = ImageDraw.Draw(im); fnt = font(22)
    for u, v, tt in labels:
        dr.text((u + 16, v - 34), f't = {tt:g} s', fill=(20, 20, 20), font=fnt)
    out = os.path.join(ANIM, f'{logname}_snapshots.png'); im.save(out); print('wrote', out)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('log', help='log name in results/, for example drake_run')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--cam', default='side', choices=['side', 'orbit', 'follow', 'wide'])
    ap.add_argument('--snapshots', default=None, help='comma-separated times for one composite image')
    ap.add_argument('--frame', type=float, default=None, help='render the frame at this time to a PNG')
    a = ap.parse_args()
    if a.frame is not None:
        one_frame(a.log, a.frame, a.cam)
    elif a.snapshots:
        snapshots(a.log, [float(s) for s in a.snapshots.split(',')], cam=a.cam)
    else:
        render(a.log, a.fps, a.cam)
