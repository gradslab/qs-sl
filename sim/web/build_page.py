r"""Render docs/index.html from results/web/numbers.json and copy the figures to docs/figs.

Run page_numbers.py, paper_figures.py --web and anim_web.py first. No result is written into the page by hand.
"""
import os, json, math, shutil

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.abspath(os.path.join(HERE, '..'))
SITE = os.path.abspath(os.path.join(SIM, '..', 'docs'))
N = json.load(open(os.path.join(SIM, 'results', 'web', 'numbers.json')))
REPO = "https://github.com/gradslab/qs-sl"

FIGS = ['W_snapshots', 'W_outputs', 'W_inputs', 'W_config', 'W_pendulum', 'W_attitude', 'W_yaw', 'W_distance', 'W_compare']


def f(x, spec=".2f", dash="n/a"):
    return dash if x is None else format(x, spec)


def sig(x, n=2, dash="n/a"):
    """2.9e-05 becomes 2.9 x 10^-5, 0.0021 stays decimal"""
    if x is None:
        return dash
    if x == 0:
        return "0"
    e = math.floor(math.log10(abs(x)))
    if -3 <= e <= 4:
        return f"{x:,.{max(0, n - 1 - e)}f}".rstrip("0").rstrip(".")
    m = x / 10 ** e
    return f"{m:.1f}&times;10<sup>{e}</sup>"


def rng(v, spec=".2f", unit=""):
    return f"[{format(v[0], spec)}, {format(v[1], spec)}]{unit}"


SEC = [("gallery", "Animations"),
       ("scenario", "The scenario of the paper"),
       ("outputs", "Outputs and inputs"),
       ("configuration", "Configuration and attitude"),
       ("invariance", "Starting on the path"),
       ("implementations", "Three implementations"),
       ("parameters", "Parameters"),
       ("reproduce", "Reproducing this"),
       ("cite", "Citation")]


def rail():
    items = "\n".join(f'      <li><a href="#{i}">{t}</a></li>' for i, t in SEC)
    return f'''  <aside class="rail">
    <div class="rail-h">On this page</div>
    <ol>
{items}
    </ol>
  </aside>'''


def video(base, alt=""):
    return (f'<div class="plate"><video src="anim/{base}.mp4" poster="anim/{base}.jpg" '
            f'autoplay loop muted playsinline preload="metadata" aria-label="{alt}"></video></div>')


def card(pill, pill_cls, title, base, alt, text=""):
    t = f"<h3>{title}</h3>" if title else ""
    p = f"<p>{text}</p>" if text else ""
    return f'<div class="card media"><span class="pill {pill_cls}">{pill}</span>{t}{video(base, alt)}{p}</div>'


def figure(src, cap):
    return f'''      <figure class="figblock">
        <div class="plate"><img src="{src}" alt="" loading="lazy"></div>
        <figcaption class="cap">{cap}</figcaption>
      </figure>'''


def table(head, rows, note=None):
    th = "".join(f'<th class="num">{h}</th>' if i else f"<th>{h}</th>" for i, h in enumerate(head))
    body = ""
    for r in rows:
        if isinstance(r, str):
            body += f'<tr><td colspan="{len(head)}"><b>{r}</b></td></tr>'
        else:
            body += "<tr>" + "".join(f'<td class="num">{c}</td>' if i else f"<td>{c}</td>" for i, c in enumerate(r)) + "</tr>"
    out = f'<div class="tablewrap"><table><thead><tr>{th}</tr></thead><tbody>{body}</tbody></table></div>'
    if note:
        out += f'<p class="cap">{note}</p>'
    return out


def params_table(title, rows):
    body = "".join(f"<tr><td>{k}</td><td>{v}</td></tr>" for k, v in rows)
    return (f'<h3 style="margin:1.6rem 0 .7rem">{title}</h3>'
            f'<div class="tablewrap"><table><thead><tr><th>Parameter</th><th>Value</th></tr></thead><tbody>{body}</tbody></table></div>')


def build():
    M = N["metrics"]; P = N["params"]; G = N["gates"]
    D, Q, ON = M["drake"], M["port"], M["onpath"]
    PM, H4 = M.get("drake_pointmass"), M.get("drake_400hz")
    DvP, PMvP, H4vP, PvM = M["drake_vs_port"], M.get("drake_pointmass_vs_port"), M.get("drake_400hz_vs_port"), M["port_vs_matlab"]
    hover = (P["m_q"] + P["m_p"]) * P["g"]
    speed_mps = P["v_d"] * P["d_star"]

    head = f'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Fault-Tolerant Path Following for a Quadrotor with a Slung Payload</title>
<meta name="description" content="Simulation study for quasi-static fault-tolerant feedback control of a quadrotor carrying a slung payload under a complete single-rotor failure.">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Newsreader:ital,opsz,wght@0,6..72,400;0,6..72,500;0,6..72,600;1,6..72,400&family=IBM+Plex+Sans:ital,wght@0,400;0,500;0,600;1,400&family=IBM+Plex+Mono:wght@400;500&display=swap" rel="stylesheet">
<link rel="stylesheet" href="assets/css/site.css">
<script>
window.MathJax = {{tex:{{inlineMath:[["\\\\(","\\\\)"]]}}, svg:{{fontCache:"global"}}}};
</script>
<script defer src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-svg.js"></script>
</head>
<body>

<header class="topbar">
  <span class="mark">Slung payload &middot; rotor failure</span>
  <span class="spacer"></span>
  <nav>
    <a href="#gallery">Animations</a>
    <a href="#scenario">Scenario</a>
    <a href="#outputs">Outputs</a>
    <a href="#invariance">Invariance</a>
    <a href="#implementations">Implementations</a>
    <a href="#parameters">Parameters</a>
    <a href="#reproduce">Reproduce</a>
  </nav>
  <button id="theme" class="btn" type="button" aria-label="Switch colour theme">Theme</button>
</header>

<div class="shell">
{rail()}
  <main>

  <section class="hero">
    <div class="eyebrow">Supplementary simulation study</div>
    <h1>Quasi-Static Fault-Tolerant Feedback Control of a Quadrotor Carrying a Slung Payload under Rotor Failure</h1>
    <p class="byline">Mohamed Al Lawati and Adeel Akhtar
      <span class="aff">Sultan Qaboos University and New Jersey Institute of Technology</span></p>
    <p class="lede">A quadrotor carries a payload on a massless rod pivoted at its center of mass. One rotor has failed
      completely, so three propellers remain and the roll torque is tied to the thrust and the yaw torque. The
      quasi-static feedback of the paper is designed for the faulty dynamics. It drives the payload onto a geometric
      path, keeps the path invariant, assigns the motion along it and keeps the yaw rate bounded. This page carries a
      Drake simulation of the scenario of the paper, the figures of the paper regenerated from it, a run that starts on
      the path, a comparison of three implementations of the closed loop, and everything needed to reproduce them.</p>
    <div class="buttons">
      <a class="btn primary" href="{REPO}">Code and data</a>
      <a class="btn" href="#invariance">Invariance</a>
      <a class="btn" href="#reproduce">Reproduce the figures</a>
    </div>
  </section>
'''

    gallery = f'''
  <section class="sec" id="gallery">
    <div class="sec-head">
      <span class="sec-tag">Animations</span>
      <h2>The payload converges to the circle while the airframe spins</h2>
      <p>Rendered in Drake from the logged run. The airframe is a Skydio X2 with the mass and inertia of the paper,
      the rod is {f(P["L"], ".0f")}&nbsp;m long and the payload is a {f(P["m_p"], ".1f")}&nbsp;kg sphere. The failed rotor is
      marked in red. The green dots are the path, a horizontal circle of radius {f(P["d_star"], ".0f")}&nbsp;m at
      {f(-P["r_star"], ".0f")}&nbsp;m above the ground, and the white line is the trace of the payload. The payload starts
      {f(D["d0"], ".1f")}&nbsp;m from the path with the quadrotor pitched by {f(P["theta0_deg"], ".0f")}&nbsp;degrees and rolled
      by {f(P["phi0_deg"], ".0f")}&nbsp;degrees. The failure ties the roll torque to the thrust and the yaw torque. With the roll
      torque near zero the yaw torque equals \\(c_d u_t\\), and the damping of the paper balances it at
      \\(\\Omega_3 = c_d u_t / (k_\\Omega J_3)\\), which is {f(D["yaw_rate_predicted"], ".2f")}&nbsp;rad/s for the thrust of
      this run. So the airframe spins up within {f(D["t_yaw_rate_above_3"], ".1f")}&nbsp;s and turns about its vertical axis at
      about {f(D["yaw_rate_mean_after_10s"], ".1f")}&nbsp;rad/s from then on, while the payload converges to the circle and
      settles at the commanded angular speed of {f(P["v_d"], ".1f")}&nbsp;rad/s, which is {f(speed_mps, ".0f")}&nbsp;m/s along
      the circle.</p>
    </div>
<div class="grid2">{card("following camera", "neutral", "", "follow", "scenario of the paper, camera following the system")}{card("side on", "neutral", "", "side", "scenario of the paper, fixed camera")}</div>
    <div class="sec-head" style="margin-top:2.6rem">
      <h2>Orbiting camera</h2>
      <p>The same run seen from a camera that circles the path once over the {f(D["T"], ".0f")}&nbsp;s of the run.</p>
    </div>
<div class="grid2">{card("orbit", "neutral", "", "orbit", "scenario of the paper, orbiting camera")}{card("starting on the path", "ok", "", "onpath_follow", "hover start on the path, camera following the system", "The hover start on the path. The payload accelerates to the commanded speed without leaving the circle. Details <a href='#invariance'>below</a>.")}</div>
    <div class="stats" style="margin-top:2.2rem">
      <div class="stat"><div class="n">{f(D["t_d_below_1cm"], ".1f")} s</div>
        <div class="k">after which the payload stays within 1&nbsp;cm of the path, from {f(D["d0"], ".1f")}&nbsp;m at the start</div></div>
      <div class="stat"><div class="n">{f(1e3 * D["d_final"], ".2f")} mm</div>
        <div class="k">distance of the payload to the path at the end of the run, set by the rotational inertia of the spherical payload</div></div>
      <div class="stat"><div class="n">{f(D["yaw_rate_mean_after_10s"], ".2f")} rad/s</div>
        <div class="k">yaw rate after 10&nbsp;s. The yaw angle grows by {f(D["yaw_total"], ".0f")}&nbsp;rad over the run and the yaw rate never exceeds {f(D["yaw_rate_max"], ".2f")}&nbsp;rad/s</div></div>
      <div class="stat"><div class="n">{f(D["thrust_after_5s"][0], ".1f")} N</div>
        <div class="k">thrust after 5&nbsp;s, against a weight of {f(hover, ".1f")}&nbsp;N. The peak is {f(D["thrust_range"][1], ".1f")}&nbsp;N during the transient</div></div>
    </div>
  </section>
'''

    scenario = f'''
  <section class="sec" id="scenario">
    <div class="sec-head">
      <span class="sec-tag">Scenario</span>
      <h2>The scenario of the paper</h2>
      <p>The path is the circle \\(h_1 = p_1^2 + p_2^2 - d^{{*2}} = 0\\), \\(h_2 = p_3 - r^* = 0\\) with \\(d^* = {f(P["d_star"], ".0f")}\\)&nbsp;m
      and \\(r^* = {f(P["r_star"], ".0f")}\\)&nbsp;m in the north-east-down frame. The path coordinate is the polar angle
      \\(h_3 = \\mathrm{{atan2}}(p_2, p_1)\\) and its rate is driven to \\(v_d = {f(P["v_d"], ".1f")}\\)&nbsp;rad/s, as in the MATLAB code
      behind the figures of the paper, so the payload moves at {f(speed_mps, ".0f")}&nbsp;m/s along the circle. The payload starts at
      \\(p(0) = ({f(P["p0"][0], ".0f")}, {f(P["p0"][1], ".0f")}, {f(P["p0"][2], ".0f")})\\)&nbsp;m with the pendulum at
      \\(\\alpha = {f(P["alpha0_deg"], ".0f")}^\\circ\\), \\(\\beta = {f(P["beta0_deg"], ".0f")}^\\circ\\) and the quadrotor at
      \\(\\theta = {f(P["theta0_deg"], ".0f")}^\\circ\\), \\(\\phi = {f(P["phi0_deg"], ".0f")}^\\circ\\), \\(\\psi = {f(P["psi0_deg"], ".0f")}^\\circ\\),
      with the velocities of Section VI. The controller is the quasi-static feedback (17), (26) of the paper with the
      damping (2), and the gains place the poles of the three chains at the values of the paper.</p>
    </div>
{figure("anim/snapshots.png", f"<b>Snapshots of the Drake run</b> at t = 0, 1, 1.5, 2, 5, 6.5, 8 and 10.5 s, the times of Fig. 2 of the paper, composed from one camera. The white line is the trace of the payload up to t = 10.5 s.")}
{figure("figs/W_snapshots.png", "<b>Fig. 2 of the paper from the Drake log.</b> The same times, drawn in the north-east-down axes of the paper. The quadrotor is drawn larger than life, the red ring is the failed rotor and the black dot is the payload.")}
  </section>
'''

    outputs = f'''
  <section class="sec" id="outputs">
    <div class="sec-head">
      <span class="sec-tag">Outputs and inputs</span>
      <h2>Outputs and inputs</h2>
      <p>The two transverse outputs decay to zero and the speed error vanishes, as the transverse dynamics (31) of the
      paper require. The thrust settles at {f(D["thrust_after_5s"][0], ".1f")}&nbsp;N, a little above the weight because the
      pendulum leans inward on the circle. After the transient the torques stay small. The roll torque
      \\(\\tau_1 = c_1 u_t - c_2\\tau_3\\) is not free: it is set by the thrust and the yaw torque through the failure.</p>
    </div>
{figure("figs/W_outputs.png", f"<b>Fig. 4 of the paper from the Drake log.</b> The outputs \\(h_1\\), \\(h_2\\), the unwrapped path coordinate \\(h_3\\) and the speed error \\(\\dot h_3 - v_d\\). The payload is within 10 cm of the path after {f(D['t_d_below_10cm'], '.1f')} s, within 1 cm after {f(D['t_d_below_1cm'], '.1f')} s and within 1 mm after {f(D['t_d_below_1mm'], '.1f')} s. The speed error is below 1 % of \\(v_d\\) after {f(D['t_speed_err_below_1pct'], '.1f')} s and below {sig(D['speed_err_after_10s_max'], 2)} rad/s after 10 s.")}
{figure("figs/W_inputs.png", f"<b>Fig. 3 of the paper from the Drake log.</b> The thrust and the three applied torques, with the torque axes clipped to &plusmn;0.05 Nm as in the paper. The thrust ranges over {rng(D['thrust_range'], '.1f')} N and the torques peak at ({', '.join(f(v, '.1f') for v in D['tau_peak'])}) Nm in the first second. After 5 s they stay below ({', '.join(f(v, '.3f') for v in D['tau_after_5s_peak'])}) Nm.")}
  </section>
'''

    configuration = f'''
  <section class="sec" id="configuration">
    <div class="sec-head">
      <span class="sec-tag">Configuration</span>
      <h2>Configuration, pendulum and attitude</h2>
      <p>The payload position, the pendulum angles and the Euler angles of the quadrotor, then the pendulum direction
      with its angular velocity and the nine entries of the rotation matrix. The yaw angle grows without bound, by
      {f(D["yaw_total"], ".0f")}&nbsp;rad over the run, while its rate stays bounded. This is the internal dynamics of
      Theorem 1: the yaw rate and the state \\(\\xi_2 = J_1\\Omega_1 + J_3 c_2\\Omega_3\\) are bounded along the solution.</p>
    </div>
{figure("figs/W_config.png", "<b>Fig. 5 of the paper from the Drake log.</b> Payload position, pendulum angles and 2-1-3 Euler angles. The pitch and roll oscillate at the period of the circle once the payload is on it.")}
{figure("figs/W_pendulum.png", f"<b>Fig. 6 of the paper from the Drake log.</b> The pendulum direction \\(q\\) and its angular velocity \\(\\omega\\). The pendulum never swings further than {f(D['max_pendulum_deg'], '.1f')} degrees from the vertical.")}
{figure("figs/W_attitude.png", f"<b>Fig. 7 of the paper from the Drake log.</b> The rotation matrix of the quadrotor. The first two columns turn at the yaw rate. The third column is \\(b_3\\), which does not depend on the yaw: it carries the small tilt that holds the payload on the circle and turns with the period of the circle, and \\(R_{{33}}\\) stays near {f(D['R33_after_10s'][0])}.")}
{figure("figs/W_yaw.png", f"<b>The internal state.</b> Left: the yaw rate \\(\\dot\\psi\\) and the body rate \\(\\Omega_3\\), which settle at {f(D['yaw_rate_mean_after_10s'], '.2f')} rad/s with a peak of {f(D['yaw_rate_max'], '.2f')} rad/s. Middle: the internal state \\(\\xi_2\\), bounded by {f(D['xi2_max'], '.2f')} Nms. Right: the two functions that must stay positive on \\(X_d\\). Over the run \\(\\langle b_3, q\\rangle \\ge {f(D['min_b3q'], '.2f')}\\) and \\(\\cos\\alpha\\cos\\beta \\ge {f(D['min_cacb'], '.2f')}\\), so the solution stays in the domain of the controller and \\(\\tilde A\\) stays invertible, with \\(|\\det\\tilde A| \\ge {sig(D['min_abs_detA'], 2)}\\).")}
  </section>
'''

    invariance = f'''
  <section class="sec" id="invariance">
    <div class="sec-head">
      <span class="sec-tag">Invariance</span>
      <h2>Starting on the path</h2>
      <p>At hover with the payload on the path, the pendulum vertical and the quadrotor level, every transverse
      coordinate is zero, so the state lies on the path-following manifold whatever the heading. The paper states that
      from such a start the transient keeps the payload on the path. In this run the payload starts at rest on the
      circle and the tangential input accelerates it to \\(v_d\\). The transverse coordinates stay at zero in continuous
      time, and in the sampled Drake run the payload never leaves the circle by more than
      {f(1e3 * ON["d_max"], ".2f")}&nbsp;mm.</p>
    </div>
{figure("figs/W_distance.png", f"<b>Distance of the payload to the path and the speed error</b> for the start of the paper and for the hover start on the path, on a logarithmic scale. The sharp dips are the instants at which a signed error passes through zero. From the hover start the distance stays below {f(1e3 * ON['d_max'], '.2f')} mm for the whole run, which is the residual set by the rotational inertia of the spherical payload, while the speed error decays from {f(P['v_d'], '.1f')} rad/s and is below 1 % of \\(v_d\\) after {f(ON['t_speed_err_below_1pct'], '.1f')} s.")}
    <div class="stats">
      <div class="stat"><div class="n">{f(1e3 * ON["d_max"], ".2f")} mm</div>
        <div class="k">largest distance of the payload to the path over the run from the hover start</div></div>
      <div class="stat"><div class="n">{rng(ON["thrust_range"], ".1f")} N</div>
        <div class="k">thrust range from the hover start, against {f(hover, ".1f")}&nbsp;N of weight</div></div>
      <div class="stat"><div class="n">{f(max(ON["tau_peak"]), ".2f")} Nm</div>
        <div class="k">largest applied torque from the hover start, against {f(max(D["tau_peak"]), ".1f")}&nbsp;Nm from the start of the paper</div></div>
      <div class="stat"><div class="n">{f(ON["yaw_rate_max"], ".2f")} rad/s</div>
        <div class="k">largest yaw rate from the hover start</div></div>
    </div>
  </section>
'''

    rows = [
        ("Payload within 1 cm of the path after [s]", f(Q["t_d_below_1cm"]), f(D["t_d_below_1cm"]), f(PM["t_d_below_1cm"]) if PM else "n/a", f(H4["t_d_below_1cm"]) if H4 else "n/a"),
        ("Distance to the path after 10 s [mm]", "&lt; " + f(1e3 * Q["d_after_10s_max"], ".1f"), "&lt; " + f(1e3 * D["d_after_10s_max"], ".1f"), "&lt; " + f(1e3 * PM["d_after_10s_max"], ".1f") if PM else "n/a", "&lt; " + f(1e3 * H4["d_after_10s_max"], ".1f") if H4 else "n/a"),
        ("Distance to the path at 20 s [mm]", sig(1e3 * Q["d_final"], 2), f(1e3 * D["d_final"], ".2f"), sig(1e3 * PM["d_final"], 2) if PM else "n/a", f(1e3 * H4["d_final"], ".2f") if H4 else "n/a"),
        ("Speed error after 10 s [rad/s]", "&lt; " + sig(Q["speed_err_after_10s_max"], 2), "&lt; " + sig(D["speed_err_after_10s_max"], 2), "&lt; " + sig(PM["speed_err_after_10s_max"], 2) if PM else "n/a", "&lt; " + sig(H4["speed_err_after_10s_max"], 2) if H4 else "n/a"),
        ("Yaw rate after 10 s [rad/s]", f(Q["yaw_rate_mean_after_10s"]), f(D["yaw_rate_mean_after_10s"]), f(PM["yaw_rate_mean_after_10s"]) if PM else "n/a", f(H4["yaw_rate_mean_after_10s"]) if H4 else "n/a"),
        ("Thrust after 5 s [N]", rng(Q["thrust_after_5s"]), rng(D["thrust_after_5s"]), rng(PM["thrust_after_5s"]) if PM else "n/a", rng(H4["thrust_after_5s"]) if H4 else "n/a"),
        "Against the port",
        ("Transverse difference after 10 s [mm]", "", f(DvP["transverse_after_10s_mm"]), f(PMvP["transverse_after_10s_mm"]) if PMvP else "n/a", f(H4vP["transverse_after_10s_mm"]) if H4vP else "n/a"),
        ("Offset along the circle at 20 s [mm]", "", f(DvP["arc_final_mm"], ".0f"), f(PMvP["arc_final_mm"], ".0f") if PMvP else "n/a", f(H4vP["arc_final_mm"], ".0f") if H4vP else "n/a"),
        ("Thrust difference after 5 s [N]", "", f(DvP["du_after_5s"], ".3f"), f(PMvP["du_after_5s"], ".3f") if PMvP else "n/a", f(H4vP["du_after_5s"], ".3f") if H4vP else "n/a"),
        ("Torque difference after 5 s [Nm]", "", sig(DvP["dtau_after_5s"], 2), sig(PMvP["dtau_after_5s"], 2) if PMvP else "n/a", sig(H4vP["dtau_after_5s"], 2) if H4vP else "n/a"),
        ("Mean speed difference after 10 s [rad/s]", "", sig(DvP["speed_bias_after_10s"], 2), sig(PMvP["speed_bias_after_10s"], 2) if PMvP else "n/a", sig(H4vP["speed_bias_after_10s"], 2) if H4vP else "n/a"),
    ]
    mat = []
    if "default" in PvM:
        mat.append(("Port against MATLAB, both with the default tolerances of ode45: largest payload position difference", f"{f(PvM['default']['max_dp_mm'], '.0f')} mm"))
    if "tight" in PvM:
        mat.append(("Port against MATLAB, both with tight tolerances: largest payload position difference", f"{f(PvM['tight']['max_dp_mm'], '.3f')} mm"))
        mat.append(("Port against MATLAB, tight tolerances: largest thrust difference", f"{sig(PvM['tight']['max_du'], 2)} N"))
    implementations = f'''
  <section class="sec" id="implementations">
    <div class="sec-head">
      <span class="sec-tag">Implementations</span>
      <h2>Three implementations of the closed loop</h2>
      <p>The MATLAB code of the paper integrates the coordinate-free model (1) with ode45 and evaluates the controller
      from expressions generated by Maple. The Python port evaluates the same expressions, translated line by line, on
      the same model. At the initial condition and at 59 random states its thrust, torques, output chains and vector
      field agree with MATLAB to a relative {sig(G["matlab_states"], 1)}, and the two closed-loop runs with tight tolerances
      agree to {sig(G["matlab_tight_dp"], 1)}&nbsp;m in the payload position. A verification gate checks the expressions against the closed
      loop itself: the time derivative of every term of the two sixth-order chains equals the next term, and at the top
      of each chain it equals the commanded \\(\\nu_1\\) or \\(\\nu_3\\), which tests the drift terms and the matrix
      \\(\\tilde A\\) together. The Drake run replaces the model by a multibody plant: a free rigid body for the quadrotor and
      one rigid body for the pendulum, joined by a universal joint at the center of mass of the quadrotor, with the
      controller sampled at {P["ctrl_hz"]}&nbsp;Hz. A second gate checks that the accelerations of this plant equal those
      of the model (1) at random states and inputs to {sig(G["plant"], 1)} relative, with a point-mass payload.</p>
    </div>
{figure("figs/W_compare.png", f"<b>Drake against the port.</b> Left: the payload position difference, which is an offset along the circle. Middle: the thrust difference. Right: the distance to the path of both runs, with a dip where the Drake run crosses the path before settling on its residual. The offset along the circle comes from the zero-order hold: the body spins at {f(D['yaw_rate_mean_after_10s'], '.1f')} rad/s while the command is held, which biases the speed along the path by {sig(abs(DvP['speed_bias_after_10s']), 2)} rad/s at 1 kHz and {sig(abs(H4vP['speed_bias_after_10s']), 2) if H4vP else 'n/a'} rad/s at 400 Hz, measured as the mean difference of \\(\\dot h_3\\) after 10 s. The residual distance to the path of {f(1e3 * D['d_final'], '.2f')} mm comes from the payload being a sphere: with a point mass the same run settles to {sig(1e3 * PM['d_final'], 2) if PM else 'n/a'} mm.")}
{table(["", "Port, point mass", "Drake, 1 kHz", "Drake, point mass", "Drake, 400 Hz"], rows, "The port integrates the coordinate-free model with a tolerance of 10<sup>-9</sup>. The Drake plant is integrated with a third-order Runge-Kutta scheme at 4 kHz. The spherical payload adds a rotational inertia of " + sig(P["payload_inertia"], 2) + " kg m<sup>2</sup> to the " + f(P["pendulum_inertia"], ".1f") + " kg m<sup>2</sup> of the point mass about the pivot, and the universal joint lets it swing but not spin about the rod.")}
    <div style="height:1.4rem"></div>
{table(["Verification gates", "Value"], mat + [("Controller, torques and vector field at 60 states against MATLAB", "relative difference " + sig(G["matlab_states"], 1)), ("Chain gate, largest relative mismatch of a derivative along the closed loop", sig(G["chains"], 1)), ("Plant gate, accelerations of the Drake plant against the model (1)", "relative difference " + sig(G["plant"], 1))])}
  </section>
'''

    PARAMS = [
        ("Slung payload system", [
            ("Quadrotor mass and inertia", f"m<sub>q</sub> = {f(P['m_q'], '.0f')} kg, J = diag({', '.join(f(v, '.2f') for v in P['J'])}) kg m<sup>2</sup>"),
            ("Payload and rod", f"m<sub>p</sub> = {f(P['m_p'], '.1f')} kg at L = {f(P['L'], '.0f')} m from the pivot on a massless rod. In Drake the payload is a sphere of radius {f(P['payload_radius'], '.2f')} m"),
            ("Rotor distance and drag ratio", f"&#8467; = {f(P['ell'], '.1f')} m, c<sub>d</sub> = {f(P['c_d'], '.2f')}, so c<sub>1</sub> = {f(P['c1'], '.1f')} m and c<sub>2</sub> = {f(P['c2'], '.0f')}"),
            ("Failure", "the rotor on the +b<sub>2</sub> arm is lost, &tau;<sub>1</sub> = c<sub>1</sub> u<sub>t</sub> &minus; c<sub>2</sub> &tau;<sub>3</sub>"),
            ("Damping", f"k<sub>&Omega;</sub> = {f(P['k_Om'], '.0f')} s<sup>-1</sup> on every axis, applied torque &tau;<sub>a</sub> = &tau; &minus; k<sub>&Omega;</sub> J &Omega;"),
            ("Gravity", f"{f(P['g'])} m/s<sup>2</sup>, north-east-down frame"),
        ]),
        ("Path and controller", [
            ("Path", f"x<sup>2</sup> + y<sup>2</sup> = {f(P['d_star'], '.0f')}<sup>2</sup>, z = {f(P['r_star'], '.0f')} m, path coordinate h<sub>3</sub> = atan2(p<sub>2</sub>, p<sub>1</sub>)"),
            ("Speed reference", f"v<sub>d</sub> = {f(P['v_d'], '.1f')} rad/s ({f(speed_mps, '.0f')} m/s along the circle)"),
            ("Poles of the h<sub>1</sub> chain (order 6)", "{" + ", ".join(f(v, '.1f') for v in P['poles_1']) + "}, K<sub>1</sub> = (" + ", ".join(f(v, '.3f').rstrip('0').rstrip('.') for v in P['K1']) + ")"),
            ("Poles of the h<sub>2</sub> chain (order 2)", "{" + ", ".join(f(v, '.1f') for v in P['poles_2']) + "}, K<sub>2</sub> = (" + ", ".join(f(v, '.1f') for v in P['K2']) + ")"),
            ("Poles of the speed chain of h<sub>3</sub> (order 5)", "{" + ", ".join(f(v, '.1f') for v in P['poles_3']) + "}, K<sub>3</sub> = (" + ", ".join(f(v, '.2f').rstrip('0').rstrip('.') for v in P['K3']) + ")"),
            ("Feedback", "thrust from (17), torques &tau;<sub>2</sub>, &tau;<sub>3</sub> from (26), &tau;<sub>1</sub> from the failure, then the damping"),
        ]),
        ("Start of the paper", [
            ("Payload", f"p(0) = ({', '.join(f(v, '.0f') for v in P['p0'])}) m, v(0) = ({', '.join(f(v, '.1f') for v in P['v0'])}) m/s"),
            ("Pendulum", f"&alpha; = {f(P['alpha0_deg'], '.0f')}&deg;, &beta; = {f(P['beta0_deg'], '.0f')}&deg;, rates ({', '.join(f(v, '.1f') for v in P['gamma0'])}) rad/s"),
            ("Quadrotor", f"&theta; = {f(P['theta0_deg'], '.0f')}&deg;, &phi; = {f(P['phi0_deg'], '.0f')}&deg;, &psi; = {f(P['psi0_deg'], '.0f')}&deg;, &Omega;(0) = ({', '.join(f(v, '.1f') for v in P['Omega0'])}) rad/s"),
            ("Horizon", f"{f(P['T'], '.0f')} s"),
        ]),
        ("Hover start on the path", [
            ("State", f"payload at rest on the circle, pendulum vertical, quadrotor level with zero rates. Every transverse coordinate is zero"),
        ]),
        ("Simulation", [
            ("MATLAB", "coordinate-free model (1), ode45 with its default tolerances, as in the paper"),
            ("Python port", "the same model and expressions, Dormand-Prince 8(5,3) with relative tolerance 10<sup>-9</sup>"),
            ("Drake", f"MultibodyPlant, continuous, third-order Runge-Kutta at {P['plant_hz'] // 1000} kHz, controller at {P['ctrl_hz']} Hz with a zero-order hold on the body-frame thrust and torque"),
            ("Rendering", "VTK render engine of Drake. The airframe is the Skydio X2 mesh of the MuJoCo Menagerie (Apache-2.0), scaled so that the rotor hubs sit at &#8467;, and the airframe, rod and payload are drawn at three times their size"),
        ]),
    ]
    params = "".join(params_table(t, r) for t, r in PARAMS)
    parameters = f'''
  <section class="sec" id="parameters">
    <div class="sec-tag">Parameters</div>
    <div class="sec-head">
      <h2>Every parameter</h2>
      <p>These are the complete settings behind every figure on this page. They are those of Section VI of the paper.</p>
    </div>
    {params}
  </section>
'''

    reproduce = f'''
  <section class="sec" id="reproduce">
    <div class="sec-head">
      <span class="sec-tag">Reproduce</span>
      <h2>Reproducing this</h2>
      <p>The simulation code is in the repository under <code>sim/</code> and the MATLAB code of the paper under
      <code>matlab/</code>. The verification gates run first.</p>
    </div>
<pre><code><span class="c"># set up and verify</span>
git clone {REPO}.git
cd qs-sl/sim
pip install -r requirements.txt
python3 model/test_chains.py
python3 model/compare_matlab.py
python3 drake/verify_plant.py

<span class="c"># the runs behind every number and figure</span>
python3 model/simulate_port.py --tight
python3 drake/sim_drake.py
python3 drake/sim_drake.py --start on-path --tag _onpath
python3 drake/sim_drake.py --point-mass --tag _pointmass
python3 drake/sim_drake.py --hz 400 --tag _400hz

<span class="c"># the numbers and the figures</span>
python3 metrics.py
python3 paper_figures.py
python3 paper_figures.py --web

<span class="c"># animations (Drake VTK renderer and ffmpeg), then the page</span>
python3 drake/render_drake.py drake_run --snapshots 0,1,1.5,2,5,6.5,8,10.5 --cam wide
for c in follow side orbit; do python3 drake/render_drake.py drake_run --cam $c; done
python3 drake/render_drake.py drake_run_onpath --cam follow
python3 web/anim_web.py
python3 web/page_numbers.py
python3 web/build_page.py</code></pre>
  </section>

  <section class="sec" id="cite">
    <div class="sec-head">
      <span class="sec-tag">Citation</span>
      <h2>Citation</h2>
    </div>
<pre><code>@misc{{allawati2026quasistatic,
  author       = {{Mohamed Al Lawati and Adeel Akhtar}},
  title        = {{Quasi-Static Fault-Tolerant Feedback Control of a Quadrotor
                  Carrying a Slung Payload under Rotor Failure}},
  howpublished = {{Manuscript}},
  year         = {{2026}}
}}</code></pre>
  </section>

  <footer>
    <div class="rulebar"></div>
    <p>GRaDS Lab, New Jersey Institute of Technology.
      Every figure and number on this page is generated by the scripts above.
      <a href="{REPO}">Repository</a>.</p>
  </footer>

  </main>
</div>
<script src="assets/js/site.js"></script>
</body>
</html>
'''
    return head + gallery + scenario + outputs + configuration + invariance + implementations + parameters + reproduce


def write():
    doc = build()
    os.makedirs(os.path.join(SITE, 'figs'), exist_ok=True)
    manifest = {"figs": [], "anim": N.get("anim_mb", {})}
    for name in FIGS:
        src = os.path.join(SIM, 'results', 'figs', name + '.png')
        if os.path.exists(src):
            shutil.copy(src, os.path.join(SITE, 'figs', name + '.png'))
            manifest["figs"].append({"file": f"figs/{name}.png", "MB": round(os.path.getsize(src) / 1e6, 2)})
    out = os.path.join(SITE, 'index.html')
    with open(out, 'w') as fh:
        fh.write(doc)
    json.dump(manifest, open(os.path.join(SITE, 'manifest.json'), 'w'), indent=1)
    print(f"wrote {out}  ({len(doc):,} characters)")


if __name__ == "__main__":
    write()
