% Reference data of the MATLAB implementation for the Python port.
%   reference_states.csv, reference_outputs.csv   the controller and the vector field at 60 states
%                                                 (the initial condition and 59 random states)
%   reference_sim_default.csv                     the closed loop with the default tolerances of ode45, as main.m
%   reference_sim_tight.csv                       the closed loop with RelTol 1e-9 and AbsTol 1e-11
% Columns of the output files are documented in sim/model/compare_matlab.py.
clc; clear;
l = 0.2; cd = 0.01; c__1 = l/2; c__2 = l/(2*cd); k__Om = 3;
r1 = 5; r2 = -3; vd = 0.8;
m__q = 5; m__p = 0.2; L = 3; g = 9.81; J = diag([0.03,0.03,0.05]); Jinv = inv(J);
IC_0; gains;

%% the controller at 60 states
rng(1);
N = 60; Zs = zeros(N,24); OUT = zeros(N, 24+11);
for i = 1:N
    if i == 1
        z = IC;
    else
        rr = 2 + 7*rand; ang = 2*pi*rand;
        p = [rr*cos(ang); rr*sin(ang); -12 + 11*rand];
        al = deg2rad(-50 + 100*rand); be = deg2rad(-50 + 100*rand);
        th = deg2rad(-50 + 100*rand); ph = deg2rad(-50 + 100*rand); ps = deg2rad(-180 + 360*rand);
        R = eul2rotm([th, ph, ps], "YXZ");
        q = [sin(be); -sin(al)*cos(be); cos(al)*cos(be)];
        v = -1 + 2*rand(3,1);
        gam = -0.5 + rand(2,1);
        W = [cos(be)^2, 0; sin(al)*sin(2*be)/2, cos(al); -cos(al)*sin(2*be)/2, sin(al)];
        om = W*gam;
        OM = -1 + 2*rand(3,1);
        z = [p; q; R(1,:).'; R(2,:).'; R(3,:).'; v; om; OM];
    end
    [dzdt,u__t,tau__1,tau__2,tau__3,h1,h2,h3,h3__dt,ch,ch_dt,tmp] = SLS_quasi(0, z, L,m__q,m__p,g,J,Jinv,K1,K2,K3,c__1,c__2,k__Om,r1,r2,vd);
    Zs(i,:) = z(:).';
    OUT(i,:) = [dzdt(:).', u__t, tau__1, tau__2, tau__3, h1, h2, h3, h3__dt, ch, ch_dt, tmp];
end
writematrix(Zs, 'reference_states.csv');
writematrix(OUT, 'reference_outputs.csv');

%% the closed loop
t = 0:0.01:20;
for mode = 1:2
    if mode == 1
        opts = odeset(); tag = 'default';
    else
        opts = odeset('RelTol',1e-9,'AbsTol',1e-11); tag = 'tight';
    end
    [T,Z] = ode45(@(t,z)SLS_quasi(t,z,L,m__q,m__p,g,J,Jinv,K1,K2,K3,c__1,c__2,k__Om,r1,r2,vd), t, IC, opts);
    n = length(T); OUT = zeros(n, 11);
    for i = 1:n
        [~,u__t,tau__1,tau__2,tau__3,h1,h2,h3,h3__dt,ch,ch_dt,tmp] = SLS_quasi(T(i),Z(i,:),L,m__q,m__p,g,J,Jinv,K1,K2,K3,c__1,c__2,k__Om,r1,r2,vd);
        OUT(i,:) = [u__t, tau__1, tau__2, tau__3, h1, h2, h3, h3__dt, ch, ch_dt, tmp];
    end
    writematrix([T(:), Z, OUT], ['reference_sim_' tag '.csv']);
end
