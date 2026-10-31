function [dzdt, u__t,tau__1,tau__2,tau__3, h1,h2,h3,h3__dt,ch,ch_dt,tmp] = SLS_quasi(t,z,L,m__q,m__p,g,J,Jinv,K1,K2,K3,c__1,c__2,k__Om,r1,r2,vd)

p__1 = z(1); p__2  = z(2); p__3  = z(3); 

q__1 = z(4); q__2 = z(5); q__3 = z(6);
q = [q__1;q__2;q__3];

% rotational states
r11 = z(7);  r12 = z(8); r13 = z(9);
r21 = z(10); r22 = z(11);r23 = z(12);
r31 = z(13); r32 = z(14);r33 = z(15);

v__1 = z(16); v__2 = z(17); v__3 = z(18);
v = [v__1;v__2;v__3];

omega__1 = z(19); omega__2 = z(20); omega__3 = z(21);
om = [omega__1;omega__2;omega__3];

Omega__1     = z(22);
Omega__2     = z(23);
Omega__3     = z(24);

OM = [Omega__1;Omega__2;Omega__3];

%% Projecting q on S2
[U,S,V] = svd([q(1);q(2);q(3)]);
q_matriax  = U*V.';
q = q_matriax(:,1); %q corrected
q__1 = q(1); q__2 = q(2); q__3 = q(3);
% q__3 = sign(q__3)*sqrt(1-q__1^2-q__2^2);
%% Projecting R on SO3
[U,S,V] = svd([r11, r12, r13; ...
    r21, r22, r23; ...
    r31, r32, r33]);
R  = U*V.';

%% Computing 2-1-3 Euler angles for UAV + alpha,beta for q
eta = rotm2eul(R,"YXZ");

theta = eta(1);
phi = eta(2);
psi = eta(3);

beta  = atan2(q(1), hypot(q(2), q(3)));
alpha = atan2(-q(2), q(3));

%% Converting omega to gamma__alpha, gamma__beta
W = [ cos(beta)^2,                       0;
      sin(alpha)*sin(2*beta)/2,          cos(alpha);
     -cos(alpha)*sin(2*beta)/2,          sin(alpha) ];

gamma = (W.'*W) \ (W.'*om);

gamma__alpha = gamma(1);
gamma__beta = gamma(2);

%% Useful parameters
e1 = [1;0;0];
e2 = [0;1;0];
e3 = [0;0;1];
M = m__q + m__p;



J__1 = J(1,1); J__2 = J(2,2); J__3 = J(3,3);
k__21 = K2(1); k__22 = K2(2);

%% Controller

h1_;
h2_;
h3_;

b_;

controller;


%% Dynamics

%no fault dynamnics
% f = [v__1 v__2 v__3 gamma__alpha gamma__beta sec(phi) * (Omega__1 * sin(psi) + Omega__2 * cos(psi)) cos(psi) * Omega__1 - sin(psi) * Omega__2 sin(psi) * tan(phi) * Omega__1 + cos(psi) * tan(phi) * Omega__2 + Omega__3 -L * sin(beta) * m__q * (cos(beta) ^ 2 * gamma__alpha ^ 2 + gamma__beta ^ 2) / (m__q + m__p) cos(beta) * L * sin(alpha) * m__q * (cos(beta) ^ 2 * gamma__alpha ^ 2 + gamma__beta ^ 2) / (m__q + m__p) ((m__q + m__p) * g - L * cos(alpha) * cos(beta) * m__q * (cos(beta) ^ 2 * gamma__alpha ^ 2 + gamma__beta ^ 2)) / (m__q + m__p) 0.2e1 * sin(beta) * gamma__alpha * gamma__beta / cos(beta) -sin(beta) * cos(beta) * gamma__alpha ^ 2 -Omega__2 * Omega__3 * (J__2 - J__3) / J__1 Omega__1 * Omega__3 * (J__1 - J__3) / J__2 -Omega__1 * Omega__2 * (J__1 - J__2) / J__3];
% g1 = [0 0 0 0 0 0 0 0 -sin(beta) * (cos(beta) * cos(alpha) * cos(theta) * cos(phi) + sin(beta) * sin(theta) * cos(phi) + cos(beta) * sin(alpha) * sin(phi)) / (m__q + m__p) cos(beta) * sin(alpha) * (cos(beta) * cos(alpha) * cos(theta) * cos(phi) + sin(beta) * sin(theta) * cos(phi) + cos(beta) * sin(alpha) * sin(phi)) / (m__q + m__p) -cos(beta) * cos(alpha) * (cos(beta) * cos(alpha) * cos(theta) * cos(phi) + sin(beta) * sin(theta) * cos(phi) + cos(beta) * sin(alpha) * sin(phi)) / (m__q + m__p) (cos(alpha) * sin(phi) - cos(phi) * cos(theta) * sin(alpha)) * sec(beta) / L / m__q (cos(beta) * cos(phi) * sin(theta) - (cos(alpha) * cos(theta) * cos(phi) + sin(alpha) * sin(phi)) * sin(beta)) / L / m__q 0 0 0];
% g2 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0.1e1 / J__1 0 0];
% g3 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0.1e1 / J__2 0];
% g4 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0.1e1 / J__3];
% 
% dzdt(1:16,1) = f + g1*u__t + g2*tau__1 + g3*tau__2 + g4*tau__3;
R_dt = R*SSS(OM);

dzdt(1:3,1)   = v;
dzdt(4:6,1)   = cross(om,q);
dzdt(7:15,1) = reshape(R_dt.', 9, 1);

dzdt(16:18,1)   = - m__q*L/M * dot(om,om) * q + g*e3  - u__t*dot(R*e3,q)*q/M;
dzdt(19:21,1) = u__t/(m__q*L)*cross(q,R*e3);
dzdt(22:24,1) = Jinv*(tau - cross(OM,J*OM));
%% Checking stuff...
ch = h3__ddddt;
ch_dt = h3__dddddt;

