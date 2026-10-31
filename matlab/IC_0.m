p0 = [r1*sind(0)-2;r1*cosd(0)+3;r2-8];

% p0 = [-r1;0;r2];
% p0 = [r1;0;r2];
% p0 = [0;r1;r2];
% p0 = [0;-9;r2-8];

alpha0 = -30*pi/180*1;
beta0 = 20*pi/180*1;

tht0 = 45*pi/180*1;
phi0 = 20*pi/180*1;
psi0 = -60*pi/180*1;

R0 = eul2rotm([tht0, phi0, psi0], "YXZ");
q0 = [ sin(beta0);
     -sin(alpha0)*cos(beta0);
      cos(alpha0)*cos(beta0) ];

v0 = [-0.2;0.1;0.5]*1;

gamma__alpha0 = -0.2*1;
gamma__beta0 = 0.1*1;
W0 = [ cos(beta0)^2,                       0;
      sin(alpha0)*sin(2*beta0)/2,          cos(alpha0);
     -cos(alpha0)*sin(2*beta0)/2,          sin(alpha0) ];

om0 = W0*[gamma__alpha0;gamma__beta0];

OM0 = [-0.1;0.3;.2]*1;

IC = [p0;...
      q0;...
      R0(1,:).';
      R0(2,:).';
      R0(3,:).';
      v0;...
      om0;...
      OM0];