%% Aux inputs
nu__1 = -K1*[h1;...
    h1__dt;...
    h1__ddt;...
    h1__dddt;...
    h1__ddddt;...
    h1__dddddt];

nu__2 = -K2*[h2;h2__dt];

nu__3 = -K3*[h3__dt;...
    h3__ddt;...
    h3__dddt;...
    h3__ddddt;...
    h3__dddddt];

%% Thrust
% u__t = -(L * cos(beta) ^ 3 * cos(alpha) * m__q * gamma__alpha ^ 2 + L * cos(beta) * cos(alpha) * m__q * gamma__beta ^ 2 + ((r2 - p__3) * k__21 - k__22 * v__3 - g) * (m__q + m__p)) * sec(beta) * sec(alpha) / ((cos(alpha) * cos(theta) * cos(phi) + sin(alpha) * sin(phi)) * cos(beta) + sin(beta) * sin(theta) * cos(phi));
  u__t = -sec(alpha) * sec(beta) * (L * cos(alpha) * cos(beta) ^ 3 * m__q * gamma__alpha ^ 2 + L * cos(alpha) * cos(beta) * m__q * gamma__beta ^ 2 + (m__q + m__p) * ((r2 - p__3) * k__21 - k__22 * v__3 - g)) / ((cos(alpha) * cos(phi) * cos(theta) + sin(phi) * sin(alpha)) * cos(beta) + sin(beta) * sin(theta) * cos(phi));

%% Torques


sigma_1 = (g - nu__2)*...
    (L*m__q*cos(alpha)*cos(beta)*(cos(beta) ^ 2*gamma__alpha^2 + gamma__beta^2)- ... 
    M*(g-nu__2) ) ...
    /...
    (m__q*L*(cos(alpha)*cos(beta))^3*(dot(q,R(:,3)))^2);


Atilde = sigma_1*...
    [cross(dh_1,dh_2);cross(dh_3,dh_2)]*...
    [cross(cross(R(:,2),q), q)/J__2, -c__2 * cross(cross(R(:,1),q), q)/J__1];



% Atilde = [0.2e1 * (L * cos(beta) ^ 3 * m__q * gamma__alpha ^ 2 + L * cos(beta) * m__q * gamma__beta ^ 2 - (m__q + m__p) * sec(alpha) * (k__21 * (p__3 - r2) + k__22 * v__3 + g)) * (k__21 * (p__3 - r2) + k__22 * v__3 + g) * sec(alpha) ^ 2 * sec(beta) ^ 3 * ((p__1 * cos(phi) * cos(psi) * cos(alpha) ^ 2 + p__1 * sin(alpha) * (sin(theta) * sin(psi) + cos(theta) * sin(phi) * cos(psi)) * cos(alpha) - p__2 * (sin(theta) * sin(phi) * cos(psi) - cos(theta) * sin(psi))) * cos(beta) ^ 2 + sin(beta) * (p__2 * (sin(theta) * sin(psi) + cos(theta) * sin(phi) * cos(psi)) * cos(alpha) + p__1 * sin(alpha) * (sin(theta) * sin(phi) * cos(psi) - cos(theta) * sin(psi)) - p__2 * cos(phi) * cos(psi) * sin(alpha)) * cos(beta) + cos(phi) * cos(psi) * sin(beta) ^ 2 * p__1) / m__q / L / J__2 / ((((cos(theta) ^ 2 + 0.1e1) * cos(alpha) ^ 2 - 0.2e1) * cos(phi) ^ 2 + 0.2e1 * cos(alpha) * cos(theta) * cos(phi) * sin(alpha) * sin(phi) + sin(alpha) ^ 2) * cos(beta) ^ 2 + 0.2e1 * cos(phi) * sin(beta) * sin(theta) * (cos(alpha) * cos(phi) * cos(theta) + sin(phi) * sin(alpha)) * cos(beta) + (0.1e1 - sin(beta) ^ 2 * cos(theta) ^ 2) * cos(phi) ^ 2) -0.2e1 * ((p__1 * cos(phi) * sin(psi) * cos(alpha) ^ 2 - p__1 * sin(alpha) * (sin(theta) * cos(psi) - cos(theta) * sin(phi) * sin(psi)) * cos(alpha) - p__2 * (cos(theta) * cos(psi) + sin(theta) * sin(phi) * sin(psi))) * cos(beta) ^ 2 + (p__1 * sin(alpha) * (cos(theta) * cos(psi) + sin(theta) * sin(phi) * sin(psi)) - p__2 * cos(phi) * sin(psi) * sin(alpha) - p__2 * (sin(theta) * cos(psi) - cos(theta) * sin(phi) * sin(psi)) * cos(alpha)) * sin(beta) * cos(beta) + sin(psi) * cos(phi) * sin(beta) ^ 2 * p__1) * sec(beta) ^ 3 * c__2 * sec(alpha) ^ 2 * (k__21 * (p__3 - r2) + k__22 * v__3 + g) * (L * cos(beta) ^ 3 * m__q * gamma__alpha ^ 2 + L * cos(beta) * m__q * gamma__beta ^ 2 - (m__q + m__p) * sec(alpha) * (k__21 * (p__3 - r2) + k__22 * v__3 + g)) / L / m__q / J__1 / ((((cos(theta) ^ 2 + 0.1e1) * cos(alpha) ^ 2 - 0.2e1) * cos(phi) ^ 2 + 0.2e1 * cos(alpha) * cos(theta) * cos(phi) * sin(alpha) * sin(phi) + sin(alpha) ^ 2) * cos(beta) ^ 2 + 0.2e1 * cos(phi) * sin(beta) * sin(theta) * (cos(alpha) * cos(phi) * cos(theta) + sin(phi) * sin(alpha)) * cos(beta) + (0.1e1 - sin(beta) ^ 2 * cos(theta) ^ 2) * cos(phi) ^ 2); -(L * cos(beta) ^ 3 * m__q * gamma__alpha ^ 2 + L * cos(beta) * m__q * gamma__beta ^ 2 - (m__q + m__p) * sec(alpha) * (k__21 * (p__3 - r2) + k__22 * v__3 + g)) * ((p__2 * cos(phi) * cos(psi) * cos(alpha) ^ 2 + p__2 * sin(alpha) * (sin(theta) * sin(psi) + cos(theta) * sin(phi) * cos(psi)) * cos(alpha) + p__1 * (sin(theta) * sin(phi) * cos(psi) - cos(theta) * sin(psi))) * cos(beta) ^ 2 + (p__1 * cos(phi) * cos(psi) * sin(alpha) + p__2 * sin(alpha) * (sin(theta) * sin(phi) * cos(psi) - cos(theta) * sin(psi)) - p__1 * (sin(theta) * sin(psi) + cos(theta) * sin(phi) * cos(psi)) * cos(alpha)) * sin(beta) * cos(beta) + cos(phi) * cos(psi) * sin(beta) ^ 2 * p__2) * (k__21 * (p__3 - r2) + k__22 * v__3 + g) * sec(alpha) ^ 2 * sec(beta) ^ 3 / m__q / (p__1 ^ 2 + p__2 ^ 2) / L / J__2 / ((((cos(theta) ^ 2 + 0.1e1) * cos(alpha) ^ 2 - 0.2e1) * cos(phi) ^ 2 + 0.2e1 * cos(alpha) * cos(theta) * cos(phi) * sin(alpha) * sin(phi) + sin(alpha) ^ 2) * cos(beta) ^ 2 + 0.2e1 * cos(phi) * sin(beta) * sin(theta) * (cos(alpha) * cos(phi) * cos(theta) + sin(phi) * sin(alpha)) * cos(beta) + (0.1e1 - sin(beta) ^ 2 * cos(theta) ^ 2) * cos(phi) ^ 2) sec(beta) ^ 3 * c__2 * ((p__2 * cos(phi) * sin(psi) * cos(alpha) ^ 2 + p__1 * (cos(theta) * cos(psi) + sin(theta) * sin(phi) * sin(psi)) - p__2 * sin(alpha) * (sin(theta) * cos(psi) - cos(theta) * sin(phi) * sin(psi)) * cos(alpha)) * cos(beta) ^ 2 + (p__1 * cos(phi) * sin(psi) * sin(alpha) + p__1 * (sin(theta) * cos(psi) - cos(theta) * sin(phi) * sin(psi)) * cos(alpha) + p__2 * sin(alpha) * (cos(theta) * cos(psi) + sin(theta) * sin(phi) * sin(psi))) * sin(beta) * cos(beta) + cos(phi) * sin(beta) ^ 2 * sin(psi) * p__2) * sec(alpha) ^ 2 * (k__21 * (p__3 - r2) + k__22 * v__3 + g) * (L * cos(beta) ^ 3 * m__q * gamma__alpha ^ 2 + L * cos(beta) * m__q * gamma__beta ^ 2 - (m__q + m__p) * sec(alpha) * (k__21 * (p__3 - r2) + k__22 * v__3 + g)) / (p__1 ^ 2 + p__2 ^ 2) / L / m__q / J__1 / ((((cos(theta) ^ 2 + 0.1e1) * cos(alpha) ^ 2 - 0.2e1) * cos(phi) ^ 2 + 0.2e1 * cos(alpha) * cos(theta) * cos(phi) * sin(alpha) * sin(phi) + sin(alpha) ^ 2) * cos(beta) ^ 2 + 0.2e1 * cos(phi) * sin(beta) * sin(theta) * (cos(alpha) * cos(phi) * cos(theta) + sin(phi) * sin(alpha)) * cos(beta) + (0.1e1 - sin(beta) ^ 2 * cos(theta) ^ 2) * cos(phi) ^ 2);];

det_Atilde = det(Atilde);
tmp = -c__2 * u__t^2 * dot(dh_2,q)^2 * (g-nu__2)^2 * dot(cross(dh_1,dh_2),dh_3) / (J__1*J__2*(L*m__q)^2 * dot(R(:,3),q) * (cos(alpha)*cos(beta))^5) ...
    -det_Atilde;

% qR3 = dot(q,R(:,3));
% 
% 
% 
% numer_Atilde = 0.2e1 * sec(beta) ^ 4 * (L * cos(alpha) * cos(beta) * m__q * (cos(beta) ^ 2 * gamma__alpha ^ 2 + gamma__beta ^ 2) - (k__21 * (p__3 - r2) + k__22 * v__3 + g) * (m__q + m__p)) ^ 2 * sec(alpha) ^ 5 * (cos(alpha) * cos(theta) * cos(phi) + cos(phi) * sin(theta) * tan(beta) + sin(alpha) * sin(phi)) * (k__21 * (p__3 - r2) + k__22 * v__3 + g) ^ 2 * c__2;
% 
% denom_Atilde = J__1*J__2*m__q^2*L^2 * (qR3)^4;



tau23 = Atilde\([nu__1;nu__3;] - [b1;b3;]);
tau__2 = tau23(1);
tau__3 = tau23(2);

tau__1 = c__1*u__t - c__2*tau__3;

tau = [tau__1;tau__2;tau__3];
tau = tau - J*k__Om*[Omega__1;Omega__2;Omega__3];
tau__1 = tau(1);  tau__2 = tau(2);  tau__3 = tau(3);

