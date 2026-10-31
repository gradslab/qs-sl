clc; clear; close all;

savePlots = false;
%% Fault parameters
l = 0.2;
cd = 0.01;

c__1 = l/2;
c__2 = l/(2*cd);


k__Om = 3;

%% Path Parameters
r1 = 5;
r2 = -3;
vd = 0.8;

%% Inertial Parameters
m__q = 5;
m__p = 0.2;

L = 3;

g = 9.81;
J =  diag([0.03,0.03,0.05])*1;
Jinv = inv(J);

%% Initilaization
t = 0:0.01:20;
IC_0;

%% Gains
gains;

%% Animation/snapshots parameters
d_arm = 1;
d_prop = d_arm/2;
d_pend = L;
THT = -45;

%% Solve
tic;
[T,Z]=ode45(@(t,z)SLS_quasi(t,z,L,m__q,m__p,g,J,Jinv,K1,K2,K3,c__1,c__2,k__Om,r1,r2,vd),t,IC);
toc;


%% Getting stuff
for i = 1 :length(T)
    [zdot(:,i),u__t(i,1),tau__1(i,1),tau__2(i,1),tau__3(i,1),h1(i,1),h2(i,1),h3(i,1),h3__dt(i,1),ch(i,1),ch_dt(i,1),tmp(i,1)] = SLS_quasi(T(i),Z(i,:),L,m__q,m__p,g,J,Jinv,K1,K2,K3,c__1,c__2,k__Om,r1,r2,vd);
end


%% Relabeling
p1 = Z(:,1); p2 = Z(:,2); p3 = Z(:,3);
q = Z(:,4:6) ./ vecnorm(Z(:,4:6),2,2);      % normalized q
beta  = atan2(q(:,1), hypot(q(:,2), q(:,3)));
alpha = atan2(-q(:,2), q(:,3));

om = Z(:,19:21);

N = length(T);
R_s = zeros(3,3,N);  r_s = zeros(3,N);
theta = zeros(N,1); phi = zeros(N,1); psi = zeros(N,1);
for i = 1:N
    Ri = reshape(Z(i,7:15), 3, 3).';          % row-wise packing
    [U,~,V] = svd(Ri);
    Ri = U*diag([1 1 det(U*V.')])*V.';
    R_s(:,:,i) = Ri;
    r_s(:,i)   = [p1(i); p2(i); p3(i)];
    eta = rotm2eul(Ri, "YXZ");
    theta(i) = eta(1); phi(i) = eta(2); psi(i) = eta(3);
end
psi = unwrap(psi); 
%% Plot: R
fR = figure;
for i = 1:3
    for j = 1:3
        subplot(3,3,3*(i-1)+j); hold on; grid on;
        plot(T, squeeze(R_s(i,j,:)))
        xlabel('Time(s)')
        ylabel(sprintf('$R_{%d%d}$', i, j), 'Interpreter', 'latex')
        ylim([-1.05 1.05])
    end
end
%% Plot:  q and om


fqom = figure;
qlabels = {'$q_1$','$q_2$','$q_3$'};
wlabels = {'$\omega_1$ (rad/s)','$\omega_2$ (rad/s)','$\omega_3$ (rad/s)'};

for i = 1:3
    % top row: q
    subplot(2,3,i); hold on; grid on;
    plot(T, q(:,i))
    xlabel('Time(s)')
    ylabel(qlabels{i}, 'Interpreter', 'latex')
    ylim([-1.05 1.05])

    % bottom row: omega
    subplot(2,3,3+i); hold on; grid on;
    plot(T, om(:,i))
    xlabel('Time(s)')
    ylabel(wlabels{i}, 'Interpreter', 'latex')
end
%% Plot: config in coords
fconf = figure;

subplot(2,4,1); hold on; grid on;
plot(T,p1)
xlabel('Time(s)')
ylabel('$p_1$')

subplot(2,4,2); hold on; grid on;
plot(T,p2)
xlabel('Time(s)')
ylabel('$p_2$')

subplot(2,4,3); hold on; grid on;
plot(T,p3)
xlabel('Time(s)')
ylabel('$p_3$')

subplot(2,4,4); hold on; grid on;
plot(T,alpha)
xlabel('Time(s)')
ylabel('$\alpha$')

subplot(2,4,5); hold on; grid on;
plot(T,beta)
xlabel('Time(s)')
ylabel('$\beta$')

subplot(2,4,6); hold on; grid on;
plot(T,theta)
xlabel('Time(s)')
ylabel('$\theta$')

subplot(2,4,7); hold on; grid on;
plot(T,phi)
xlabel('Time(s)')
ylabel('$\phi$')

subplot(2,4,8); hold on; grid on;
plot(T,psi)
xlabel('Time(s)')
ylabel('$\psi$')

%% Plot: input
fin = figure;

ax=subplot(2,2,1); hold on; grid on;
plot(T,u__t)
xlabel('Time(s)')
ylabel('$u_t$(N)')
%addInset(ax, T, u__t, [25 30], [0.35 0.30 0.58 0.45]);

ax = subplot(2,2,2); hold on; grid on;
plot(T,tau__1)
xlabel('Time(s)')
ylabel('$\tau_1$(N.m)')
ylim([-50e-3, 50e-3])
% addInset(ax, T, tau__1, [0 .02], [0.35 0.30 0.58 0.45]);

ax = subplot(2,2,3); hold on; grid on;
plot(T,tau__2)
xlabel('Time(s)')
ylabel('$\tau_2$(N.m)')
ylim([-50e-3, 50e-3])
% addInset(ax, T, tau__2, [0 5], [0.35 0.30 0.58 0.45]);

ax = subplot(2,2,4); hold on; grid on;
plot(T,tau__3)
xlabel('Time(s)')
ylabel('$\tau_3$(N.m)')
ylim([-50e-3, 50e-3])
% addInset(ax, T, tau__3, [0 5], [0.35 0.30 0.58 0.45]);

%% Plot: output
fout = figure;

subplot(2,2,1); hold on; grid on;
plot(T,h1)
xlabel('Time(s)')
ylabel('$h_1$')

subplot(2,2,2); hold on; grid on;
plot(T,h2)
xlabel('Time(s)')
ylabel('$h_2$')

subplot(2,2,3); hold on; grid on;
plot(T,unwrap(h3))
xlabel('Time(s)')
ylabel('${h}_3$')

subplot(2,2,4); hold on; grid on;
plot(T,h3__dt)
xlabel('Time(s)')
ylabel('$\dot{h}_3$')

%% Plot: tmp
figure; hold on; grid on;
plot(T,tmp)
xlabel('Time(s)')
ylabel('tmp')

%% Plot: ch and ch_dt
figure; hold on; grid on;
plot(T,ch_dt)
plot(T(2:end),diff(ch)./diff(T))

%% Snaptost
fsnap=snapshots(T,r_s,R_s,alpha,beta, ...
    d_arm, d_prop, d_pend, THT,r1,r2,...
    [1, 101,151, 201,501,651,801,1051],true);

%% Save for publications
if savePlots
    fconf.Renderer = 'painters';
    theme(fconf, "light");
    exportgraphics(fconf, 'conf.pdf', 'BackgroundColor','white', 'ContentType','vector')
    
    fR.Renderer = 'painters';
    theme(fR, "light");
    exportgraphics(fR, 'R.pdf', 'BackgroundColor','white', 'ContentType','vector')
    
    fqom.Renderer = 'painters';
    theme(fqom, "light");
    exportgraphics(fqom, 'q_om.pdf', 'BackgroundColor','white', 'ContentType','vector')


    fout.Renderer = 'painters';
    theme(fout, "light");
    exportgraphics(fout, 'output.pdf', 'BackgroundColor','white', 'ContentType','vector')


    fin.Renderer = 'painters';
    theme(fin, "light");
    exportgraphics(fin, 'input.pdf', 'BackgroundColor','white', 'ContentType','vector')

    %%
    fsnap.Renderer = 'painters';
    exportgraphics(fsnap, 'snapshot.pdf', 'BackgroundColor','white', 'ContentType','vector')
end

%% Aniamte
% animate(0,'file',T,r_s,R_s,alpha,beta, ...
%     d_arm, d_prop, d_pend, THT,r1,r2)