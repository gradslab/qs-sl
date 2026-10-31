function f = snapshots(T,p,R_rot,alpha,beta, d_arm, d_prop, d_pend, THT,r1,r2, indx, light_mode)
if nargin < 13
    light_mode = false;
end
if light_mode
    pend_color    = [0.3 0.3 0.3];   % was [0.9 0.9 0.9] - invisible on white
    payload_color = [0 0 0];         % was [1 1 1]       - invisible on white
else
    pend_color    = [0.9 0.9 0.9];
    payload_color = [1 1 1];
end
R_THT = [cos(THT), -sin(THT), 0;
    sin(THT), cos(THT), 0
    0,0,1];
L=d_pend;
for i=1:length(T)
   q(:,i) = [p(1,i) - sin(beta(i))*L;...
             p(2,i) + sin(alpha(i))*cos(beta(i))*L;...
             p(3,i) - cos(alpha(i))*cos(beta(i))*L];
end
f=figure; hold on;
if light_mode
    theme(f, "light");
end
grid on;
% the path
s=0:0.001:2*pi;
x1 = r1*cos(s);
x2 = r1*sin(s);
x3= r2*s./s;
path_plot = plot3(x1,x2,x3,'color',[0.1,0.4,0.2],'LineWidth',3);
plot3(p(1,:),p(2,:),p(3,:),'color',[0.5,0.5,0.5],'LineWidth',1);
f.CurrentAxes.YDir = 'Reverse';
f.CurrentAxes.ZDir = 'Reverse';
xlabel('$n_1$-axis(m)','Interpreter','latex')
ylabel('$n_2$-axis(m)','Interpreter','latex')
zlabel('$n_3$-axis(m)')
legend(path_plot,'The path $\mathcal{C}$')
% for PF vs TT comparison
view(110,25)
set(gca,'xlim',[-10 10]);
set(gca,'ylim',[-10 10]);
set(gca,'zlim',[-15 0]);
% for dist scenario
%view(40,20)
% set(gca,'xlim',[-7 7]);
% set(gca,'ylim',[-7 7]);
% set(gca,'zlim',[-15 0]);
for j = indx
    line_1 = [  q(1,j)-d_arm+d_prop  q(1,j)+d_arm-d_prop;
        q(2,j)   q(2,j) ;
        q(3,j)   q(3,j) ];
    line_2 = [  q(1,j)   q(1,j) ;
        q(2,j)-d_arm+d_prop   q(2,j)+d_arm-d_prop ;
        q(3,j)   q(3,j) ];
    line_3 = [q(:,j) q(:,j)+d_pend*[sin(beta(j));
        -sin(alpha(j)).*cos(beta(j));
        cos(alpha(j)).*cos(beta(j))  ]];
    line_1_r  = R_rot(:,:,j)*R_THT*(line_1-q(:,j))+q(:,j);
    line_2_r  = R_rot(:,:,j)*R_THT*(line_2-q(:,j))+q(:,j);
% draw propellers
    t = 0:pi/50:2*pi;
    x_circ_1 = d_prop * cos(t) + q(1,j)+d_arm;
    y_circ_1 = d_prop * sin(t) + q(2,j);
    z_circ_1 = q(3,j)*ones(1,length(t));
    x_circ_2 = (d_prop * cos(t) + q(1,j)-d_arm);
    y_circ_2 = d_prop * sin(t) + q(2,j);
    z_circ_2 = q(3,j)*ones(1,length(t));
    x_circ_3 = (d_prop * cos(t) + q(1,j));
    y_circ_3 = d_prop * sin(t) + q(2,j)+d_arm;
    z_circ_3 = q(3,j)*ones(1,length(t));
    x_circ_4 = d_prop * cos(t) + q(1,j);
    y_circ_4 = d_prop * sin(t) + q(2,j)-d_arm;
    z_circ_4 = q(3,j)*ones(1,length(t));
    circ_1 = R_rot(:,:,j)*R_THT*([x_circ_1;y_circ_1;z_circ_1]-q(:,j))+q(:,j);
    circ_2 = R_rot(:,:,j)*R_THT*([x_circ_2;y_circ_2;z_circ_2]-q(:,j))+q(:,j);
    circ_3 = R_rot(:,:,j)*R_THT*([x_circ_3;y_circ_3;z_circ_3]-q(:,j))+q(:,j);
    circ_4 = R_rot(:,:,j)*R_THT*([x_circ_4;y_circ_4;z_circ_4]-q(:,j))+q(:,j);
    h1 = plot3(line_1_r(1,:), line_1_r(2,:),line_1_r(3,:),'color',[0 0.4470 0.7410],'linewidth',2);
    h2 = plot3(line_2_r(1,:), line_2_r(2,:),line_2_r(3,:),'color',[0 0.4470 0.7410],'linewidth',2);
    h3 = plot3(q(1,j),q(2,j),q(3,j),'.','color',[0.3010 0.7450 0.9330],'MarkerSize',22);
    h_pend = plot3(line_3(1,:), line_3(2,:),line_3(3,:),'color',pend_color,'linewidth',2);
    h_payload = plot3(line_3(1,2), line_3(2,2),line_3(3,2),'.','color',payload_color,'markersize',22);
    h4 = plot3(circ_1(1,:),circ_1(2,:),circ_1(3,:),'color',[0 0.4470 0.7410],'linewidth',3);
    h5 = plot3(circ_2(1,:),circ_2(2,:),circ_2(3,:),'color',[0.6350 0.0780 0.1840],'linewidth',3);
    h6 = plot3(circ_3(1,:),circ_3(2,:),circ_3(3,:),'color',[0 0.4470 0.7410],'linewidth',3);
    h7 = plot3(circ_4(1,:),circ_4(2,:),circ_4(3,:),'color',[0 0.4470 0.7410],'linewidth',3);
    uistack(h3, 'top')
   text(q(1,j)+0.5,q(2,j)+0.5,q(3,j)-2,['$t =$ ',num2str(T(j)), 's'])
end