clear
close all
clc

%rosinit
cd('/home/vise/Universita/Autonomous_vehicles/Assignments/Assignment_2/ExtraB/LQR')

%% Pre-simulation

% Define waypoints

x_way=[0 5 5 -5 -5 0 3 -3 0 3 0];
y_way=[0 0 5 -5 5 0 3 0 -3 0 0];
orient_way=[0 pi/2 5/4*pi pi/2 0 0 3/4*pi 3/2*pi pi/4 pi/2 3/2*pi];

N_way=length(x_way);
waypoints=[x_way' y_way' orient_way'];

% frequencies [Hz]

f_sim=100;
f_cmd=100;
f_odom=100;

% pass the optimal control and optimisation data

load('optimal_data.mat')
nxi=param.nxi;
nu=param.nu;
dfdxi=param.dfdxi;
dfdu=param.dfdu;

time_optu=linspace(0,tf,length(u_opt));
time_optxi=linspace(0,tf,length(xi_opt));

% resample the optimal data accordingly to the publisher frequency

time_cmd=0:1/f_cmd:tf;
time_cmd=time_cmd';
N=length(time_cmd);

xik=interp1(time_optxi,xi_opt,time_cmd);
uk=interp1(time_optu,u_opt,time_cmd);

xik(:,3)=mod(xik(:,3),2*pi);  % bring theta in (0,2*pi)

% control constraints

v_max=0.6;     % [m/s]
v_min=0;       % [m/s] 
w_max=1;       % [rad/s]
w_min=-1;      % [rad/s]

% tolerances on the distance to the waypoint

toll_dist=0.6;

%% LQR

% LQR parameters (weights on delta_xi and delta_u)

p_delta_x = 0.2/1^2;
p_delta_y = 0.2/1^2;
p_delta_theta = 0.2/1^2;
r_delta_v = 10/1^2;
r_delta_w = 0.001/1^2;
q_delta_x = 1/1^2;
q_delta_y = 1/1^2;
q_delta_theta = 0.1/1^2;

P=[p_delta_x 0 0; 0 p_delta_y 0; 0 0 p_delta_theta];     % Phi=0.5*(x(tf)-xf)'*P*(x(tf)-xf)
R=[r_delta_v 0; 0 r_delta_w ];                           % L=0.5*(u'*R*u+x'*Q*x)
Q=[q_delta_x 0 0; 0 q_delta_y 0; 0 0 q_delta_theta];     % L=0.5*(u'*R*u+x'*Q*x)

% solve Riccati equation to find matrix PP.
%
%   ! backward integration
%   ! Riccati equation is a matrix equation, but ode45 only works with
%     vectors; we need to "unwrap" all matrices in input to ode45

options=odeset('RelTol',1e-5,'AbsTol',1e-5*ones(1,nxi^2));
t0=time_cmd(1);
tf=time_cmd(end);

PP_tf=P;                % PP(tf) = d^2/dx^2(Phi)|tf = P
PP_tf_vect=P(1:end)';   % convert initial condition matrix into vector to input ode45

[t_PP,PP_vect]=ode45(@(t,PP) DRE(t,PP,Q,R,xik,time_cmd,uk,time_cmd,param),[tf t0],PP_tf_vect,options);

% note that PP_vect is actually a (N x Nx^2) matrix, where the i-th row
% corresponds to a vector that represents the "unwrapped" Riccati matrix in
% the i-th time instant. Let's flip, re-sample and reshape PP_vect to
% obtain Riccati matrix time history in a (Nx x Nx x N) 3D matrix where each
% one of the N "slices" corresponds to the Nx x Nx Riccati matrix at a
% certain time instant

PP_vect=flipud(PP_vect);              % flip PP_vector (was backward integrated)
t_PP=flipud(t_PP);                    % flip t_PP vector (was backward integrated)
PP_vect=interp1(t_PP,PP_vect,time_cmd);   % re-sample PP_vect
PP=reshape(PP_vect.',nxi,nxi,[]);       % reshape PP_vect into PP as said before
PP=permute(PP,[2, 1, 3]);

% compute the gain matrix K time history ( K is a Nx by Nu by N 3D matrix; 
% each "slice" Nu x nx is the K matrix at one of the N time instants)

B=dfdu(xik,uk);
K=pagemtimes(inv(R)*B',PP);     % K(t)=R^-1*B(t)*P(t)

K_simv=K(1,:,:);
K_simw=K(2,:,:);
K_simv=squeeze(K_simv)';
K_simw=squeeze(K_simw)';

%% Save simulation data

%save('sim_data','out')

%% Post-simulation

close all
clc

% extract data

%load('sim_data.mat')

time_sim=out.tout;
dist=out.dist;                       % distance from next point
err_dir=out.err_dir;                 % error on the linear direction between the robot and the next point
err_or=out.err_orient;               % error on the orientation of the robot wrt the one of the next waypoint
theta=out.orientation;               % theta (orientation of the robot: 0-360° wrt positive x semiaxis, counterclockwise)
pos=out.position;                    % position of the robot (x,y)
u_v=out.u_v;                         % input velocity (no saturation)
u_w=out.u_w;                         % input angular velocity (no saturation)
v_cmd=out.v_cmd;                     % actual velocity command (with saturation)
w_cmd=out.w_cmd;                     % actual angular velocity command (with saturation)
delta_xi=squeeze(out.delta_xi_sim)'; % xi-xi* LQR input
u_ref=out.u_ref_sim;                 % u* reference control
u_cmd=out.u_cmd_sim;                 % u actual control (after saturation)

u_ref=u_ref(2:2:end);

% animation of the trajectory

if false

figure
hold on
tr=plot(nan,nan,'k--');
h1=plot(pos(1,1),pos(1,2),'ro');
h2=plot([pos(1,1) pos(1,1)+cos(theta(1))],[pos(1,2) pos(1,2)+cos(theta(1))],'r');
way1=plot(x_way(1),y_way(1),'go');
way2=plot([x_way(1) x_way(1)+cos(orient_way(1))],[y_way(1) y_way(1)+sin(orient_way(1))],'g');
grid on
box on
xlabel('X [m]')
ylabel('Y [m]')
dynamic_title=title('Animation (t = 0 s)');
axis([-8 8 -8 8])

pause(1)
jj=2;
for ii=1:1:length(time_sim)

    front_x=pos(ii,1)+0.7*cos(theta(ii));
    front_y=pos(ii,2)+0.7*sin(theta(ii));

    set(tr,'XData',pos(1:ii,1),'YData',pos(1:ii,2))
    set(h1,'XData',pos(ii,1),'YData',pos(ii,2))
    set(h2,'Xdata',[pos(ii,1) front_x],'YData',[pos(ii,2) front_y])

    if ii==1 || ii==1110 || ii==2280 || ii==5350 || ii==7490 || ii==8900 || ii==9850 || ii==11250 || ii==12166 || ii==13060
        frontway_x=x_way(jj)+0.7*cos(orient_way(jj));
        frontway_y=y_way(jj)+0.7*sin(orient_way(jj));
        set(way1,'XData',x_way(jj),'YData',y_way(jj))
        set(way2,'Xdata',[x_way(jj) frontway_x],'YData',[y_way(jj) frontway_y])
        jj=jj+1;
    end
    set(dynamic_title,'String',sprintf('Animation (t = %.1f s)',time_sim(ii)));

    drawnow
    pause(0.001)

end

end

% plot trajectory and waypoints

figure
hold on
plot(x_way,y_way,'b.')
quiver(x_way-cos(orient_way)/2,y_way-sin(orient_way)/2,cos(orient_way),sin(orient_way),0,'r')
plot(pos(:,1),pos(:,2),'k')
plot(xik(:,1),xik(:,2),'k--')
grid on
box on
xlabel('X [m]','Interpreter','latex')
ylabel('Y [m]','Interpreter','latex')
title('Trajectory and waypoints')
legend('','','actual trajectory','reference','Interpreter','LaTex','Location','best')
hold off

% plot states

figure

subplot(3,1,1)
hold on
plot(time_sim,pos(:,1),'b')
plot(time_cmd,xik(:,1),'b--')
grid on
box on
xlabel('t [s]',Interpreter='latex')
ylabel('x [m]',Interpreter='latex')
title('x')
legend('x','x*','Interpreter','latex','Location','best')
axis tight
hold off

subplot(3,1,2)
hold on
plot(time_sim,pos(:,2),'g')
plot(time_cmd,xik(:,2),'g--')
grid on
box on
xlabel('t [s]',Interpreter='latex')
ylabel('y [m]',Interpreter='latex')
title('y')
legend('y','y*','interpreter','latex','Location','best')
axis tight
hold off

subplot(3,1,3)
hold on
plot(time_sim,rad2deg(theta),'c')
plot(time_cmd,rad2deg(xik(:,3)),'c--')
grid on
box on
xlabel('t [s]',Interpreter='latex')
ylabel('$\theta$ [deg]',Interpreter='latex')
title('Orientation')
legend('$\theta$','$\theta$*','interpreter','latex','Location','best')
axis tight
hold off

% plot velocities

figure

subplot(2,1,1)
hold on
plot(time_sim,u_v,'b.')
plot(time_sim,v_cmd,'r.')
yline(0)
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('v [m/s]','Interpreter','latex')
title('Linear velocity')
legend('control v','actual input v','interpreter','latex','Location','best')
xlim([time_sim(1) time_sim(end)])
hold off

subplot(2,1,2)
hold on
plot(time_sim,u_w,'b.')
plot(time_sim,w_cmd,'r.')
yline(0)
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('$\omega$ [rad/s]','Interpreter','latex')
title('Yaw rate')
legend('control $\omega$','actual input $\omega$','interpreter','latex','Location','best')
xlim([time_sim(1) time_sim(end)])
hold off

figure

subplot(2,1,1)
hold on
plot(time_sim,v_cmd,'b')
plot(time_cmd,uk(:,1),'r')
yline(0)
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('v [m/s]','Interpreter','latex')
title('Linear velocity command')
legend('actual input v','reference v*','interpreter','latex','Location','best')
xlim([time_sim(1) time_sim(end)])
ylim([v_min-0.1 v_max+0.1])
hold off

subplot(2,1,2)
hold on
plot(time_sim,w_cmd,'b')
plot(time_cmd,uk(:,2),'r')
yline(0)
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('$\omega$ [rad/s]','Interpreter','latex')
title('Yaw rate command')
legend('actual input $\omega$','reference $\omega$*','interpreter','latex','Location','best')
xlim([time_sim(1) time_sim(end)])
ylim([w_min-0.2 w_max+0.2])
hold off

% plot LQR

figure

subplot(2,1,1)
plot(time_sim,delta_xi)
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('$\Delta \underline{\xi}$','Interpreter','latex')
title('LQR input')
legend('$\Delta x$','$\Delta y$','$\Delta \theta$','Interpreter','LaTex','Location','best')
xlim([time_sim(1) time_sim(end)])

subplot(2,1,2)
plot(time_sim,u_cmd-u_ref)
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('$\Delta \underline{u}$','Interpreter','latex')
title('LQR output')
legend('$\Delta v$','$\Delta \omega$','Interpreter','LaTex','Location','best')
xlim([time_sim(1) time_sim(end)])

% plot errors

figure

subplot(3,1,1)
plot(time_sim,dist)
yline(0)
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('dist [m]','Interpreter','latex')
title('Distance from the next waypoint')
xlim([time_sim(1) time_sim(end)])

subplot(3,1,2)
plot(time_sim,rad2deg(err_dir))
yline(0)
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('err [deg]','Interpreter','latex')
title('Error on the direction (wrt to the line to the next waypoint)')
xlim([time_sim(1) time_sim(end)])

subplot(3,1,3)
plot(time_sim,rad2deg(err_or))
yline(0)
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('err [deg]','Interpreter','latex')
title('Error on the orientation (wrt to the orientation at the waypoint)')
xlim([time_sim(1) time_sim(end)])