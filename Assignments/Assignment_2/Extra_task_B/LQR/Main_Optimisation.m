clear
close all
clc

cd('/home/vise/Universita/Autonomous_vehicles/Assignments/Assignment_2/ExtraB/LQR')

%% Setup

% control limits

v_max=0.5;     % [m/s]
v_min=0;    % [m/s] 
w_max=1;     % [rad/s]
w_min=-1;    % [rad/s]

% waypoints

x_way=[0 5 5 -5 -5 0 3 -3 0 3 0];
y_way=[0 0 5 -5 5 0 3 0 -3 0 0];
orient_way=[0 pi/2 5/4*pi pi/2 0 0 3/4*pi 3/2*pi pi/4 pi/2 3/2*pi];

N_way=length(x_way);
waypoints=[x_way' y_way' orient_way'];

% number of controls and states

nxi=3;
nu=2;

% equation of motion and its Jacobians

xip=@(xi,u) [ cos(xi(3))*u(1);
              sin(xi(3))*u(1);
              u(2) 
            ];

dfdxi = @(xi,u) [ 0, 0, -u(1)*sin(xi(3));
                  0, 0,  u(1)*cos(xi(3));
                  0, 0,  0 ];

dfdu = @(xi,u) [ cos(xi(3)), 0;
                 sin(xi(3)), 0;
                 0, 1 ];

% collect parameters in a struct

param.v_max=v_max;
param.v_min=v_min;
param.w_max=w_max;
param.w_min=w_min;
param.dfdxi=dfdxi;
param.xip=xip;
param.dfdu=dfdu;
param.nxi=nxi;
param.nu=nu;

%% Optimise every piece of trajectory

% discretisation intervals for the signle optimisation between 2 waypoints

N=201;
param.N=N;

% iterative procedure

time_opt=[];
u_opt=[];
xi_opt=[];

u_opt_ii=[];
xi_opt_ii=waypoints(1,:);
tf_prec=0;
t_waypoints=zeros(1,N_way);

for ii=1:N_way-1
    
    % optimise
    xi1_ii=xi_opt_ii(end,:);
    xi2_ii=waypoints(ii+1,:);

        if ii==8                  
            xi2_ii(3)=7*pi/4;     
        end                     
        if ii==9                  
            xi1_ii(3)=pi/4;      
        end                    

    [u_opt_ii,xi_opt_ii,tf_opt_ii]=optimise(xi1_ii,xi2_ii,param);

    time_ii=linspace(0,tf_opt_ii,N+1);

    % update total vectors
    xi_opt=[xi_opt; xi_opt_ii(1:end-1,:)];
    u_opt=[u_opt; u_opt_ii];
    time_opt=[time_opt, time_ii(1:end-1)+tf_prec];

    tf_prec=tf_opt_ii+tf_prec;
    t_waypoints(ii+1)=tf_prec;

end
time_opt=[time_opt, tf_prec]';
xi_opt=[xi_opt; xi_opt_ii(end,:)];

tf=time_opt(end);
N=length(time_opt)-1;

% build time vectors

h=tf/N;
t_xi=0:h:N*h;
t_u=t_xi(1:end-1);

% resample xi_opt and u_opt on t_xi and t_u

xi_opt=interp1(time_opt,xi_opt,t_xi);
u_opt=interp1(time_opt(1:end-1),u_opt,t_u,'linear','extrap');

%% Plot the results

%load('optimal_data.mat')

% plot optimal controls in time

figure
hold on
plot(t_u,u_opt(:,1),'b')
plot(t_u,u_opt(:,2),'r')
yline(0)
grid on
box on
xlabel('t [s]',Interpreter='latex')
ylabel('u*',Interpreter='latex')
title('Optimal control')
legend('$v^*(t)$','$\omega^*(t)$','interpreter','latex','location','best')
axis tight

% plot optimal state in time

figure

subplot(3,1,1)
hold on
plot(t_xi,xi_opt(:,1),'b')
plot(t_waypoints,waypoints(:,1),'bo','MarkerSize',5)
grid on
box on
xlabel('t [s]',Interpreter='latex')
ylabel('x [m]',Interpreter='latex')
title('Optimal x')
legend('$x^*(t)$','waypoints','interpreter','latex','location','best')
axis tight
hold off

subplot(3,1,2)
hold on
plot(t_xi,xi_opt(:,2),'g')
plot(t_waypoints,waypoints(:,2),'go','MarkerSize',5)
grid on
box on
xlabel('t [s]',Interpreter='latex')
ylabel('y [m]',Interpreter='latex')
title('Optimal y')
legend('$y^*(t)$','waypoints','interpreter','latex','location','best')
axis tight
hold off

subplot(3,1,3)
hold on
plot(t_xi,rad2deg(xi_opt(:,3)),'c')
plot(t_waypoints,rad2deg(waypoints(:,3)),'co','MarkerSize',5)
grid on
box on
xlabel('t [s]',Interpreter='latex')
ylabel('$\theta$ [deg]',Interpreter='latex')
title('Optimal orientation')
legend('$\theta^*(t)$','waypoints','interpreter','latex','location','best')
axis tight
hold off

% plot optimal trajectory

figure
hold on
plot(xi_opt(:,1),xi_opt(:,2),'k')
plot(x_way,y_way,'b.')
quiver(x_way-cos(orient_way)/2,y_way-sin(orient_way)/2,cos(orient_way),sin(orient_way),0,'r')
grid on
box on
xlabel('x [m]','Interpreter','latex')
ylabel('y [m]','Interpreter','latex')
title(['Optimal trajectory (t_f = ',num2str(tf),' s)'])
legend('','','waypoints','interpreter','latex','location','best')
axis([-6 6 -6 6])
hold off

%% save optimal control and final time

%save('optimal_data.mat','u_opt','t_u','tf','xi_opt','t_xi','param','waypoints','N_way','t_waypoints')