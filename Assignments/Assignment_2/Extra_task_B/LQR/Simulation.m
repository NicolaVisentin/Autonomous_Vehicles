clear
close all
clc

%% Data and parameters definition

% simulation data

t0=0;           % initial time [s]

x_i=0;          % initial x [m]
y_i=0;          % initial y [m]
theta_i=0;      % initial orientation [rad]

xi_in=[x_i y_i theta_i]';

% control definition

load('optimal_data.mat')
u=u_opt;
N=max(size(u));
t_u=linspace(t0,tf,N);

%% Simulation

[t_xi,xi]=ode45(@(t,xi) EquationOfMotion(t,xi,u,t_u),[t0 tf],xi_in);

%% Animation

% choose parameters

FPS=60;
duration=5;

% setup

N_anim=FPS*duration;              % number of frames ( = number of data points)
t_anim=linspace(t0,tf,N_anim);
xi_anim=interp1(t_xi,xi,t_anim);
u_anim=interp1(t_u,u,t_anim);

x_anim=xi_anim(:,1); 
y_anim=xi_anim(:,2); 
theta_anim=xi_anim(:,3);

% open figure

AnimFig=figure;
hold on
pl_pos=plot(nan,nan,'ro');
pl_seg=plot(nan,nan,'r');
pl_tr=plot(nan,nan,'k--');
grid on
box on
xlabel('x','Interpreter','latex')
ylabel('y','Interpreter','latex')
title('Simulation')
axis([-6 6 -6 6])

% starting position

xb=x_anim(1);   % back end x coordinate
yb=y_anim(1);   % back end y coordinate

xt=xb+0.5*cos(theta_anim(1));  % front end x coordinate
yt=yb+0.5*sin(theta_anim(1));  % front end y coordinate

set(pl_seg,'XData',[xb xt],'YData',[yb yt])
set(pl_pos,'XData',xb,'YData',yb)

pause(2)

% animation

for ii=2:N_anim

    % update plots

    xb=x_anim(ii);
    yb=y_anim(ii);

    xt=xb+0.5*cos(theta_anim(ii));  % front end x coordinate
    yt=yb+0.5*sin(theta_anim(ii));  % front end y coordinate
    
    set(pl_seg,'XData',[xb xt],'YData',[yb yt])
    set(pl_pos,'XData',xb,'YData',yb)
    set(pl_tr,'XData',x_anim(1:ii),'YData',y_anim(1:ii))

    % pause

    drawnow
    pause(1/FPS);

end
pause(2)

%% Plots

% plot controls in time

figure
hold on
plot(t_u,u(:,1),'b')
plot(t_u,u(:,2),'r')
yline(0)
grid on
box on
xlabel('t [s]',Interpreter='latex')
ylabel('u',Interpreter='latex')
title('Optimal control')
legend('$v^*(t)$','$\omega^*(t)$','interpreter','latex','location','best')
axis tight

% plot state in time

figure

subplot(3,1,1)
plot(t_xi,xi(:,1),'b')
grid on
box on
xlabel('t [s]',Interpreter='latex')
ylabel('x [m]',Interpreter='latex')
title('Optimal x')
axis tight

subplot(3,1,2)
plot(t_xi,xi(:,2),'g')
grid on
box on
xlabel('t [s]',Interpreter='latex')
ylabel('y [m]',Interpreter='latex')
title('Optimal y')
axis tight

subplot(3,1,3)
plot(t_xi,rad2deg(xi(:,3)),'c')
grid on
box on
xlabel('t [s]',Interpreter='latex')
ylabel('$\theta$ [deg]',Interpreter='latex')
title('Optimal orientation')
axis tight

figure
hold on
plot(xi(:,1),xi(:,2),'k')
plot(xi(1,1),xi(1,2),'ko')
grid on
box on
xlabel('x [m]','Interpreter','latex')
ylabel('y [m]','Interpreter','latex')
title('Optimal trajectory')
legend('','starting point','interpreter','latex','location','best')
axis([-6 6 -6 6])
hold off