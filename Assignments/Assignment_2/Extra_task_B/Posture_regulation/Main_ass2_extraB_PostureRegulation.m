clear
close all
clc

%rosinit
cd('/home/vise/Universita/Autonomous_vehicles/Assignments/Assignment_2/ExtraB/PostureRegulation')

%% Pre-simulation

% Define waypoints

x_way=[0 5 5 -5 -5 0 3 -3 0 3 0];
y_way=[0 0 5 -5 5 0 3 0 -3 0 0];
orient_way=[0 pi/2 5/4*pi pi/2 0 0 3/4*pi 3/2*pi pi/4 pi/2 3/2*pi];

N=length(x_way);

waypoints=[x_way' y_way' orient_way'];

% Plot waypoints

figure
hold on
plot(x_way,y_way,'b.')
quiver(x_way-cos(orient_way)/2,y_way-sin(orient_way)/2,cos(orient_way),sin(orient_way),0,'r')
grid on
box on
xlabel('X [m]','Interpreter','latex')
ylabel('Y [m]','Interpreter','latex')
title('Waypoints')
hold off

% Select control gains

K1=3;
K2=3;
K3=0.8;

r_slow=0.4;     % radius within slowing down before reaching the waypoint

% select tolerances on the reaching of the waypoints (to pass to the next
% one)

toll_dist=0.01;

% constraints on command

v_max=0.5;
v_min=-0.5;

w_max=2.84;
w_min=-2.84;

% frequencies [Hz]

f_sim=100;
f_cmd=100;
f_odom=100;

%% Save simulation data

%save('sim_data','out')
load('sim_data.mat')

%% Post-simulation

close all
clc

% extract data

time=out.tout;
dist=out.dist;           % distance from next point
err_dir=out.err_dir;     % error on the linear direction between the robot and the next point
err_or=out.err_orient;   % error on the orientation of the robot wrt the one of the next waypoint
theta=out.orientation;   % theta (orientation of the robot: 0-360° wrt positive x semiaxis, counterclockwise)
pos=out.position;        % position of the robot (x,y)
u_v=out.u_v;             % input velocity (no saturation)
u_w=out.u_w;             % input angular velocity (no saturation)
v_cmd=out.v_cmd;         % actual velocity command (with saturation)
w_cmd=out.w_cmd;         % actual angular velocity command (with saturation)

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
for ii=1:1:length(time)

    front_x=pos(ii,1)+0.7*cos(theta(ii));
    front_y=pos(ii,2)+0.7*sin(theta(ii));

    set(tr,'XData',pos(1:ii,1),'YData',pos(1:ii,2))
    set(h1,'XData',pos(ii,1),'YData',pos(ii,2))
    set(h2,'Xdata',[pos(ii,1) front_x],'YData',[pos(ii,2) front_y])

    if ii==1 || ii==1330 || ii==2750 || ii==6400 || ii==8830 || ii==10530 || ii==11640 || ii==13370 || ii==14510 || ii==15600
        frontway_x=x_way(jj)+0.7*cos(orient_way(jj));
        frontway_y=y_way(jj)+0.7*sin(orient_way(jj));
        set(way1,'XData',x_way(jj),'YData',y_way(jj))
        set(way2,'Xdata',[x_way(jj) frontway_x],'YData',[y_way(jj) frontway_y])
        jj=jj+1;
    end
    set(dynamic_title,'String',sprintf('Animation (t = %.1f s)',time(ii)));

    drawnow
    pause(0.001)

end

end

% plot trajectory and waypoints

figure
hold on
plot(x_way,y_way,'b.')
quiver(x_way-cos(orient_way)/2,y_way-sin(orient_way)/2,cos(orient_way),sin(orient_way),0,'r')
plot(pos(:,1),pos(:,2),'k--')
grid on
box on
xlabel('X [m]','Interpreter','latex')
ylabel('Y [m]','Interpreter','latex')
title('Trajectory and waypoints')
hold off

% plot velocities

figure

subplot(2,1,1)
hold on
plot(time,u_v,'b.')
plot(time,v_cmd,'r.')
yline(0)
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('v [m/s]','Interpreter','latex')
title('Linear velocity')
legend('control v','actual input v','interpreter','latex','Location','best')
xlim([time(1) time(end)])
hold off

subplot(2,1,2)
hold on
plot(time,u_w,'b.')
plot(time,w_cmd,'r.')
yline(0)
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('$\omega$ [rad/s]','Interpreter','latex')
title('Yaw rate')
legend('control $\omega$','actual input $\omega$','interpreter','latex','Location','best')
xlim([time(1) time(end)])
hold off

figure

subplot(2,1,1)
plot(time,v_cmd,'r')
yline(0)
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('v [m/s]','Interpreter','latex')
title('Linear velocity command')
xlim([time(1) time(end)])
ylim([v_min-0.1 v_max+0.1])

subplot(2,1,2)
plot(time,w_cmd,'r')
yline(0)
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('$\omega$ [rad/s]','Interpreter','latex')
title('Yaw rate command')
xlim([time(1) time(end)])
ylim([w_min-0.2 w_max+0.2])

% plot errors

figure

subplot(3,1,1)
plot(time,dist)
yline(0)
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('dist [m]','Interpreter','latex')
title('Distance from the next waypoint')
xlim([time(1) time(end)])

subplot(3,1,2)
plot(time,rad2deg(err_dir))
yline(0)
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('err [deg]','Interpreter','latex')
title('Error on the direction (wrt to the line to the next waypoint)')
xlim([time(1) time(end)])

subplot(3,1,3)
plot(time,rad2deg(err_or))
yline(0)
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('err [deg]','Interpreter','latex')
title('Error on the orientation (wrt to the orientation at the waypoint)')
xlim([time(1) time(end)])