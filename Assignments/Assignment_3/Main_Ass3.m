clear
close all
clc
%rosinit

cd('/home/vise/Universita/Autonomous_vehicles/Assignments/Assignment_3')

%% Spawn stuff into gazebo world

% Create matlab object to communicate with gazebo

gazebo=ExampleHelperGazeboCommunicator;

% Add balls to gazebo

ball1=ExampleHelperGazeboModel('Ball');
addLink(ball1,'sphere',0.1,'color',[200 48 48 1]);   % color: #c83030
spawnModel(gazebo,ball1,[0 6 0.1])                   % 6 meters on the left

ball2=ExampleHelperGazeboModel('Ball2');
addLink(ball2,'sphere',0.2,'color',[200 0 103 1]);   % color: #c80067
spawnModel(gazebo,ball2,[6 0 0.1])                   % 6 meters in front

%% Control limits

v_max=0.26;
v_min=-0.26;

w_max=1.82;
w_min=-1.82;

%% Simulink

f_sim=30;

%% Save data

%save('sim_data','out')
load('sim_data_2024_12_29.mat')

%% Plot results

t=out.tout;
scan=out.scan;
v_cmd=out.v_cmd;
w_cmd=out.w_cmd;
x=out.x;
y=out.y;
theta=out.theta;
u_centroid=out.centr_position(:,1);
v_centroid=out.centr_position(:,2);

% Trajectory

figure
hold on
plot(x(1),y(1),'bx')
plot(x,y,'b')
rectangle('Position',[[6 0]-0.2,2*0.2,2*0.2],'Curvature',[1,1],'FaceColor','#c80067','EdgeColor','#c80067','LineWidth',2);
rectangle('Position',[[0 6]-0.2,2*0.2,2*0.2],'Curvature',[1,1],'FaceColor','#c83030','EdgeColor','#c83030','LineWidth',2);
grid on
box on
xlabel('X [m]','Interpreter','latex')
ylabel('Y [m]','Interpreter','latex')
title('Trajectory')
legend('starting position','Interpreter','latex','Location','best')
axis([-1 7 -1 7])
hold off

% Velocity command

figure
hold on
plot(t,v_cmd,'b')
plot(t,w_cmd,'r')
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Velocity command')
legend('v [m/s]','$\omega$ [rad/s]','Interpreter','latex','Location','best')
xlim([t(1) t(end)])
hold off

% Centroid position

figure
plot(t,u_centroid,'b')
yline(320,'--')
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Centroid horizontal position')
legend('u [px]','center of the frame','Interpreter','latex','Location','best')
xlim([t(1) t(end)])