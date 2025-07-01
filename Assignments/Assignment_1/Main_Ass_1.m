clear
close all
clc
%rosshutdown
cd('/home/vise/Universita/Autonomous_vehicles/Assignments/Assignment_1')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% Assignment 1                                                            %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Load bag

bagselect=rosbag('ex_A1.bag');
%bagselect=rosbag('MyBag_Ass1_2024-11-17.bag');

%% PART 1: Minimum distance from obstacles

% select messages to extract (Lidar data)

bagselect_scan=select(bagselect,'Topic','/scan');

% extract messages and data

scanStruct=readMessages(bagselect_scan,'DataFormat','struct');
N_scan=length(scanStruct);

angles=scanStruct{1}.AngleMin:scanStruct{1}.AngleIncrement:scanStruct{1}.AngleMax;
ranges=zeros(N_scan,length(angles));
time_scan=zeros(1,N_scan);
min_dist=zeros(1,N_scan);

for ii=1:N_scan   
    time_scan(ii)=double(scanStruct{ii}.Header.Stamp.Sec)+double(scanStruct{ii}.Header.Stamp.Nsec)*1e-9;
    ranges(ii,:)=scanStruct{ii}.Ranges;
    min_dist(ii)=min(ranges(ii,:));
end
time_scan=time_scan-time_scan(1);

% plot minimum distance wrt time

figure
plot(time_scan,min_dist,'-')
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('dist [m]','Interpreter','latex')
title('Minimun distance from obstacles')
axis tight

% plot provided Lidar data

lidar=0;
if lidar==1
    FigTag=figure;
    for ii=1:N_scan
        clf(FigTag)
        plotXY(ranges(ii,:),angles);
        title('Lidar (provided)')
        drawnow
        pause(0.001)
    end
end

%% PART 2: Input estimation

% extract odometry data

bagselect_odom=select(bagselect,'Topic','/odom');
odomStruct=readMessages(bagselect_odom,'DataFormat','struct');

N_odom=length(odomStruct);
w_odom=zeros(N_odom,1);
v_odom=zeros(N_odom,1);
time_odom=zeros(1,N_odom);

for ii=1:N_odom  
    time_odom(ii)=double(odomStruct{ii}.Header.Stamp.Sec)+double(odomStruct{ii}.Header.Stamp.Nsec)*1e-9;
    w_odom(ii)=odomStruct{ii}.Twist.Twist.Angular.Z;
    v_odom(ii)=odomStruct{ii}.Twist.Twist.Linear.X;
end
time_odom=time_odom-time_odom(1);

f_odom=mean(1./diff(time_odom));

% plot provided trajectory

x_traj=zeros(1,N_odom);
y_traj=zeros(1,N_odom);
for ii=1:N_odom
    x_traj(ii)=odomStruct{ii}.Pose.Pose.Position.X;
    y_traj(ii)=odomStruct{ii}.Pose.Pose.Position.Y;
end

figure
hold on
plot(x_traj,y_traj,'b')
plot(x_traj(1),y_traj(1),'bx')
grid on
box on
xlabel('X','Interpreter','latex')
ylabel('Y','Interpreter','latex')
title('Trajectory (provided)')
legend('','starting position','Interpreter','latex','Location','best')
hold off

% elaborate data to reconstruct the \cmd_vel: filter signal

v_odom_filtered=medfilt1(v_odom,80);
w_odom_filtered=medfilt1(w_odom,80);

figure

subplot(2,1,1)
hold on
plot(time_odom,v_odom,'b')
plot(time_odom,v_odom_filtered,'r')
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('v [m/s]','Interpreter','latex')
title('Provided linear velocity (odom)')
legend('provided v (odom)','filtered v','Interpreter','latex','Location','best')
axis tight
hold off

subplot(2,1,2)
hold on
plot(time_odom,w_odom,'b')
plot(time_odom,w_odom_filtered,'r')
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('$\omega$ [rad/s]','Interpreter','latex')
title('Provided angular velocity (odom)')
legend('provided $\omega$ (odom)','filtered $\omega$','Interpreter','latex','Location','best')
axis tight
hold off

% elaborate data to reconstruct the \cmd_vel: reconstruct input vector

v_cmd=0.01*round(v_odom_filtered/0.01);
w_cmd=0.1*round(w_odom_filtered/0.1);

figure

subplot(2,1,1)
hold on
plot(time_odom,v_odom,'b')
plot(time_odom,v_cmd,'r.')
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('v [m/s]','Interpreter','latex')
title('Reconstructed linear velocity')
legend('provided v (odom)','reconstructed v','Interpreter','latex','Location','best')
axis tight
hold off

subplot(2,1,2)
hold on
plot(time_odom,w_odom,'b')
plot(time_odom,w_cmd,'r.')
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('$\omega$ [rad/s]','Interpreter','latex')
title('Reconstructed angular velocity')
legend('provided $\omega$ (odom)','reconstructed $\omega$','Interpreter','latex','Location','best')
axis tight
hold off

%% Simulink

% create time vector and command velocity for simulink

time_cmd=time_odom-time_odom(1);

v_ts=timeseries(v_cmd,time_cmd);
w_ts=timeseries(w_cmd,time_cmd);
null=timeseries(zeros(1,N_odom),time_cmd);

f_cmd=round(f_odom);

% initialise ROS node

ipaddress='localhost';
tbot=turtlebot(ipaddress);
%rosinit

% reset initial position and time of the turtlebot in gazebo

pause(1)
resetService=rossvcclient('/gazebo/reset_world');
call(resetService);

% set initial position of the turtlebot

setStateService = rossvcclient('/gazebo/set_model_state');
setModelStateMsg = rosmessage(setStateService);
setModelStateMsg.ModelState.ModelName = 'turtlebot3_waffle_pi';

setModelStateMsg.ModelState.Pose.Position.X = odomStruct{1}.Pose.Pose.Position.X;
setModelStateMsg.ModelState.Pose.Position.Y = odomStruct{1}.Pose.Pose.Position.Y;
setModelStateMsg.ModelState.Pose.Position.Z = odomStruct{1}.Pose.Pose.Position.Z;
setModelStateMsg.ModelState.Pose.Orientation.X = odomStruct{1}.Pose.Pose.Orientation.X;
setModelStateMsg.ModelState.Pose.Orientation.Y = odomStruct{1}.Pose.Pose.Orientation.Y;
setModelStateMsg.ModelState.Pose.Orientation.Z = odomStruct{1}.Pose.Pose.Orientation.Z;
setModelStateMsg.ModelState.Pose.Orientation.W = odomStruct{1}.Pose.Pose.Orientation.W;

call(setStateService, setModelStateMsg);

% simulate

%out=sim('Ass_1.slx');

%% Plot results of the simulation

%load('sim_data_2024_12_29.mat')

t_sim=out.tout;
x_sim=out.x_sim;
y_sim=out.y_sim;
v_sim=out.v_sim;
w_sim=out.w_sim;

    % (to remove first "ficticious zeros in the vectors)
indx_start_x=find(round(100*x_sim)==round(100*x_traj(1)),1);
indx_start_y=find(round(100*y_sim)==round(100*y_traj(1)),1);
x_sim(1:indx_start_x)=x_sim(indx_start_x+1);
y_sim(1:indx_start_y)=y_sim(indx_start_y+1);

figure
hold on
plot(x_traj(1),y_traj(1),'bx')
plot(x_traj,y_traj,'b--')
plot(x_sim,y_sim,'b')
grid on
box on
xlabel('X','Interpreter','latex')
ylabel('Y','Interpreter','latex')
title('Trajectory')
legend('starting position','provided','simulated','Interpreter','latex','Location','best')
hold off

figure

subplot(3,1,1)
hold on
plot(time_odom,v_odom,'b.')
plot(time_odom,w_odom,'r.')
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Provided velocities (odom)')
legend('v [m/s]','$\omega$ [rad/s]','Interpreter','latex','Location','best')
xlim([time_odom(1) time_odom(end)])
ylim([-0.3 0.5])
hold off

subplot(3,1,2)
hold on
plot(time_cmd,v_cmd,'b.')
plot(time_cmd,w_cmd,'r.')
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Simulation velocities (command)')
legend('v [m/s]','$\omega$ [rad/s]','Interpreter','latex','Location','best')
xlim([time_cmd(1) time_cmd(end)])
ylim([-0.3 0.5])
hold off

subplot(3,1,3)
hold on
plot(t_sim,v_sim,'b.')
plot(t_sim,w_sim,'r.')
grid on
box on
xlabel('t [s]','Interpreter','latex')
title('Simulation velocities (odom)')
legend('v [m/s]','$\omega$ [rad/s]','Interpreter','latex','Location','best')
xlim([t_sim(1) t_sim(end)])
ylim([-0.3 0.5])
hold off

%% Functions

function endSim = plotXY(Ranges,Angles)

% Takes as input "Read Scan" block output and processes it to create a plot in
% global XY reference. Only starts when at least some data different from 
% zero are provided (i.e. when the subscriber outputs a message).

endSim = any(Ranges>0);

if endSim

    x = Ranges.*cos(Angles);
    y = Ranges.*sin(Angles);

    xValid = x(isfinite(x));
    yValid = y(isfinite(y));
    
    plot(xValid,yValid,'.','MarkerSize',8)
    title('Laser scan')
    grid on
    xlabel('X')
    ylabel('Y')
    set(gca,'Ydir','reverse')   % invert Y axis
    axis equal
    xlim([-5 5])
    ylim([-5 5])
    view([90 -90])

end

end