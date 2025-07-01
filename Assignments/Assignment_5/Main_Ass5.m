close all
clc

t=out.tout;
car_position=out.car_position;
gate_angle=out.gate_angle;
gate_speed=out.gate_speed;
gate_acceleration=out.gate_acceleration;

figure

subplot(4,1,1)
plot(t,car_position,'b-','LineWidth',1.5)
yline(0,'r-')
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('$x$ $[m]$','Interpreter','latex')
title('Cars position')

subplot(4,1,2)
plot(t,gate_angle,'b-','LineWidth',1.5)
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('$\theta$ $[deg]$','Interpreter','latex')
title('Gate angle')

subplot(4,1,3)
plot(t,gate_speed,'b-','LineWidth',1.5)
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('$\omega$ $[deg/s]$','Interpreter','latex')
title('Gate speed')

subplot(4,1,4)
plot(t,gate_acceleration,'b-','LineWidth',1.5)
grid on
box on
xlabel('t [s]','Interpreter','latex')
ylabel('$\alpha$ $[deg/s^2]$','Interpreter','latex')
title('Gate acceleration')