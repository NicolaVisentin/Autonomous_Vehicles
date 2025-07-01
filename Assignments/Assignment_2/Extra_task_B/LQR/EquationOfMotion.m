function xip=EquationOfMotion(t,xi,u,t_u)

% ode45 discretises ad cazzum: make sure to take u at the "correct" time
% istant (coherent with x which is being integrated)

u=interp1(t_u,u,t);

% differential equation of motion

xip=zeros(3,1);

xip(1)=cos(xi(3))*u(1);
xip(2)=sin(xi(3))*u(1);
xip(3)=u(2);

end