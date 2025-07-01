function [u_opt,xi_opt,tf_opt]=optimise(xi1,xi2,param)

% extract parameters

v_max=param.v_max;
v_min=param.v_min;
w_max=param.w_max;
w_min=param.w_min;
xip=param.xip;
N=param.N;
nxi=param.nxi;
nu=param.nu;

% optimisation problem data

t0=0;                % initial time [s]

x_i=xi1(1);          % initial x [m]
x_f=xi2(1);          % final x [m]

y_i=xi1(2);          % initial y [m]
y_f=xi2(2);          % final y [m]

theta_i=xi1(3);      % initial orientation [rad]
theta_f=xi2(3);      % final orientation [rad]

xi_i=[x_i y_i theta_i]';
xi_f=[x_f y_f theta_f]';

% weights for the control and the state

dx=x_f-x_i;
dy=y_f-y_i;
dist=sqrt(dy^2+dx^2);

p_x = 1000/dist^2;          % weight for the final x
p_y = 1000/dist^2;          % weight for the final y
p_theta = 10/2*pi^2;        % weight for the final orientation
r_v = 0/v_max^2;            % weight for the control action v (integral cost)
r_w = 0.1/w_max^2;          % weight for the control action w (integral cost)
q_x = 0/dist^2;             % weight for x (integral cost)
q_y = 0/dist^2;             % weight for y (integral cost)
q_theta = 0/2*pi^2;         % weight for the orientation (integral cost)
alpha=10/(dist/v_max)^2;    % weight on the final time

P=[p_x 0 0; 0 p_y 0; 0 0 p_theta];     % Phi=0.5*(xi(tf)-xif)'*P*(xi(tf)-xif)+alpha*(tf-t0)
R=[r_v 0; 0 r_w];                      % L=0.5*(u'*R*u+xi'*Q*xi)
Q=[q_x 0 0; 0 q_y 0; 0 0 q_theta];     % L=0.5*(u'*R*u+xi'*Q*xi)

% solver options

options=optimoptions('fmincon','SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true,'FiniteDifferenceType','central','UseParallel',false,'PlotFcn','optimplotfval','Display','none');

% cost functions and their Jacobians

L = @(xi,u) 0.5*xi'*Q*xi+0.5*u'*R*u;
phi = @(xi,tf) 0.5*(xi-xi_f)'*P*(xi-xi_f)+alpha*(tf-t0);

dLdxi = @(xi,u) xi'*Q;
dLdu = @(xi,u) u'*R;
dphidxi =@(xi) (xi-xi_f)'*P;
dphidtf=alpha;

%% Setup for the method

% define initial guess: z_guess=[xi0' u0' xi1' u1' ... xiN-1' uN-1' xiN' tf]'

u_guess=[v_max*ones(N,1), zeros(N,1)];             % choose initial guess for the control: (N x nu) vector
tf_guess=dist/v_max;                        % choose initial guess for the final time
z_guess=assign_z0(nxi,nu,N,xi_i,u_guess,tf_guess,xip);

% add temporay parameters to param struct to adapt notation to be able to
% use already existing functions

param.xi_in=xi_i;
param.L=L;
param.dLdxi=dLdxi;
param.dLdu=dLdu;
param.phi=phi;
param.dphidxi=dphidxi;
param.dphidtf=dphidtf;

% compute initial cost

J_in=CostAndGrad(z_guess,param);

%% Optimisation

% define objective function 

ObjFun=@(z) CostAndGrad(z,param);

% define bounds on state and control

z_lb=AssignZbound([-inf -inf -inf],[v_min w_min],0,N,nxi,nu);
z_ub=AssignZbound([inf inf inf],[v_max w_max],inf,N,nxi,nu);

lb = z_lb; % lower bound
ub = z_ub; % upper bound

% define nonlinear constraints

NLcon=@(z) NonlinConstraintAndGrad(z,param);

% define linear constraints

A = [];
b = [];
Aeq = [];
beq = []; 

% check the supplied gradients

check_options=optimoptions("fmincon",FiniteDifferenceType="central");

valid_cost=checkGradients(@(z) CostAndGrad(z,param), z_guess,check_options,'Display','on');
valid_con=checkGradients(@(z) NonlinConstraintAndGrad(z,param), z_guess,check_options,'IsConstraint',true,'Display','on');

fprintf('CheckGradients on cost function: %g\n',valid_cost)
fprintf('CheckGradients on inequality constraints: %g\n',valid_con(1))
fprintf('CheckGradients on equality constraints: %g\n\n',valid_con(2))

% minimize ObjFun

tic
[z,fval]=fmincon(ObjFun,z_guess,A,b,Aeq,beq,lb,ub,NLcon,options);
ela_time=toc;

fprintf('\nTime required for optimisation: %g s\n',ela_time)
fprintf('Initial cost: J=%g\nFianl cost: J=%g\n\n',J_in,fval)

% extract states, control and tf from z

tf_opt=z(end);
z(end)=[];

xi=zeros(N+1,nxi); 
u=zeros(N,nu);
for ii=0:N
    xi(ii+1,:)=z((1+ii*(nu+nxi)):(nxi+ii*(nu+nxi)));
end    
for ii=0:N-1
    u(ii+1,:)=z((1+nxi+ii*(nu+nxi)):(nxi+nu+ii*(nu+nxi)));    
end

xi_opt=xi;
u_opt=u;

end