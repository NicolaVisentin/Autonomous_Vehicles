function [C,Ceq,Dc,DCeq] = NonlinConstraintAndGrad(z,param)

% computes the value of nonlinear inequality constraint C(z)<=0 and nonlinear
% equality constraint Ceq(z)=0 for a given z, and, if required, also the 
% value of their gradients
%
%   [C,Ceq,Dc,DCeq] = NonlinConstraintAndGrad(z,param)
%
% Outputs
%   C:       value of C in nonlinear inequality constraint C(z)<0
%   Ceq:     value of Ceq in equality nonlinear constraint Ceq(z)=0
%   Dc:      value of the gradient of the inequality nonlinear constraint Dc(z)
%   DCeq:    value of the gradient of the equality nonlinear constraint Dcon(z)
 
% extract parameters

N = param.N;
nxi = param.nxi;
nu = param.nu;
xip = param.xip;
xi_in = param.xi_in;

tf=z(end);
h=tf/N;

% extract states and control from z

xi = zeros(nxi,N+1); 
u = zeros(nu,N);
for ii = 0:N
    xi(:,ii+1) = z((1 + ii*(nu + nxi)):(nxi + ii*(nu + nxi)));
end    
for ii = 0:N-1
    u(:,ii+1) = z((1 + nxi + ii*(nu + nxi)):(nxi + nu + ii*(nu + nxi)));    
end

% constraint functions

C = []; % here is room for inequality constraint
Ceq = zeros((N+1)*nxi,1);
Ceq(1:nxi) = xi_in - xi(:,1) ; % initial condition constraint

for ii = 1:N
    Ceq((1:nxi) + ii*nxi) = xi(:,ii) + h*xip(xi(:,ii),u(:,ii)) - xi(:,ii+1);    
end

% gradients of the constraints: it's a ( (N+1)*nxi, N*(nu*nxi)+nxi+1 ) matrix

dfdxi = param.dfdxi;
dfdu = param.dfdu;

if nargout > 2

    Dc = []; % here is room for inequality constraint
    DCeq = zeros((N+1)*nxi, N*(nu+nxi) + nxi + 1);
    DCeq(1:nxi,1:nxi) = - eye(nxi); 
    for ii = 1:N
        DCeq((1 + nxi +(ii - 1)*nxi):((nxi + ii*nxi)),(1 +(ii - 1)*(nxi+nu)):((ii - 1)*(nxi+nu) + nxi)) = eye(nxi) + h*dfdxi(xi(:,ii),u(:,ii));
        DCeq((1 + nxi +(ii - 1)*nxi):((nxi + ii*nxi)),(1 +(ii)*(nxi+nu)):((ii)*(nxi+nu) + nxi)) = - eye(nxi);
        DCeq((1 + nxi +(ii - 1)*nxi):((nxi + ii*nxi)),(1 +(ii - 1)*(nxi+nu) + nxi):((ii - 1)*(nxi+nu) + nxi + nu)) = h*dfdu(xi(:,ii),u(:,ii));
        DCeq((1 + nxi +(ii - 1)*nxi):((nxi + ii*nxi)),end) = 1/N*xip(xi(:,ii),u(:,ii));
    end
    DCeq = DCeq.';   % transpose it because ode works with time along the columns, not rows
    
end

end