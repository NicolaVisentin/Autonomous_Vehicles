function [J,DJ] = CostAndGrad(z,param)

% Gives the cost function J and, if requested, also its gradient

% extract parameters

N = param.N;
nxi = param.nxi;
nu = param.nu;
L = param.L;
phi = param.phi;

% extract final time

tf=z(end);
z(end)=[];   % remove tf

% extract states and control from z

xi = zeros(nxi,N+1); u = zeros(nu,N);
for ii = 0:N
    xi(:,ii+1) = z((1 + ii*(nu + nxi)):(nxi + ii*(nu + nxi)));
end    
for ii = 0:N-1
    u(:,ii+1) = z((1 + nxi + ii*(nu + nxi)):(nxi + nu + ii*(nu + nxi)));    
end
    
% compute the value of cost function J

h=tf/N;

J = 0;
for ii = 1:N
    J = J + h*L(xi(:,ii),u(:,ii));
end
J = J + phi(xi(:,end),tf);

% compute the value of the gradient of J

if nargout > 1

    dLdxi = param.dLdxi;
    dLdu = param.dLdu;
    dphidxi = param.dphidxi;
    dphidtf = param.dphidtf;
    L = param.L;
    
    DJ = zeros(size(z));   % without counting tf contribution
    DJ_tf=0;
    for ii = 0:N-1
        DJ((1 + ii*(nu + nxi)):(nxi + ii*(nu + nxi)),1) = h*dLdxi(xi(:,ii + 1),u(:,ii + 1));
        DJ((1 + nxi + ii*(nu + nxi)):(nxi + nu + ii*(nu + nxi)),1) = h*dLdu(xi(:,ii + 1),u(:,ii + 1));   
        DJ_tf = DJ_tf + L(xi(:,ii+1),u(:,ii+1));  % update contribution for tf (DJ(end))
    end
    DJ(end - nxi + 1:end,1) = dphidxi(xi(:,end));
    
    DJ=[DJ; dphidtf+1/N*DJ_tf];      % now add tf contribution
    
end

end