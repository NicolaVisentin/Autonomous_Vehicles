function [z_bound]=AssignZbound(xi_bound,u_bound,tf_bound,N,nxi,nu)

% Takes the values xi_bound=[xi1_bound, x12_bound, ...],
% u_bound=[u1_bound, u2_bound, ...] and tf_bound and fills properly
% z_bound with them

% creates xi_bound and u_bound matrices (each i-th column is the bound for 
% each time instant on the i-th state (or i-th control)

xi_bound_mat=zeros(N+1,nxi);
for ii=1:nxi
    xi_bound_mat(:,ii)=xi_bound(ii)*ones(N+1,1);
end

u_bound_mat=zeros(N,nu);
for ii=1:nu
    u_bound_mat(:,ii)=u_bound(ii)*ones(N,1);
end

% initialise z_bound (without accounting for tf_bound)

z_bound=zeros(N*(nxi+nu)+nxi,1); 

% assign u_bound into z_bound

ii=0;
indx_flag=(nxi+1)+ii*(nxi+nu)+(nu-1);
while indx_flag<length(z_bound)-nxi
    z_bound((nxi+1)+ii*(nxi+nu):(nxi+1)+ii*(nxi+nu)+(nu-1))=u_bound_mat(ii+1,:);
    indx_flag=(nxi+1)+ii*(nxi+nu)+(nu-1);
    ii=ii+1;
end

% assign xi_bound into z_bound

ii=0;
indx_flag=1+ii*(nxi+nu)+(nxi-1);
while indx_flag<length(z_bound)
    z_bound(1+ii*(nxi+nu):1+ii*(nxi+nu)+(nxi-1))=xi_bound_mat(ii+1,:);
    indx_flag=1+ii*(nxi+nu)+(nxi-1);
    ii=ii+1;
end

% assign tf_bound into z_bound

z_bound=[z_bound; tf_bound];

end