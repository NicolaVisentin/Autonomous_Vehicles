function [map_out,G,start,goal,nfree]=MapElaboration(map_in,n,m,BW_tresh,diag)

% Elaborates the provided NxMx3 RGB map.
%
% Inputs:
%   map_in........provided map: NxMx3 RGB with gray and black (obstacles),
%                 white (free), green (start) and red (goal) pixels
%   n,m...........new dimensions
%   BW_tresh......grey scale treshold for distinguishing obstacles from
%                 free nodes
%   diag..........1 for also accounting for diagonal movement, 0 for only
%                 transversal
%
% Outputs:
%   map_out.......binary output rescaled matrix: nxm binary matrix with 1
%                 (free nodes) and 0 (not free nodes)
%   G.............edge matrix
%   start,goal....positions of start and final point: (i,j) indices in a
%                 vector [i j]
%   nfree.........number of free nodes


% Find start/goal coordinates

map_RGB=imread(map_in);     % RGB: (N,M,3) matrix
map_HSV=rgb2hsv(map_RGB);   % HSV: (N,M,3) matrix

N=size(map_HSV,1);          % original number of pixels
M=size(map_HSV,2);          % original number of pixels

H=map_HSV(:,:,1);           % extract hue values

[start_i,start_j]=find(H>0.2 & H<0.5);       % coordinates in original image (red)
[goal_i,goal_j]=find(H>0.9 | H<0.1 & H~=0);  % coordinates in original image (green)

map_RGB(start_i,start_j,:)=255;   % set to white (free) the pixels of the starting positions
map_RGB(goal_i,goal_j,:)=255;   % set to white (free) the pixels of the goal positions

start_i=round(mean(start_i)*n/N);  % rescale coordinates
start_j=round(mean(start_j)*m/M);
goal_i=round(mean(goal_i)*n/N);
goal_j=round(mean(goal_j)*m/M);

start_i=min(max(start_i,1),n);     % make sure index is within the borders
start_j=min(max(start_j,1),m);
goal_i=min(max(goal_i,1),n);
goal_j=min(max(goal_j,1),m);

start=[start_i start_j];
goal=[goal_i goal_j];

% Create nodes

map_resized=imresize(map_RGB,[n m],'Method','nearest');     % resize to (n,m,3), with n=m=30
map_BW=im2gray(map_resized);                                % convert in gray scale: (n,m) matrix
map_bin=imbinarize(map_BW,BW_tresh);                        % logic: 1 if node (ii,jj) is free, 0 if obstacle; (n,m) matrix

% Create edge matrix

G=-1*ones(n*m);     % initialise it to all -1, (n*m,n*m) matrix
for ii=1:n
    for jj=1:m

        % consider only free nodes
        if map_bin(ii,jj)==1

            % node itself (diagonal of G)
            indx_lin=sub2ind([n m],ii,jj);              % linear index (inside BW) of the node
            G(indx_lin,indx_lin)=0;

            % node above
            if ii-1>=1 && map_bin(ii-1,jj)==1
                indx_lin_above=sub2ind([n m],ii-1,jj);  % linear index of the node above
                G(indx_lin,indx_lin_above)=1;
            end

            % node below
            if ii+1<=n && map_bin(ii+1,jj)==1
                indx_lin_below=sub2ind([n m],ii+1,jj);  % linear index of the node below
                G(indx_lin,indx_lin_below)=1;
            end

            % node on the left
            if jj-1>=1 && map_bin(ii,jj-1)==1
                indx_lin_left=sub2ind([n m],ii,jj-1);   % linear index of the node on the left
                G(indx_lin,indx_lin_left)=1;
            end

            % node on the right
            if jj+1<=m && map_bin(ii,jj+1)==1
                indx_lin_right=sub2ind([n m],ii,jj+1);  % linear index of the node on the right
                G(indx_lin,indx_lin_right)=1;
            end

            % node top-right (only if diag==1)
            if jj+1<=m && ii-1>=1 && map_bin(ii-1,jj+1)==1 && diag==1
                indx_lin_topright=sub2ind([n m],ii-1,jj+1);   % linear index of the node on top-right
                G(indx_lin,indx_lin_topright)=sqrt(2);
            end

            % node top-left (only if diag==1)
            if jj-1>=1 && ii-1>=1 && map_bin(ii-1,jj-1)==1 && diag==1
                indx_lin_topleft=sub2ind([n m],ii-1,jj-1);   % linear index of the node on top-left
                G(indx_lin,indx_lin_topleft)=sqrt(2);
            end

            % node bottom-left (only if diag==1)
            if jj-1>=1 && ii+1<=n && map_bin(ii+1,jj-1)==1 && diag==1
                indx_lin_botleft=sub2ind([n m],ii+1,jj-1);   % linear index of the node on bottom-left
                G(indx_lin,indx_lin_botleft)=sqrt(2);
            end

            % node bottom-right (only if diag==1)
            if jj+1<=m && ii+1<=n && map_bin(ii+1,jj+1)==1 && diag==1
                indx_lin_botright=sub2ind([n m],ii+1,jj+1);   % linear index of the node on bottom-right
                G(indx_lin,indx_lin_botright)=sqrt(2);
            end

        end

    end
end

% Assign outputs

map_out=map_bin;
nfree=sum(sum(map_bin));

end