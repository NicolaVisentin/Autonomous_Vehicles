function [map_out,start,goal,nfree]=MapElaboration(map_in,n,m,BW_tresh)
%
% Elaborates the provided NxMx3 RGB map.
%
%   [map_out,start,goal,nfree]=MapElaboration(map_in,n,m,BW_tresh)
%
% Inputs:
%   map_in........provided map: NxMx3 RGB with gray and black (obstacles),
%                 white (free), green (start) and red (goal) pixels
%   n,m...........new dimensions
%   BW_tresh......grey scale treshold for distinguishing obstacles from
%                 free nodes
%
% Outputs:
%   map_out.......binary output rescaled matrix: nxm binary matrix with 1
%                 (free nodes) and 0 (not free nodes)
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
map_RGB(goal_i,goal_j,:)=255;     % set to white (free) the pixels of the goal positions

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

map_resized=imresize(map_RGB,[n m],'Method','nearest');     % resize to (n,m,3)
map_BW=im2gray(map_resized);                                % convert in gray scale: (n,m) matrix
map_bin=imbinarize(map_BW,BW_tresh);                        % logic: 1 if node (ii,jj) is free, 0 if obstacle; (n,m) matrix

% Assign outputs

map_out=map_bin;
nfree=sum(sum(map_bin));

end