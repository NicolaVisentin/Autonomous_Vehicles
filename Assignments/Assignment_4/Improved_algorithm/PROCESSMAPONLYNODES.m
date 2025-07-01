function [BW,nodes,edges]=PROCESSMAPONLYNODES(map_BW,n,m)
%
% Elaborates the provided gray scale map.
%
%   [BW,not_clearanced]=PROCESSIMPROVEDMAP(map_BW,n,m)
%
% Inputs:
%   map_BW...........provided map: NxM gray scale image with gray and black 
%                    (obstacles), white (free)
%   n,m..............new dimensions of the map
%
% Outputs:
%   BW...............binary output rescaled matrix: nxm binary matrix with 0
%                    (free nodes) and 1 (not free nodes)


% Resize given map and assign free/occupied nodes

BW_tresh=0.99;
map_resized=imresize(map_BW,[n m],'Method','nearest');     % resize to (n,m,3), with n=m=30
map_BW=im2gray(map_resized);                               % convert in gray scale: (n,m) matrix
BW=imbinarize(map_BW,BW_tresh);                             
BW=~BW;                                                    % logic: 0 if node (ii,jj) is free, 1 if obstacle; (n,m) matrix