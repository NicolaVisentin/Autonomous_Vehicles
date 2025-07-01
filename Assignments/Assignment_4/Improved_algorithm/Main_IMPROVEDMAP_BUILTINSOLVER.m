clear
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                    %
% Test with improved map processing algorithm and built-in A* solver %
%                                                                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Cool algorithm

% Load image and build map

load('PROVA.mat')  % gray scale map of map1
[BW,not_clearanced,nodes,edges]=PROCESSIMPROVEDMAP(map_BW,50,50);
% load('PROCESSED_1000x1000.mat')
PlotMap(not_clearanced)
pause(2)
[n,m]=size(BW);

% Generate graph object

graphObj=navGraph(nodes,edges);

% Ask positions to the user

points=nan(2,2);
fig=figure;
imshow(BW)
hold on
for ii=1:2
    if ii==1
        title('Select start point:');
    else
        title('Select goal point:');
    end
    [x,y]=ginput(1);
    if ii==1
        plot(x,y,'ro','MarkerSize',10,'LineWidth',2);
    else
        plot(x,y,'go','MarkerSize',10,'LineWidth',2);
        pause(1.5)
    end
    points(ii,:)=[x,y];
end
close(fig)

start_i=round(points(1,2));
start_j=round(points(1,1));
goal_i=round(points(2,2));
goal_j=round(points(2,1));

% Make sure index is within the borders

start_i=min(max(start_i,1),n);    
start_j=min(max(start_j,1),m);
goal_i=min(max(goal_i,1),n);
goal_j=min(max(goal_j,1),m);

% Start and goal in terms of indices (i,j)

start=[start_i start_j];
goal=[goal_i goal_j];

start=[19 1];  % start for map 1 (50x50)
goal=[19 46];  % goal for map 1 (50x50)

% start=[34 3];  % start for map 1 (100x100)
% goal=[34 92];  % goal for map 1 (100x100)
 
% Start and goal in terms of index start_idx/goal_idx inside "nodes"

[~,start_idx]=min(vecnorm(nodes-start,2,2));
[~,goal_idx]=min(vecnorm(nodes-goal,2,2));

% Generate A* planner object and run the algorithm

tic
planner=plannerAStar(graphObj,'HeuristicCostFcn',@nav.algs.distanceEuclidean);
[pathOutput,info]=plan(planner,start_idx,goal_idx);
elatime=toc;

% Show the result

start=nodes(start_idx,:);
goal=nodes(goal_idx,:);

figure
hold on
PlotMap(BW)
hold on
plot(pathOutput(:,2),pathOutput(:,1),'r','LineWidth',2)
splot=plot(start(2),start(1),'bo','MarkerFaceColor','b');
gplot=plot(goal(2),goal(1),'go','MarkerFaceColor','g');
xlabel('jj')
ylabel('ii')
title('A* solution path')
axis tight

fprintf(['Optimal path computed (A*) in: ',num2str(elatime),' seconds\n'])
fprintf(['Path cost: ',num2str(info.PathCost),'\n'])
fprintf(['Explored nodes: ',num2str(info.NumExploredStates),'\n\n'])

%% Shit algorithm

[map_bin,G_diag,start,goal,N_free]=MapElaboration('map_1_d.png',50,50,0.99,1);

start=[19 1];  % start for map 1 (50x50)
goal=[19 46];  % goal for map 1 (50x50)

% start=[34 3];  % start for map 1 (100x100)
% goal=[34 92];  % goal for map 1 (100x100)

[cost,N_nod]=AStar(map_bin,G_diag,start,goal);
fprintf('\nA* with diagonal movements: \n\tlength of the optimal path: %g \n\tnodes evaluated: %g/%g\n',cost,N_nod,N_free)
%saveas(gcf,'A_1.png');
pause(1)

%% Super cool algorithm (Matlab does all the job)

% Compute graph (map object)

load('PROVA.mat')  % gray scale map of map1
n=50;  % new resolution
m=50;  % new resolution
BW=PROCESSMAPONLYNODES(map_BW,n,m);
tic
mapObj=binaryOccupancyMap(BW,1);
elatime=toc;
fprintf(['Map generated in ',num2str(elatime),' seconds\n\n'])

% Ask positions to the user

points=nan(2,2);
fig=figure;
imshow(~BW)
hold on

title('Select the start point:');
[x,y]=ginput(1);
points(1,:)=[x,y];
plot(x,y,'ro','MarkerSize',10,'LineWidth',2);
title('Select goal point:');
[x,y]=ginput(1);
points(2,:)=[x,y];
plot(x,y,'go','MarkerSize',10,'LineWidth',2);
pause(1.5)
close(fig)

start_i=round(points(1,2));
start_j=round(points(1,1));
goal_i=round(points(2,2));
goal_j=round(points(2,1));

% Make sure index is within the borders

start_i=min(max(start_i,1),n);    
start_j=min(max(start_j,1),m);
goal_i=min(max(goal_i,1),n);
goal_j=min(max(goal_j,1),m);

% Assign start and goal

start=[start_i start_j];
goal=[goal_i goal_j];

start=[19 1];  % start for map 1 (50x50)
goal=[19 46];  % goal for map 1 (50x50)

% start=[34 3];  % start for map 1 (100x100)
% goal=[34 92];  % goal for map 1 (100x100)

% Generate A* planner object and run the algorithm

tic
planner=plannerAStarGrid(mapObj);
[pathOutput,info]=plan(planner,start,goal);
elatime=toc;
fprintf(['Optimal path computed (A*) in: ',num2str(elatime),' seconds\n'])
fprintf(['Path cost: ',num2str(info.PathCost),'\n'])
fprintf(['Explored nodes: ',num2str(info.NumNodesExplored),'\n\n'])
figure
show(planner)
legend