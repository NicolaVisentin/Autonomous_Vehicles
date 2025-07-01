%% Figure 1

clear
close all
clc

% Load image and build map

[map_bin,G_trans,start,goal,N_free]=MapElaboration('map_1_d.png',50,50,0.99,0);
[~,G_diag,~,~,~]=MapElaboration('map_1_d.png',50,50,0.99,1);

figure
PlotMap(map_bin)
hold on
plot(start(2),start(1),'yo','MarkerFaceColor','y')
plot(goal(2),goal(1),'ro','MarkerFaceColor','r')
title('Map')
hold off

% Dijkstra

[cost,N_nod]=Dijkstra(map_bin,G_trans,start,goal);
fprintf('Dijkstra: \n\tlength of the optimal path: %g \n\tnodes evaluated: %g/%g',cost,N_nod,N_free)
%saveas(gcf,'Dij_simple1.png');

% Dijkstra with diagonals

[cost,N_nod]=Dijkstra(map_bin,G_diag,start,goal);
fprintf('\nDijkstra with diagonal movements: \n\tlength of the optimal path: %g \n\tnodes evaluated: %g/%g\n',cost,N_nod,N_free)
%saveas(gcf,'Dij_1.png');

% Astar

[cost,N_nod]=AStar(map_bin,G_trans,start,goal);
fprintf('\nA*: \n\tlength of the optimal path: %g \n\tnodes evaluated: %g/%g\n',cost,N_nod,N_free)
%saveas(gcf,'A_simple1.png');

% Astar with diagonal

[cost,N_nod]=AStar(map_bin,G_diag,start,goal);
fprintf('\nA* with diagonal movements: \n\tlength of the optimal path: %g \n\tnodes evaluated: %g/%g\n',cost,N_nod,N_free)
%saveas(gcf,'A_1.png');

%% Figure 2

clear
close all
clc

% Load image and build map

[map_bin,G_trans,start,goal,N_free]=MapElaboration('map_2_d.png',50,50,0.99,0);
[~,G_diag,~,~,~]=MapElaboration('map_2_d.png',50,50,0.99,1);

figure
PlotMap(map_bin)
hold on
plot(start(2),start(1),'yo','MarkerFaceColor','y')
plot(goal(2),goal(1),'ro','MarkerFaceColor','r')
title('Map')
hold off

% Dijkstra

[cost,N_nod]=Dijkstra(map_bin,G_trans,start,goal);
fprintf('Dijkstra: \n\tlength of the optimal path: %g \n\tnodes evaluated: %g/%g',cost,N_nod,N_free)
%saveas(gcf,'Dij_simple2.png');

% Dijkstra with diagonals

[cost,N_nod]=Dijkstra(map_bin,G_diag,start,goal);
fprintf('\nDijkstra with diagonal movements: \n\tlength of the optimal path: %g \n\tnodes evaluated: %g/%g\n',cost,N_nod,N_free)
%saveas(gcf,'Dij_2.png');

% Astar

[cost,N_nod]=AStar(map_bin,G_trans,start,goal);
fprintf('\nA*: \n\tlength of the optimal path: %g \n\tnodes evaluated: %g/%g\n',cost,N_nod,N_free)
%saveas(gcf,'A_simple2.png');

% Astar with diagonal

[cost,N_nod]=AStar(map_bin,G_diag,start,goal);
fprintf('\nA* with diagonal movements: \n\tlength of the optimal path: %g \n\tnodes evaluated: %g/%g\n',cost,N_nod,N_free)
%saveas(gcf,'A_2.png');

%% Figure 3

clear
close all
clc

% Load image and build map

[map_bin,G_trans,start,goal,N_free]=MapElaboration('map_3_d.png',50,50,0.99,0);
[~,G_diag,~,~,~]=MapElaboration('map_3_d.png',50,50,0.99,1);

figure
PlotMap(map_bin)
hold on
plot(start(2),start(1),'yo','MarkerFaceColor','y')
plot(goal(2),goal(1),'ro','MarkerFaceColor','r')
title('Map')
hold off

% Dijkstra

[cost,N_nod]=Dijkstra(map_bin,G_trans,start,goal);
fprintf('Dijkstra: \n\tlength of the optimal path: %g \n\tnodes evaluated: %g/%g',cost,N_nod,N_free)
%saveas(gcf,'Dij_simple3.png');

% Dijkstra with diagonals

[cost,N_nod]=Dijkstra(map_bin,G_diag,start,goal);
fprintf('\nDijkstra with diagonal movements: \n\tlength of the optimal path: %g \n\tnodes evaluated: %g/%g\n',cost,N_nod,N_free)
%saveas(gcf,'Dij_3.png');

% Astar

[cost,N_nod]=AStar(map_bin,G_trans,start,goal);
fprintf('\nA*: \n\tlength of the optimal path: %g \n\tnodes evaluated: %g/%g\n',cost,N_nod,N_free)
%saveas(gcf,'A_simple3.png');

% Astar with diagonal

[cost,N_nod]=AStar(map_bin,G_diag,start,goal);
fprintf('\nA* with diagonal movements: \n\tlength of the optimal path: %g \n\tnodes evaluated: %g/%g\n',cost,N_nod,N_free)
%saveas(gcf,'A_3.png');

%% Figure 4

clear
close all
clc

% Load image and build map

[map_bin,G_trans,start,goal,N_free]=MapElaboration('map_4_d.png',50,50,0.99,0);
[~,G_diag,~,~,~]=MapElaboration('map_4_d.png',50,50,0.99,1);

figure
PlotMap(map_bin)
hold on
plot(start(2),start(1),'yo','MarkerFaceColor','y')
plot(goal(2),goal(1),'ro','MarkerFaceColor','r')
title('Map')
hold off

% Dijkstra

[cost,N_nod]=Dijkstra(map_bin,G_trans,start,goal);
fprintf('Dijkstra: \n\tlength of the optimal path: %g \n\tnodes evaluated: %g/%g',cost,N_nod,N_free)
%saveas(gcf,'Dij_simple4.png');

% Dijkstra with diagonals

[cost,N_nod]=Dijkstra(map_bin,G_diag,start,goal);
fprintf('\nDijkstra with diagonal movements: \n\tlength of the optimal path: %g \n\tnodes evaluated: %g/%g\n',cost,N_nod,N_free)
%saveas(gcf,'Dij_4.png');

% Astar

[cost,N_nod]=AStar(map_bin,G_trans,start,goal);
fprintf('\nA*: \n\tlength of the optimal path: %g \n\tnodes evaluated: %g/%g\n',cost,N_nod,N_free)
%saveas(gcf,'A_simple4.png');

% Astar with diagonal

[cost,N_nod]=AStar(map_bin,G_diag,start,goal);
fprintf('\nA* with diagonal movements: \n\tlength of the optimal path: %g \n\tnodes evaluated: %g/%g\n',cost,N_nod,N_free)
%saveas(gcf,'A_4.png');