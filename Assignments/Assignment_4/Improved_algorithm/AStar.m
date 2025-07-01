function [dst,nnodes]=AStar(map,G,start,goal)

% A* algorithm.
%
% Inputs:
%   map...........input map: nxm binary matrix with 1 (free nodes) and 0 
%                 (obstacle nodes)
%   G.............edge matrix
%   start,goal....start and goal nodes (i,j) position: [i j] vectors
% Outputs:
%   dst...........length of the computed path
%   nodes.........number of nodes that were evaluated


% Extract relevant informations

n=size(map,1);
m=size(map,2);

start_lin=sub2ind([n m],start(1),start(2)); % linear index 
goal_lin=sub2ind([n m],goal(1),goal(2));    % linear index

% Allocate variables

dist=inf*ones(n*m,1);      % distance (along a certain path) from the start: n*m vector
prec=inf*ones(n*m,1);      % collects the parent of each node: n*m vector
status=-1*ones(n*m,1);     % node list: -1=not visited, 1=to visit (alive set), 0=visited

% Initialisation

figure(1001)               % open figure to show how the algorithm runs
PlotMap(map)
hold on
plot(start(2),start(1),'yo','MarkerFaceColor','y')
plot(goal(2),goal(1),'ro','MarkerFaceColor','r')
title('A* path research')

act_node=start_lin;       % choose initial node
dist(start_lin)=0;        % set its distance to zero

[~,con_nodes]=find(G(act_node,:)>0);   % find nodes connected to the active node (children)
status(con_nodes)=1;                   % set them to be visited (insert in the alive set)
status(act_node)=0;                    % set start node as visited
figure(1001)
plot(start(2),start(1),'bx')

% Iterations

while any(status==1) && act_node~=goal_lin   % stop when there are no nodes left in the alive set OR when we meet the goal
    
    % check all children
    n_con=length(con_nodes);
    while n_con>0

        [ii_act,jj_act]=ind2sub([n m],act_node);
        [ii_con,jj_con]=ind2sub([n m],con_nodes(n_con));
        
        if abs(ii_act-ii_con)==1 && abs(jj_act-jj_con) == 1    % check diagonal nodes
            if dist(con_nodes(n_con))>dist(act_node)+sqrt(2)   % if the current children's dist (along some previously computed path) is bigger (so less optimal) than the one that we have in the current path (i.e. passing for act_node)...
                dist(con_nodes(n_con))=dist(act_node)+sqrt(2); % ... update the distance of this children...
                prec(con_nodes(n_con))=act_node;               % ... and set act_node to be its parent
            end
        else                                                   % check transversal nodes
            if dist(con_nodes(n_con))>dist(act_node)+1         % if the current children's dist (along some previously computed path) is bigger (so less optimal) than the one that we have in the current path (i.e. passing for act_node)...
                dist(con_nodes(n_con))=dist(act_node)+1;       % ... update the distance of this children...
                prec(con_nodes(n_con))=act_node;               % ... and set act_node to be its parent
            end
        end
        n_con=n_con-1;  % move to the next children

    end

    % update the "searching" visualisation
    [ii_act_node,jj_act_node]=ind2sub([n m],act_node);
    figure(1001)
    plot(jj_act_node,ii_act_node,'bx')
    drawnow

    % pick the next active node from the alive set: choose the one with the
    % lower cost (distance) also accounting for the cost-to-arrival (Euclidean distance)
    alive=find(status==1);       % indices of the nodes in the alive set
    [row_con,col_con]=ind2sub([n m],alive);
    [row_goal,col_goal]=ind2sub([n m],goal_lin);
    H=sqrt((row_con-row_goal).^2+(col_con-col_goal).^2);

    [~,temp]=min(dist(alive)+H);  % position (inside "alive") of the lowest cost alive node
    act_node=alive(temp);        % move to the next node...
    status(act_node)=0;          % ... set it as visited

    % check all children
    [~,con_nodes]=find(G(act_node,:)>0);   % find nodes connected to the active node (children)
    n_con=length(con_nodes);
    while n_con>0

        if status(con_nodes(n_con))~=0  % if not visited yet...
            status(con_nodes(n_con))=1;    % ... set it to be visited (add to alive set, if not already there)
        end
        n_con=n_con-1;  % move to the next children

    end

end

% Recontstruct shortest path

if dist(goal_lin)<inf     % if goal node was reached
 
    % build optimal path
    path=goal_lin;    % start filling the optimal solution vector: contains the squence, from goal to start, of the nodes of the optimal path
    while path(end)~=start
        path=[path prec(path(end))];      
    end
    path=flip(path);  % reverse: from start to goal
    
    % plot optimal path
    for kk=1:length(path)
        [ii,jj]=ind2sub([n m],path(kk));
        figure(1001)
        plot(jj,ii,'rx','LineWidth',3)
    end
    hold off

else      % if goal node was not reached

    fprintf('\nNo solution found\n\n')

end

% Store outputs

dst=dist(goal_lin);         % length of the path
nnodes=sum(status==0);  % number of visited nodes

end