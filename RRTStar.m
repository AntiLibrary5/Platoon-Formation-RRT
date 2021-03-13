% Brief description of algorithm: 
% 1. Pick a random node q_rand.
% 2. Find the closest node q_near from explored nodes to branch out from, towards
%    q_rand.
% 3. Steer from q_near towards q_rand: interpolate if node is too far away, reach
%    q_new. Check that obstacle is not hit.
% 4. Update cost of reaching q_new from q_near, treat it as Cmin. For now,
%    q_near acts as the parent node of q_new.
% 5. From the list of 'visited' nodes, check for nearest neighbors with a 
%    given radius, insert in a list q_nearest.
% 6. In all members of q_nearest, check if q_new can be reached from a
%    different parent node with cost lower than Cmin, and without colliding
%    with the obstacle. Select the node that results in the least cost and 
%    update the parent of q_new.
% 7. Add q_new to node list.
% 8. Continue until maximum number of nodes is reached or goal is hit.


function [path] = RRTfunction(K, xMin, xMax, yMin, yMax, xInit, yInit, xGoal, yGoal, thresh, xy_leader, xy_traffic, xy_slave, n_slave, n_traffic)

coder.extrinsic('dist','ccw','noCollision','ObstacleFree','steer');


x_min = xMin;
y_min = yMin;
x_max = xMax;
y_max = yMax;

for i = 1:n_traffic
traffic_obstacle(i).pos = [xy_traffic(i,1)-1.25 xy_traffic(i,2)-2.5 2.5 5 ];%[200,150,200,200];
end

% for i = 1:n_slave
% slave_obstacle(i).pos = [xy_slave(i,1)-1.25 xy_slave(i,2)-2.5 2.5 5 ];%[200,150,200,200];
% end

leader_obstacle.pos = [xy_leader(1)-1.25 xy_leader(2)-2.5 2.5 5 ];%[200,150,200,200];

obstacle = traffic_obstacle;

% for i = 1:n_slave
%     obstacle(i+n_traffic).pos = slave_obstacle(i).pos;
% end
    
obstacle(1+n_traffic).pos = leader_obstacle.pos;

EPS =1;%EPS=20 higher value will result in larger space explored faster thus faster solution, but possibly bad one
numNodes = K;        

GoalThresh = 1;

q_start.coord = [xInit yInit];
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = [xGoal yGoal];
q_goal.cost = 0;

nodes(1) = q_start;

% for i = 1:(n_traffic + n_slave)
% figure(1)
% axis([0 x_max 0 y_max])
% rectangle('Position',obstacle(i).pos);%,'FaceColor',[0 .5 .5]
% hold on
% end

% figure(1)
% plot(xGoal,yGoal,'k+','LineWidth',2);
% hold on;
% plot(xInit, yInit,'r.','LineWidth',2);

j = 1;

for i = 1:numNodes
    if mod(i,5)==0
        q_rand = [(rand(1)*x_max) (rand(1)*y_max)];%yLeader OR xy_leader(2)
    else
        q_rand = [xGoal yGoal];
    end
%     figure(1)
%     plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410])
    
    % Break if goal node is already reached
    %for j = 1:1:length(nodes)
    if sqrt((nodes(j).coord(1)-q_goal.coord(1))^2 + (nodes(j).coord(2)-q_goal.coord(2))^2 ) <= GoalThresh
        %if nodes(j).coord == q_goal.coord
       break
    end
    j = i+1;
    
    % Pick the closest node from existing list to branch out from
    ndist = [];
    for j = 1:length(nodes)
        n = nodes(j);
        tmp = dist(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx);
    q_new.coord = steer(q_rand, q_near.coord, val, EPS);


    if ObstacleFree(q_rand, q_near.coord, obstacle)
%         figure(1)
%         line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 2);
%         drawnow
%         hold on
        q_new.cost = dist(q_new.coord, q_near.coord) + q_near.cost;
 
        % Within a radius of r, find all existing nodes
        q_nearest = [];
        r = 60;%r=60
        neighbor_count = 1;
        for j = 1:length(nodes)
            if  ObstacleFree(nodes(j).coord, q_new.coord, obstacle) && dist(nodes(j).coord, q_new.coord) <= r
                q_nearest(neighbor_count).coord = nodes(j).coord;
                q_nearest(neighbor_count).cost = nodes(j).cost;
                neighbor_count = neighbor_count+1;
            end
         end
       
        
        % Initialize cost to currently known value
        q_min = q_near;
        C_min = q_new.cost;
        
        % Iterate through all nearest neighbors to find alternate lower
        % cost paths
        
        
        
        for k = 1:1:length(q_nearest)
            if ObstacleFree(q_nearest(k).coord, q_new.coord, obstacle) && q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord) < C_min
                q_min = q_nearest(k);
                C_min = q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord);
%                 line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], 'Color', 'g');                
%                 hold on
            end
        end
        
         % Update parent to least cost-from node
         for j = 1:1:length(nodes)
            if nodes(j).coord == q_min.coord
                q_new.parent = j;
            end
         end
        % Append to nodes
        nodes = [nodes q_new];       
              
      end
     end

%calculate distance from goal to all nodes in D
D = [];
for j = 1:1:length(nodes)
    tmpdist = dist(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
% ie min distance 
[val, idx] = min(D);
q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];


i=1;
while q_end.parent ~= 0
    start = q_end.parent;
%      figure(1)
%       line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color', 'r');
    hold on
    %path(counter).x1(i) = q_end.coord(1);
    path(i,1) = nodes(start).coord(1);
    %path(counter).y1(i) = q_end.coord(2);
    path(i,2) = nodes(start).coord(2);
    i=i+1;
    

    q_end = nodes(start);
end

% input('Function path has finished. Input to continue')

end

