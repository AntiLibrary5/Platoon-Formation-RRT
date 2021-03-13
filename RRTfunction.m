function [tree, path] = RRTfunction(K, xMin, xMax, yMin, yMax, xInit, yInit, xGoal, yGoal, thresh, yLeader, xy_traffic)

tree.vertex(1).x = xInit;
tree.vertex(1).y = yInit;
tree.vertex(1).xPrev = xInit;
tree.vertex(1).yPrev = yInit;
tree.vertex(1).dist=0;
tree.vertex(1).ind = 1; tree.vertex(1).indPrev = 0;

xArray=xInit; yArray = yInit;

figure(1); hold on; grid on;
plot(xInit, yInit, 'ko', 'MarkerSize',10, 'MarkerFaceColor','k');
plot(xGoal, yGoal, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');

path_color = {'b','r','g','k','y'};
color = randi(5);

xTrafficMin =  xy_traffic(1)-1.25;
xTrafficMax =  xy_traffic(1)+1.25;
yTrafficMin =  xy_traffic(2)-2.5;
yTrafficMax = xy_traffic(2)+2.5;

for iter = 2:K
    %make the tree randomly
%     while 1
        xRand = (xMax-xMin)*rand;
        yRand = (yLeader-yMin)*rand;%(yMax-yMin)*rand;
%         %avoid traffic
%         if xRand>=xTrafficMin && xRand<=xTrafficMax &&...
%                 yRand>=yTrafficMin && yRand<=yTrafficMax
%             xRand = (xMax-xMin)*rand;
%             yRand = (yMax-yMin)*rand;%(yLeader-yMin)*rand;
%         end
%            %      (xRand>=xTrafficMin && xRand<=xTrafficMax))...%top or bottom of box
%            %                (yRand>=yTrafficMin && yRand<=yTrafficMax)) %parallel of box
%         if ((xRand>=xTrafficMax && xRand>=xTrafficMin) ||...%right of box
%                 (xRand<=xTrafficMax && xRand<=xTrafficMin))...%left of box
%                 &&...
%                 ((yRand>=yTrafficMax && yRand>=yTrafficMin) ||...%top of box
%                 (yRand<=yTrafficMax && yRand<=yTrafficMin))...%bottom of box
% 
%             break
%         end
%     end            
      
    dist = Inf*ones(1,length(tree.vertex));
    for j = 1:length(tree.vertex)
        dist(j) = sqrt( (xRand-tree.vertex(j).x)^2 + (yRand-tree.vertex(j).y)^2 );
    end
    [val, ind] = min(dist);
       
    tree.vertex(iter).x = xRand; tree.vertex(iter).y = yRand;
    tree.vertex(iter).dist = val;
    tree.vertex(iter).xPrev = tree.vertex(ind).x;
    tree.vertex(iter).yPrev = tree.vertex(ind).y;
    tree.vertex(iter).ind = iter; tree.vertex(iter).indPrev = ind;
    %exit loop if goal within threshold of our current node
    if sqrt( (xRand-xGoal)^2 + (yRand-yGoal)^2 ) <= thresh
        plot([tree.vertex(iter).x; tree.vertex(ind).x],[tree.vertex(iter).y; tree.vertex(ind).y], 'r');
        break
    end
    %plotting the current node with previous node
    plot([tree.vertex(iter).x; tree.vertex(ind).x],[tree.vertex(iter).y; tree.vertex(ind).y], 'r');
    pause(0);
end

%if path found, traces back the path
if iter < K
    path.pos(1).x = xGoal; path.pos(1).y = yGoal;
    path.pos(2).x = tree.vertex(end).x; path.pos(2).y = tree.vertex(end).y;
    pathIndex = tree.vertex(end).indPrev;

    j=0;
    while 1
        path.pos(j+3).x = tree.vertex(pathIndex).x;
        path.pos(j+3).y = tree.vertex(pathIndex).y;
        pathIndex = tree.vertex(pathIndex).indPrev;
        if pathIndex == 1
            break
        end
        j=j+1;
    end

    path.pos(end+1).x = xInit; path.pos(end).y = yInit;
%plotting the final path found
    for j = 2:length(path.pos)
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], path_color{color}, 'Linewidth', 3);
    %     plot([tree.vertex(i).x; tree.vertex(ind).x],[tree.vertex(i).y; tree.vertex(ind).y], 'r');
    %     pause(0);
    end
else
    disp('No path found. Increase number of iterations and retry.');
end
end