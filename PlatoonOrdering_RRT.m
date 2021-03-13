close all;
clear all

nLane = input('How many lanes? ');
dX = 2.5;
dY = 5;
lengthLane = input('length of platooning look-ahead distance? ');

Yleftlane = 0:5:lengthLane;
Xleftlane = 0:dX:nLane*dX;

[X_grid,Y_grid] = meshgrid(Xleftlane,Yleftlane);
[X_grid2,Y_grid2] = meshgrid(Yleftlane,Xleftlane);

figure(1)
set(gcf,'Units','centimeters','position',[2 2 10 15])
plot(X_grid,Y_grid,'k')
hold on
plot(Y_grid2,X_grid2,'k')

figure(1)
hold on;
xy_leader = ginput(1);
xi = find(X_grid(1,:) > xy_leader(1));
yi = find(Y_grid(:,1) > xy_leader(2));
xy_leader(1) = X_grid(1,xi(1)) - dX*0.5;
xy_leader(2) = Y_grid(yi(1),1) - dY*0.5;

plot(xy_leader(1),xy_leader(2),'b+','LineWidth',3);

d = imread ('BM.png');
e = imread ('PPP.jpg');
f = imread ('TRFC.jpg');

image (flipud(d), 'XData' , [xy_leader(1)-1 xy_leader(1)+1], 'YData' , [xy_leader(2)-2 xy_leader(2)+2]);
text(xy_leader(1)+0.5,xy_leader(2),'Leader','VerticalAlignment','top')

n_slave = input('How many slave vehicles in the scenario? ');
xy_slave = zeros(n_slave,2);

for i=1:n_slave
    figure(1)
    hold on;
    
    xy_slave(i,:) = ginput(1);
   
    xi = find(X_grid(1,:) > xy_slave(i,1));
    yi = find(Y_grid(:,1) > xy_slave(i,2));
    xy_slave(i,1) = X_grid(1,xi(1)) - dX*0.5;
    xy_slave(i,2) = Y_grid(yi(1),1) - dY*0.5;
    
    if xy_slave(i,:) == xy_leader
        error('Cannot be on same square as leader')
    end
    
    if i > 1
        for j=1:i-1
            if xy_slave(j,:) == xy_slave(i,:)
                error('Cannot have 2 slaves in same box')
            end
        end
    end
    
    tag = strcat({'Slave '},num2str(i));    
    plot(xy_slave(i,1),xy_slave(i,2),'r*','LineWidth',2);
    image (flipud(e), 'XData' , [xy_slave(i,1)-1 xy_slave(i,1)+1], 'YData' , [xy_slave(i,2)-2 xy_slave(i,2)+2]);
    text(xy_slave(i,1)+0.5,xy_slave(i,2),tag,'VerticalAlignment','top')
end


n_traffic = input('How many traffic objects? ');
if n_traffic ~= 0    
    xy_traffic = zeros(n_traffic,2);    
    for i=1:n_traffic
        figure(1)
        hold on;
        xy_traffic(i,:) = ginput(1);
        
        xi = find(X_grid(1,:) > xy_traffic(i,1));
        yi = find(Y_grid(:,1) > xy_traffic(i,2));
        xy_traffic(i,1) = X_grid(1,xi(1)) - dX*0.5;
        xy_traffic(i,2) = Y_grid(yi(1),1) - dY*0.5;
        
        tag = strcat({'Traffic '},num2str(i));
        plot(xy_traffic(i,1),xy_traffic(i,2),'ok','LineWidth',2);
        image (flipud(f), 'XData' , [xy_traffic(i,1)-1 xy_traffic(i,1)+1], 'YData' , [xy_traffic(i,2)-2 xy_traffic(i,2)+2]);
        text(xy_traffic(i,1)+0.5,xy_traffic(i,2),tag,'VerticalAlignment','top')
    end    
end

[SlavePosition,SlaveDistance,Weights] = RespectivePosition(xy_slave,xy_leader,n_slave);
temp_Weights = Weights; 
for i=1:n_slave
    [max_Weight,index_max(i)] = max(temp_Weights);
    temp_Weights(index_max(i))= -1;
end

% using RRT for path planning
K=2000;
xMin=0; xMax=nLane*dX;
yMin=0; yMax=lengthLane;
thresh=1; %acceptable threshold for solution
xGoal=xy_leader(1); yGoal=xy_leader(2)-5; %goal for planner

% xy_traffic = [0 0];
for i = 1:n_slave
        slave = index_max(i);
        xInit=xy_slave(slave,1); yInit=xy_slave(slave,2); %initial point for planner
        path(i) = RRTStar(K, xMin, xMax, yMin, yMax, xInit,...
            yInit, xGoal, yGoal, thresh, xy_leader, xy_traffic, xy_slave, n_slave, n_traffic);
        
        init_state(i).x = xy_slave(slave,1);
        init_state(i).y = xy_slave(slave,2);
        init_state(i).yaw = 0;
        init_state(i).v = 30/3.6;
        
        xy_slave(slave,1) = xGoal; %update new slave position
        xy_slave(slave,2) = yGoal;
        
        new_yGoal = xy_slave(slave,2)-(5);
        yGoal = new_yGoal;
end
target_speed = 30/3.6;

%     for i = 1:numel(path)
%         xPath = path(i).x;
%         yPath = path(i).y;
%         if length(xPath) > 3
%             [rx, ry] =  bspline_planning(xPath, yPath);
%             figure(1)
%             hold on
%             plot(rx,ry,'--r','LineWidth',2)
%             hold on
%             [xTrack, yTrack] = PathTracking(rx, ry, init_state(i), target_speed);
%             plot(xTrack,yTrack,'k','LineWidth',1)
%         else
%             hold on
%             figure(1)
%             plot(xPath,yPath,'--r','LineWidth',2)
% 
%             if length(xPath)==3
%                 xPath = [linspace(xPath(1),xPath(2),40) linspace(xPath(2),xPath(3),40)];
%                 yPath = [linspace(yPath(1),yPath(2),40) linspace(yPath(2),yPath(3),40)];
%                 [xTrack, yTrack] = PathTracking(xPath, yPath, init_state(i), target_speed);
% 	            figure(1)
%                 hold on
% 	            plot(xTrack,yTrack,'k','LineWidth',1)
%             elseif length(xPath)==2
%                 xPath = [linspace(xPath(1),xPath(2),40)];
%                 yPath = [linspace(yPath(1),yPath(2),40)];
%                 [xTrack, yTrack] = PathTracking(xPath, yPath, init_state(i), target_speed);
% 	            figure(1)
%                 hold on
% 	            plot(xTrack,yTrack,'k','LineWidth',1)
%             end
%                 
%         end
%     end
