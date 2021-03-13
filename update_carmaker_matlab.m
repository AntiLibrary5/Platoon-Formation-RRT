function [s_update,t_update,v_update] = update_carmaker_matlab(tsv_leader,tsv_slave,tsv_traffic,id_slave,platoon_pos,time,time_last)

coder.extrinsic('RRTStar','input');


% -- Initialise output
s_update = tsv_slave(1);
t_update = tsv_slave(2);
v_update = tsv_slave(3);

frequency = 10000; %Hz
time_last= floor(time_last*frequency);
time_new = floor(time*frequency);

% if time_last~=time_new || time_new == time_last
    
    t = tsv_slave(1);
    s = tsv_slave(2);
    v = tsv_slave(3);
    
    K = 2000;
    
    % -- Research area for RRTstar
    xMin = 0;
    xMax = 3.5*3;
    yMin = min(s,tsv_leader(2)) - 10 ;
    yMax = max(s,tsv_leader(2)) + 10;
    
    % -- Init of RRTstar
    xInit = t;
    yInit = s;
    xGoal = tsv_leader(1);
    yGoal = tsv_leader(2) - platoon_pos*(5 - 1.5);
    
    thresh = 3;
    
    xy_leader = tsv_leader(1:2);
    xy_slave = tsv_slave(:,1:2);
    xy_traffic = tsv_traffic(:,1:2);
    [n_slave,~] = size(tsv_slave);
    %[n_traffic,~] = size(tsv_traffic);
    n_traffic = 1;
    
    path = RRTStar(K, xMin, xMax, yMin, yMax, xInit,...
        yInit, xGoal, yGoal, thresh, xy_leader, xy_traffic, xy_slave, n_slave, n_traffic);
    
    xPath = flip(path(:,1));
    yPath = flip(path(:,2));
        
    figure(1)
    plot(path(:,1),path(:,2),'--x')

   xynew = [0 0];
   eps = 0.01;
    x = xPath(2);
    y = yPath(2);
   
   d = sqrt((x-t)^2 + (y-s)^2);
   % Steer towards the first point on path with maximum step size of eps
   if d >= eps
       xynew(1) = t + ((x-t)*eps)/d;
       xynew(2) = s + ((y-s)*eps)/d;
   else
       xynew(1) = x;
       xynew(2) = y;
   end   
   xyPath = [xynew(1), xynew(2)];
  
    % Check output of path
    t_update = xyPath(1);
    s_update = xyPath(2);
    
% %         Check output of path
%     t_update = t + (tsv_leader(1)-t)*0.001
%     s_update =  s + (tsv_leader(2)-s)*0.001
    


    
    % -- Compute the speed -> wrong for the moment
    distance = sqrt(sum((tsv_leader(1:2)-tsv_slave(id_slave,1:2)).^2));
    if (yGoal-yInit)>4
        v_update = tsv_leader(3) + distance*0.2;
    else
        v_update = tsv_leader(3);
    end
    
    
% end

figure(1)
plot(tsv_slave(1),tsv_slave(2),'bl+');
hold on;
plot(t_update,s_update,'o');





end

