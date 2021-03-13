function [Position,Distance,Weights] = RespectivePosition(xy_slave,xy_leader,n_slave)
        dist_leader = [xy_leader(1).*ones(n_slave,1) xy_leader(2).*ones(n_slave,1)];
        dist_leader = dist_leader-xy_slave;
        for i=1
            for j=1:n_slave
                if dist_leader(j,i)<0
                    S(j).lane = {'RL'};
                elseif dist_leader(j,i)>0
                    S(j).lane = {'LL'};
                elseif dist_leader(j,i)==0
                    S(j).lane = {'SL'};
                end
            end
        end
        for i=2
           for j=1:n_slave
                    if dist_leader(j,i)<0
                        S(j).Position = {'FR'};
                    elseif dist_leader(j,i)>0
                        S(j).Position = {'BK'};
                    elseif dist_leader(j,i)==0
                        S(j).Position = {'PL'};
                    end
           end
        end
Position = S;
Distance = dist_leader;
for i=1:n_slave
        if S(i).Position{1}=='BK'
            if S(i).lane{1}=='RL' 
                Weight.Slave(i)=1000;
            elseif S(i).lane{1}=='LL' 
                Weight.Slave(i)=1000;
            elseif S(i).lane{1}=='SL' 
                Weight.Slave(i)=10000;
            end
        elseif S(i).Position{1}=='PL'
            if S(i).lane{1}=='RL' 
                Weight.Slave(i)=50;
            elseif S(i).lane{1}=='LL' 
                Weight.Slave(i)=50;
            end
        elseif S(i).Position{1}=='FR'
            if S(i).lane{1}=='RL' 
                Weight.Slave(i)=10;
            elseif S(i).lane{1}=='LL' 
                Weight.Slave(i)=10;
            elseif S(i).lane{1}=='SL' 
                Weight.Slave(i)=2;
            end
        end
end
Y_Distance=abs(dist_leader(:,2));
for i = 1:n_slave
    if Position(i).Position{1}=='PL'
        Y_Distance(i) = 1;
    end
end
Weights=(Weight.Slave).*(1./Y_Distance)';
end
            
            
        