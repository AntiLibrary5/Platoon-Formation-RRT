close all;
clear all
Yleftlane = [-20:5:20];
Xleftlane = zeros(size(Yleftlane));

dX = 2.5;
dY = 5;
figure(1)
set(gcf,'Units','centimeters','position',[2 2 10 15])

plot(Xleftlane,Yleftlane);
axis([0 dX*3 -20 20])
hold on;
plot(Xleftlane+dX,Yleftlane,'k');
plot(Xleftlane+dX*2,Yleftlane,'k');
plot(Xleftlane+dX*3,Yleftlane,'k');
a = -5:5;
for i=1:length(Yleftlane)
    figure(1)  
    hold on
    plot(Yleftlane,Xleftlane+(a(i)*dY),'k')
end

figure(1)
hold on;
xy_leader = ginput(1);
plot(xy_leader(1),xy_leader(2),'b+','LineWidth',3);

n_slave = input('How many slaves ');
xy_slave = zeros(n_slave,2);

for i=1:n_slave
    figure(1)
    hold on;
    xy_slave(i,:) = ginput(1);
    plot(xy_slave(i,1),xy_slave(i,2),'*','LineWidth',2);
end
    

