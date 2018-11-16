close all;
clear all;

xin = [0; 1; 0];
xfin = [15; 4; 15];

tfin = 2;
vcruise = [10; 3; 10];
tcurr = 0;
tsampling = 0.01;

for i=1:3
    [pos, vel, acc] = ComputeTrajectory(xin(i), xfin(i), tfin, vcruise(i), tcurr, tsampling);
    figure(i);
    subplot(1,3,1);
    plot(pos);
    title('Position');
    subplot(1,3,2);
    plot(vel);
    title('Velocity');
    subplot(1,3,3);
    plot(acc);
    title('Acceleration');
end

