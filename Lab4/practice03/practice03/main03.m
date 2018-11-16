% MAIN03.M
%
% "Introduction to robotics" exercise: trapezoidal velocity profile computation

% it clears the workspace 
clear all
close all
clc

% variables initialization 
q_i = [0 -20 30]'/180*pi;
q_f = [10 -40 60]'/180*pi;
tf = 2;
dq_c = [1.5 1.6 1.8]'.*abs(q_f-q_i)/tf; % NOTE: A.*B multiplies A and B element by element. It is different from A*B that is the matrix product of A and B

Ts = 1e-3;

n = size(q_i,1);
t = 0:Ts:1.5*tf;
N = length(t);
q = zeros(n,N);
dq = zeros(n,N);
ddq = zeros(n,N);

for i=1:N
    [q(:,i),dq(:,i),ddq(:,i)] = trapezoidal(q_i,q_f,dq_c,tf,t(i));
end

figure
subplot(311)
plot(t,q,'.')
title('Position')
ylabel('pos')
subplot(312)
plot(t,dq)
title('Velocity')
ylabel('vel')
subplot(313)
plot(t,ddq,'-.')
title('Acceleration')
ylabel('acc')
