% MAIN05.M
%
% "Introduction to robotics" exercise: inverse kinematics algorithms
%  using the quaternion error 

% it clears the workspace 
clear
close all
clc

% robot variables initialization
init


algorithm='inverse';

if strcmp(algorithm,'inverse')
    K = 1*diag([10 10 10 10 10 10 10]);
    fprintf('\n Jacobian inverse algorithm \n')
else
    K = 10*diag([12 12 12 12 12 12 12]);
    fprintf('\n Jacobian transpose algorithm \n')
end


% simulation variables initialization 
tf = 2;
Ts = 1e-3;
r = 3;  % dim task  
n = size(DH,1);
t = 0:Ts:tf;
N = length(t);
xd  = zeros(r,N);
x   = zeros(r,N);
q   = zeros(n,N);
% q(:,1) = DH(:,4); 
q(:, 1) = [0 pi/4 0 pi/4 pi/3 pi/2 pi/2];
dq  = zeros(n,N);
quat = zeros(4,N);
error_pos = zeros(r,N);
error_quat = zeros(3,N);
error = zeros(6,N);

for i=1:N
    % desired task trajectory
    xd(:,i) = [0.4 0.5 0.4]';
    quat_d(:,i) = [0 0 0 1]';
    % direct kinematics
%     DH(:,4) = q(:,i);
%     T = DirectKinematics(DH); % Kukadirect
    T = kuka_directkinematics(q(:,i))
    x(1:3,i) = T(1:3,4);
    quat(:,i) = Rot2Quat(T(1:3,1:3));
    % Jacobian
%     J = Jacobian(DH); % KukaJ
    J = kuka_J(q(:, i));
%     J = [J(1:3,:); J(4:6,:)];
    % Inverse kinematics algorithm
    error_pos(:,i) = xd(:,i) - x(:,i);
    error_quat(:,i) = QuatError(quat_d(:,i),quat(:,i));
    error(:,i) = [error_pos(:,i);error_quat(:,i)];
        
    if strcmp(algorithm,'transpose')
        dq(:,i) = J'*K*error(:,i);
    else
        dq(:,i) = K*pinv(J)*error(:,i);
    end
    if i<N
        q(:,i+1) = q(:,i) + Ts*dq(:,i);
    end
end


figure
    subplot(411)
        plot(t,q)
        grid on
        ylabel('joint position')
        legend('q1','q2','q3')
    subplot(412)
        plot(t,dq)
        grid on
        ylabel('joint velocity')
        legend('dq1','dq2','dq3')
    subplot(413)
        plot(t,error(1:2,:))
        grid on
        ylabel('position error')
        legend('ep-x','ep-y')
    subplot(414)
        plot(t,error(3:5,:))
        grid on
        ylabel('orientation error')
        legend('eo-x','eo-y','eo-z')
        xlabel('time [s]')

figure
hold on
DH(:,4) = q(:,1);
DrawRobot(DH);
DH(:,4) = q(:,N);
DrawRobot(DH);