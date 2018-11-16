function [ output_args ] = InverseKinematics( DHcurr )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
Ts = 0.001;
t_curr = 0;
t_fin = 10;
t = t_curr:t_sampling:t_fin;
N = length(t) - 1;
x_curr = DHcurr;
for i = 1:N
   x = [10 10 10];
   x_curr = ComputeDKTransform(x_curr);
   jac = GetJacobian(x);
   dq = pinv(jac)*(x-x_curr);
   
end

end

