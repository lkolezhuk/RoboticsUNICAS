function  T0 = DirectKinematics(DH)

% Homogeneous transformation matrices with respect to the base frame 
% T = Homogeneous_dh(DH_i)
%
% input:
%       DH     dim nx4     Denavit-Hartenberg table
%
% output:
%       T      dim 4x4xn   Homogeneous transformation matrices with respect to the base frame   


n = size(DH,1);
T0 = zeros(4,4,n);

% computation of homogeneous transformation matrix between consecutive frames
for i=1:n
   
    T(:,:,i) = Homogeneous(DH(i,:));
    
end

% computation of homogeneous transformation matrices with respect to the base frame. 
%T0(:,:,n) contains the homogeneous transformation matrix between the end-effector frame and the base one

T0(:,:,1) = T(:,:,1);

for i=2:n

    T0(:,:,i) = T0(:,:,i-1) * T(:,:,i);
    
end


end
