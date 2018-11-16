function [ res ] = ComputeDKTransform( DH )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

[num_links,~] = size(DH);
disp(num_links);
aframes = cell(num_links);

aframes{1} = NextFrameTransform(DH, 1);
transformMatrix = aframes{1};

for i = 2:num_links
   aframes{i} = NextFrameTransform(DH, i);
   transformMatrix = transformMatrix * aframes{i};

end
% 
% transformMatrix = zeros(4,4);
% transformMatrix(1,1) = 0;
% transformMatrix(1,2) = sin(DH(1,4) + DH(2,4));
% transformMatrix(1,3) = cos(DH(1,4) + DH(2,4));
% transformMatrix(1,4) = DH(1,1)*cos(DH(1,4)) + DH(1,2)*cos(DH(1,4)+DH(2,4));
% transformMatrix(2,2) = -cos(DH(1,4) + DH(2,4));
% transformMatrix(2,3) = sin(DH(1,4) + DH(2,4));
% transformMatrix(2,4) = DH(1,1)*sin(DH(1,4)) + DH(1,2)*sin(DH(1,4)+ DH(2,4));
% transformMatrix(3,1) = 1;
% transformMatrix(4,4) = 1;
 res = transformMatrix;



end

