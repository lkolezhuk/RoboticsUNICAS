function [ res ] = ComputeDKTransform( DH )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

[num_links,~] = size(DH);
% disp(num_links);
aframes = cell(num_links);

aframes{1} = NextFrameTransform(DH, 1);
transformMatrix = aframes{1};

for i = 2:num_links
   aframes{i} = NextFrameTransform(DH, i);
   transformMatrix = transformMatrix * aframes{i};

end

res = transformMatrix;



end

