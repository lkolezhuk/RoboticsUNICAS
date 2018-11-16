function [ res ] = NextFrameTransform( DH, current_frame )
    %UNTITLED6 Summary of this function goes here
    %   Detailed explanation goes here
    sith = sin(DH(current_frame, 4));
    cith = cos(DH(current_frame, 4));
    sial = sin(DH(current_frame, 2));
    cial = cos(DH(current_frame, 2));
    
    frameTransform = zeros(4,4);
    frameTransform(1,1) = cith;
    frameTransform(1,2) = -sith*cial;
    frameTransform(1,3) = sith*sial;
    frameTransform(1,4) = DH(current_frame, 1) * cith;

    frameTransform(2,1) = sith;
    frameTransform(2,2) = cith*cial;
    frameTransform(2,3) = -cith*sial;
    frameTransform(2,4) = DH(current_frame, 1) * sith;
    
    
    frameTransform(3,2) = sial;
    frameTransform(3,3) = cial;
    frameTransform(3,4) = DH(current_frame, 3);
    frameTransform(4,4) = 1;

    res = frameTransform;
end

