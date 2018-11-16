function [ DH ] = init()
DH = zeros(2,4);
DH(1,1) = 1;
DH(2,1) = .4;
DH(3,1) = .2;

DH(1,3) = 0.5;

DH(1,2) = pi/3;
DH(2,2) = pi/2;
DH(3,2) = pi/4;


DH(1,4) = 0;
DH(2,4) = pi/4;
DH(3,4) = -pi/6;






disp('Initial Denavit Hartenberg matrix');
disp(DH)
end

