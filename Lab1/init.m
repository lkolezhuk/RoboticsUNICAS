function [ DH ] = init()
DH = zeros(2,4);
DH(1,1) = .5;
DH(2,1) = .4;
DH(3,1) = .2;

DH(1,4) = 3.14/3;
DH(2,4) = 3.14/3;
DH(3,4) = -3.14/6;

DH(1,1) = .5;
DH(2,1) = .4;
DH(3,1) = .2;
DH(1,2) = 3.14/2;

DH(1,4) = 3.14/3;
DH(2,4) = 3.14/3;
DH(3,4) = -3.14/6;


disp('Denavit Hartenberg matrix');
disp(DH)
end

