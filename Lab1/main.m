DH = init();

dkTransform = ComputeDKTransform(DH);

DrawRobot(DH);
disp('Homogenius transformation');
disp(dkTransform);
