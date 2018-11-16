close all; 
clear all;

DHcurr = init();

% DrawRobot(DHcurr);
jacobian = GetJacobian(DHcurr);
resm = InverseKinematics(DHcurr);
disp('Computed Jacobian:');
disp(jacobian);



