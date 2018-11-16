DH = init();

% DrawRobot(DH);
jacobian = GetJacobian(DH);

disp('Computed Jacobian:');
disp(jacobian);





