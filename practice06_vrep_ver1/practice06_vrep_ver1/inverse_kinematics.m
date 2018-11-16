% MAIN06.M
%
% esercizio di sistemi robotici: algoritmi di inversione cinematica
% con utilizzo dei quaternioni per l'errore di orientamento

function [dq, error_pos, error_quat] = inverse_kinematics(q,xd,quat_d, DH)

algorithm = 'inverse';
if strcmp(algorithm,'transpose')
    K = diag([30*[1 1 1], 30*[1 1 1]]);
else
    %K = diag([2.5*[1 1 1], 4*[1 1 1]]);
    K = diag([3*[1 1 1], 3*[1 1 1]]);
 
end

   n = size(DH,1);
   % direct kinematics
  
   T = DirectKinematics(DH);
   x = T(1:3,4,n);
   quat = Rot2Quat(T(1:3,1:3,n));
   % Jacobian
   J = Jacobian(DH);
   % Inverse kinematics algorithm
   error_pos = xd - x;
   error_quat = QuatError(quat_d,quat);
   error = [error_pos;error_quat];
        
    if strcmp(algorithm,'transpose')
        dq = J'*K*error;
    else
        dq = pinv(J)*K*error;
    end
   
end

