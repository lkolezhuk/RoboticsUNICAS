%
%
% Template to visualize in V-REP the inverse kinematics algorithm developed
%          for a 7-DOF robot of the family Kuka LWR or IIWA
%
% Read Instructions.odt first !
% 
% Do not modify any part of this file except the strings within
%    the symbols << >>
%
% G. Antonelli, Sistemi Robotici, fall 2014


function [t, q, q_act] = main
close all
    porta = 19997;          % default V-REP port
    tf = 5;                 % final time
    Ts = 0.01;              % sampling time
    t  = 0:Ts:tf;           % time vector
    N  = length(t);         % number of points of the simulation 
    n = 7;                  % joint number
    q  = zeros(n,N);        % q(:,i) collects the joint position for t(i)
    dq  = zeros(n,N);       % q(:,i) collects the joint position for t(i)
    error_pos = zeros(3,N);
    error_quat = zeros(3,N);
    xd = zeros(3,N);
    quat_d = zeros(4,N);
    
    kuka_init

     
    q(:,1) = DH(:,4);
    qvrep = zeros(n,N);
    qvrep(:,1) = q(:,1);
    qvrep(2,1) = -q(2,1);
    qvrep(6,1) = -q(6,1);
 
    clc
    fprintf('----------------------');
    fprintf('\n simulation started ');
    fprintf('\n trying to connect...\n');
    [clientID, vrep ] = StartVrep(porta);
    
    [returnCode]= vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
    pause(2)
    
    handle_joint = my_get_handle_Joint(vrep,clientID);      % handle to the joints
    
    my_set_joint_target_position(vrep, clientID, handle_joint, qvrep(:,1)); % first move to q0
    q_act(:,1) = my_get_joint_target_position(clientID,vrep,handle_joint,n);% get the actual joints angles from v-rep     
    pause(2)
    
  
  
    % main simulation loop
    for i=1:N
        
        DH(:,4) = q(:,i);
     
        xd(:,i) = [0.15 -0.5 0.7]';
        quat_d(:,i) = [0 0 0 1]';
     
       
        [dq(:,i), error_pos(:,i), error_quat(:,i)] = inverse_kinematics(q(:,i),xd(:,i),quat_d(:,i), DH);
        
        
        if i<N
            q(:,i+1) = q(:,i) + Ts*dq(:,i);
        end
      
        
       
        qvrep(:,i) = q(:,i);
        qvrep(2,i) = -q(2,i);
        qvrep(6,i) = -q(6,i);
        
    
        my_set_joint_target_position(vrep, clientID, handle_joint, qvrep(:,i));        
        q_act(:,i) = my_get_joint_target_position(clientID,vrep,handle_joint,n);% get the actual joints angles from v-rep  
   
    end

    DeleteVrep(clientID, vrep);
    
    figure
        subplot(211)
            plot(t,error_pos)
            title('Position error')
            grid on
        subplot(212)
            plot(t,error_quat)
            title('Orientation error')
            grid on
    
    figure
        subplot(211)
            plot(t,q)
            grid on
            title('Joint positions')
            legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7')
        subplot(212)
            plot(t,dq)
            grid on
            title('Joint velocities')
            legend('dq1', 'dq2', 'dq3', 'dq4', 'dq5', 'dq6', 'dq7')
    
    figure
        hold on
        DH(:,4) = q(:,1);
        DrawRobot(DH);
        DH(:,4) = q(:,N);
        DrawRobot(DH);
  
end

% constructor
function [clientID, vrep ] = StartVrep(porta)

    vrep = remApi('remoteApi');   % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1);        % just in case, close all opened connections
    clientID = vrep.simxStart('127.0.0.1',porta,true,true,5000,5);% start the simulation
    
    if (clientID>-1)
        disp('remote API server connected successfully');
    else
        disp('failed connecting to remote API server');   
        DeleteVrep(clientID, vrep); %call the destructor!
    end
    % to change the simulation step time use this command below, a custom dt in v-rep must be selected, 
    % and run matlab before v-rep otherwise it will not be changed 
    % vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, 0.002, vrep.simx_opmode_oneshot_wait);
    
end  

% destructor
function DeleteVrep(clientID, vrep)

    vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait); % pause simulation
%   vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait); % stop simulation
    vrep.simxFinish(clientID);  % close the line if still open
    vrep.delete();              % call the destructor!
    disp('simulation ended');
    
end

function my_set_joint_target_position(vrep, clientID, handle_joint, q)
           
    [m,n] = size(q);
    for i=1:n
        for j=1:m
            err = vrep.simxSetJointTargetPosition(clientID,handle_joint(j),q(j,i),vrep.simx_opmode_oneshot);
            if (err ~= vrep.simx_error_noerror)
                fprintf('failed to send joint angle q %d \n',j);
            end
        end
    end
    
end

function handle_joint = my_get_handle_Joint(vrep,clientID)

    [~,handle_joint(1)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint1',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(2)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint2',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(3)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint3',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(4)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint4',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(5)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint5',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(6)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint6',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(7)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint7',vrep.simx_opmode_oneshot_wait);

end

function my_set_joint_signal_position(vrep, clientID, q)
           
    [~,n] = size(q);
    
    for i=1:n
        joints_positions = vrep.simxPackFloats(q(:,i)');
        [err]=vrep.simxSetStringSignal(clientID,'jointsAngles',joints_positions,vrep.simx_opmode_oneshot_wait);

        if (err~=vrep.simx_return_ok)   
           fprintf('failed to send the string signal of iteration %d \n',i); 
        end
    end
    pause(8);% wait till the script receives all data, increase it if dt is too small or tf is too high
    
end


function angle = my_get_joint_target_position(clientID,vrep,handle_joint,n)
    
    for j=1:n
         vrep.simxGetJointPosition(clientID,handle_joint(j),vrep.simx_opmode_streaming);
    end

    pause(0.05);

    for j=1:n          
         [err(j),angle(j)]=vrep.simxGetJointPosition(clientID,handle_joint(j),vrep.simx_opmode_buffer);
    end

    if (err(j)~=vrep.simx_return_ok)   
           fprintf(' failed to get position of joint %d \n',j); 
    end

end

