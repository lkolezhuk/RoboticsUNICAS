% MAIN01.M
%
% "Introduction to robotics" exercise: direct kinematics computation

% it clears the workspace 
clear
close all

% it initializes variables
init

% direct kinematics computation
T = DirectKinematics(DH);

% plot
DrawRobot(DH)
      
