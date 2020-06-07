clear all
close all

% Load robot object from URDF
robot_6link_edo = importrobot('../robots/edo_sim.urdf');

robot_6link_edo.Gravity = [0.0, 0.0, -9.81];
% Parameters (same as in URDF)
L0 = 0.145292221848281; % base_link [m]
%L1 = 0.18967; % link_1 [m] (old)
L1 = 0.334962221848281; % link_1 [m]
L2 = 0.210497555007178; % link_2 [m]
L3 = 0.427990120537185; % link_3 (=link_3 + link_4 + link_5 + link_6)[m]
