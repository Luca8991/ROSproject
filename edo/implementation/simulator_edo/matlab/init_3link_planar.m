clear all
close all

% Load robot object from URDF
robot_3link_planar = importrobot('/home/eulero/Documents/ros/edo_impl/simulator_edo/robots/edo_sim.urdf');

% Parameters (same as in URDF)
L0 = 0.145292221848281; % base_link [m]
%L1 = 0.18967; % link_1 [m]
L1 = 0.334962221848281; % link_1 [m]
L2 = 0.210497555007178; % link_2 [m]
L3 = 0.427990120537185; % link_3 (=link_3 + link_4 + link_5 + link_6)[m]