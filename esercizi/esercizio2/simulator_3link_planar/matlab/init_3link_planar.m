clear all
close all

% Load robot object from URDF
robot_3link_planar = importrobot('../urdf/3link_planar_model.urdf');

% Parameters (same as in URDF)
L1 = 1; % Arm length [m]
L2 = 1; % Forearm length [m]
L3 = 1; % Wrist length [m]