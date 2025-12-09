%% Template Exam Modelling and Control of Manipulators
clc;
close all;
clear;

addpath('include');
addpath('utils');
addpath('utils/plot');
addpath('utils/factory');

%% ric's version - DH standard
DHp = @(q)...
    [0.105          q(1)    0   0; ...
     0.11           0       0   -pi/2; ...
     0              q(2)    0   0;...           # extraFrame
     0              q(3)    0   pi/2; ...
     0.425          0    0   -pi/2; ...
     0              q(4)    0   0;...           # extraFrame
     0              q(5)    0   pi/2; ...
     0.095 + q(6)   0       0   0;...
     0.355          q(7)    0   0];

T_EE = tFactory(eye(3), [0 0 0.155]');

% joint variable: 
% [!] ATTENTION q(6) is traslational, it is later overwritten
q0 = deg2rad([0 0 0 0 0 0 0]); % [deg] -> [rad]
q0(6) = 0; % [m]

% plot
figure
plotDH(DHp(q0), T_EE, 'std'); title("RoboticArm\_p0");
view([30, 30])

% pose 1
figure
q1 = [pi/4, -pi/4, 0, -pi/4, 0, 0.15, pi/4];
T_EE_0 = plotDH(DHp(q1), T_EE, 'std'), title("RoboticArm\_p1");
view([30, 30])