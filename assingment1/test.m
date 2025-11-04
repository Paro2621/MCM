addpath('include');
addpath('utils');
addpath('utils/plot');

clear, close, clc;

%% TEST 1
roll = deg2rad(10);
pitch = deg2rad(30);
yaw = deg2rad(45);

YPR = [yaw; pitch; roll];

rot = YPRToRot(YPR);

YPR_1 = RotToYPR(rot);

rot_1 = YPRToRot(YPR_1);

% figure, 
% dispFrame(rot)
% dispFrame(rot_1)
% axis equal

delta_test1 = YPR_1 - YPR

%% TEST 2
roll = deg2rad(10);
pitch = deg2rad(30);
yaw = deg2rad(45);

YPR = [yaw; pitch; roll];

rot = YPRToRot(YPR);

[h, theta] = RotToAngleAxis(rot);

rot_1 = AngleAxisToRot(h, theta);

YPR_1 = RotToYPR(rot_1);

% figure, 
% dispFrame(rot)
% dispFrame(rot_1)
% axis equal

delta_test2 = YPR_1 - YPR