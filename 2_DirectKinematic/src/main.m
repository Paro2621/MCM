%% Template Exam Modelling and Control of Manipulators
clc;
close all;
clear;

addpath('include');
addpath('utils');
addpath('utils/plot');
addpath('utils/factory');

% Compute the geometric model for the given manipulator
iTj_0 = BuildTree();
disp('iTj_0')
disp(iTj_0);
jointType = [0 0 0 0 0 1 0]; % specify two possible link type: Rotational, Prismatic.
geometricModel = geometricModel(iTj_0, jointType);

%% Q1.3
function out = invertHTM(A) 
    R_t = A(1:3, 1:3)';
    r = A(1:3, 4);
    
    out = eye(4);
    out(1:3, 1:3) = R_t;
    out(1:3, 4) = -R_t*r;
end

% b_T_e
T_be = geometricModel.getTransformWrtBase(7)

% 6_T_2
T_b2 = geometricModel.getTransformWrtBase(2);
T_b6 = geometricModel.getTransformWrtBase(6);
T_62 = invertHTM(T_b2)*T_b6

%% Q1.4 Simulation
% Given the following configurations compute the Direct Geometry for the manipulator

% Compute iTj : transformation between the base of the joint <i>
% and its end-effector taking into account the actual rotation/traslation of the joint
qi = [pi/4, -pi/4, 0, -pi/4, 0, 0.15, pi/4];
geometricModel.updateDirectGeometry(qi)
disp('iTj')
disp(geometricModel.iTj);

%%% Compute the transformation of the ee w.r.t. the robot base
bTe = geometricModel.getTransformWrtBase(length(jointType));  
disp('bTe')
disp(bTe)

% Show simulation
show_simulation = true;

% Set initial and final joint positions
qf = [5*pi/12, -pi/3, 0, -pi/4, 0, 0.18, pi/5];

% Simulation loop
% Simulation variables

% simulation time definition 
samples = 10;
t_start = 0.0;
t_end = 10.0;
dt = (t_end-t_start)/samples;
t = t_start:dt:t_end; 

pm = plotManipulators(show_simulation);
pm.initMotionPlot(t);

qSteps =[linspace(qi(1),qf(1),samples)', ...
    linspace(qi(2),qf(2),samples)', ...
    linspace(qi(3),qf(3),samples)', ...
    linspace(qi(4),qf(4),samples)', ...
    linspace(qi(5),qf(5),samples)', ...
    linspace(qi(6),qf(6),samples)', ...
    linspace(qi(7),qf(7),samples)'];

for i = 1:samples
    brij= zeros(3,geometricModel.jointNumber);
    q = qSteps(i,1:geometricModel.jointNumber)';
    % Updating transformation matrices for the new configuration 
    geometricModel.updateDirectGeometry(q)
    % Get the transformation matrix from base to the tool
    bTe = geometricModel.getTransformWrtBase(length(jointType)); 

    % Plot the motion of the robot 
    if (rem(i,0.1) == 0) % only every 0.1 sec
        for j=1:geometricModel.jointNumber
            bTi(:,:,j) = geometricModel.getTransformWrtBase(j); 
        end
        pm.plotIter(bTi)
    end
end

pm.plotFinalConfig(bTi)

%% Q1.5
km = kinematicModel(geometricModel);
km.getJacobianOfLinkWrtBase(6)

%% Q1.6
km.updateJacobian();
km.J

%% Q1.7
clc;
close all;
clear;

addpath('include');
addpath('utils');
addpath('utils/plot');
addpath('utils/factory');

qi = [0.7 -0.1 1 -1 0 0.03 1.3];

qi_dot = [0.9 0.1 -0.2 0.3 -0.8 0.5 0]';

% Compute the geometric model for the given manipulator
iTj_0 = BuildTree();
jointType = [0 0 0 0 0 1 0]; % specify two possible link type: Rotational, Prismatic.

gm = geometricModel(iTj_0, jointType);
gm.updateDirectGeometry(qi);
km = kinematicModel(gm);
km.updateJacobian();
velocities = km.J*qi_dot 
    % [vx vy vx omega_x omega_y omega_z]