%% Template Exam Modelling and Control of Manipulators
clear, clc;
close all;

addpath('include');
addpath('utils');
addpath('utils/plot');
addpath('utils/factory');

% Compute the geometric model for the given manipulator
iTj_0 = BuildTree();

% disp('iTj_0')
% disp(iTj_0);
jointType = [0 0 0 0 0 1 0];
q = [pi/2, -pi/4, 0, -pi/4, 0, 0.15, pi/4]';

% Define the tool frame rigidly attached to the end-effector
% Tool frame definition
eRt = eye(3);
e_r_te = [0 0 0]';
eTt = tFactory(eRt, e_r_te);

% Initialize Geometric Model (GM) and Kinematic Model (KM)
% Initialize geometric model with q0
gm = geometricModel(iTj_0, jointType, eTt);

% Update direct geoemtry given q0
gm.updateDirectGeometry(q);

% Initialize the kinematic model given the goemetric model
km = kinematicModel(gm);

bTt = gm.getToolTransformWrtBase();

% disp("eTt");
% disp(eTt);
% disp('bTt q = 0');
% disp(bTt);

% Define the goal frame and initialize cartesian control
% Goal definition 
bOg = [0.2; -0.7; 0.3];
theta = pi/2;
bRg = YPRToRot([0 theta 0]);
bTg = [bRg bOg;0 0 0 1]; 
% disp('bTg')
% disp(bTg)

% control proportional gain 
k_a = 0.8;
k_l = 0.8;

% Cartesian control initialization
cc = cartesianControl(gm, k_a,k_l);

% --- test aerea ---
pm = plotManipulators(1);
pm.initMotionPlot(0);

for j=1:gm.jointNumber
    bTi(:,:,j) = gm.getTransformWrtBase(j); 
    plotFrame(bTi(:,:,j), ' ')
    axis equal
    pm.plotIter(bTi)
end
plotFrame(bTg, '<T>')

% --- end test aera ---

%% Initialize control loop 

% Simulation variables
samples = 100;
t_start = 0.0;
t_end = 10.0;
dt = (t_end-t_start)/samples;
t = t_start:dt:t_end; 

% preallocation variables
bTi = zeros(4, 4, gm.jointNumber);
bri = zeros(3, gm.jointNumber+1);

% joints upper and lower bounds 
qmin = -3.14 * ones(7,1);
qmin(6) = 0;
qmax = +3.14 * ones(7,1);
qmax(6) = 1;

show_simulation = true;
pm = plotManipulators(show_simulation);
pm.initMotionPlot(t, bTg(1:3,4));

%%%%%%% Kinematic Simulation %%%%%%%
for i = t
    % Updating transformation matrices for the new configuration 

    % Get the cartesian error given an input goal frame

    % Update the jacobian matrix of the given model

    %% INVERSE KINEMATICS
    % Compute desired joint velocities 
    q_dot = ...

    % simulating the robot
    q = KinematicSimulation(....);
    
    pm.plotIter(gm, km, i, q_dot);

    if(norm(x_dot(1:3)) < 0.01 && norm(x_dot(4:6)) < 0.01)
        disp('Reached Requested Pose')
        break
    end

end

pm.plotFinalConfig(gm);