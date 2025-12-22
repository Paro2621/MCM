%% Template Exam Modelling and Control of Manipulators
clear, clc;
close all;

addpath('include');
addpath('utils');
addpath('utils/plot');
addpath('utils/factory');

%% Initialize Geometric Model (GM) and Kinematic Model (KM)
iTj_0 = BuildTree();

% Define the tool frame rigidly attached to the end-effector
eRt = eye(3);
e_r_te = [0 0 0]';
eTt = tFactory(eRt, e_r_te);

% Define joints type, upper and lower bounds 
jointType = [0 0 0 0 0 1 0];
q_min = -3.14 * ones(7,1);  q_min(6) = 0;
q_max = 3.14 * ones(7,1);   q_max(6) = 1;

% Initialization of geometric and kinematic model
gm = geometricModel(iTj_0, jointType, q_min, q_max, eTt);
km = kinematicModel(gm);

% Display stuff
% disp('iTj_0'); disp(iTj_0);
% disp("eTt"); disp(eTt);
% bTt = gm.getToolTransformWrtBase();
% disp('bTt q = 0'); disp(bTt);

%% Define the goal frame and initialize cartesian control
% Goal definition 
bOg = [0.2; -0.7; 0.3];
theta = pi/2;
bRg = YPRToRot([0 theta 0]);
bTg = [bRg bOg;0 0 0 1]; 

% control proportional gain 
k_a = 0.8;
k_l = 0.8;

% Cartesian control initialization
cc = cartesianControl(gm, k_a, k_l);

% Display stuff
% disp('bTg'); disp(bTg);

%% Initial configuration
q = [pi/2, -pi/4, 0, -pi/4, 0, 0.15, pi/4]';
gm.updateDirectGeometry(q);

%% Kinematic Simulation - setup
% variables
t_start = 0.0;
t_end = 10.0;
dt = 1e-2;
t = t_start:dt:t_end;

% preallocation variables
bTi = zeros(4, 4, gm.jointNumber);
bri = zeros(3, gm.jointNumber+1);
    
%% Kinematic Simulation - main
i = 1;
qSteps = q;
for ti = t
    km.updateJacobian();
    eJb = km.J_EEwrtB();
    x_dot = 0.8*cc.getCartesianReference(bTg);

    feasible = 'f';

    for f = 1:gm.jointNumber
        
        % Possible controlling actions
        % translation only:     
        %   q_dot = k_l*eJb(4:6, :)\x_dot(4:6);
        %
        % rotation only:        
        %   q_dot = k_a*eJb\x_dot(1:3);
        %
        % complete control:     
        %   q_dot = eJb\x_dot;
        %   q_dot(1:3, :) = k_a*q_dot(1:3, :)
        %   q_dot(4:6, :) = k_l*q_dot(4:6, :)
        
        q_dot = k_l*eJb(4:6, :)\x_dot(4:6);

        % simulating the robot -> q = KinematicSimulation(q, q_dot, dt, q_min, q_max);
        q = q + q_dot.*dt;

        feasible = 't';
        
        for j = 1:length(q)
            if q(j) > q_max(j)
                q(j) = q_max(j);
                eJb(:, j) = zeros(6,1);
                feasible = 'f';
            elseif q(j) < q_min(j)
                q(j) = q_min(j);
                eJb(:, j) = zeros(6,1);
                feasible = 'f';
            end
        end
    end

    gm.updateDirectGeometry(q);

    qSteps(:, i) = q;

    i = i+1;
    
    % if(norm(x_dot(1:3)) < 0.01 && norm(x_dot(4:6)) < 0.01)
    if(norm(x_dot(1:3)) < 0.01 || norm(x_dot(4:6)) < 0.01)
        disp('Reached Requested Pose')
        break
    end
end


%% Motion plot
show_simulation = true;

% number of intermediate steps that will be plotted
samples = 10; 

% % linear spacing:       
% simIdx = ceil(linspace(1, length(qSteps)-1, samples))

% exponential spacing:
idx_max = 4;    % trial-and-error, nel caso base 4 o 5 vanno bene
idx = linspace(1, idx_max, samples);
simIdx = [1, floor((exp(idx)/exp(idx_max))*length(qSteps))];

qSteps = qSteps(:, simIdx);

pm = plotManipulators(show_simulation);
pm.initMotionPlot(t);

plotFrame(bTg, '<T>')
hold on
for i = 1:samples
    q = qSteps(:, i);

    % Updating transformation matrices for the new configuration 
    gm.updateDirectGeometry(q)

    % Get the transformation matrix from base to the tool
    bTe = gm.getTransformWrtBase(length(jointType)); 
    
    plotFrame(eye(4), ' ')
    hold on

    % Plot the motion of the robot 
    for j=1:gm.jointNumber
        bTi(:,:,j) = gm.getTransformWrtBase(j); 
        %plotFrame(bTi(:,:,j), ' ')
    end
    pm.plotIter(bTi)
end

