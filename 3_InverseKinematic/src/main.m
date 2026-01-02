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
eRt = YPRToRot(pi/10, 0, pi/6);
e_r_te = [0.3 0.1 0]';
eTt = tFactory(eRt, e_r_te)
iTj_0(:, :, end+1) = eTt;

% Define joints type, upper and lower bounds 
jointType = [0 0 0 0 0 1 0];
q_min = -3.14 * ones(7,1);  q_min(6) = 0;
q_max =  3.14 * ones(7,1);  q_max(6) = 1;

% Initialization of geometric and kinematic model
gm = geometricModel(iTj_0, jointType, q_min, q_max, eTt);
km = kinematicModel(gm);

bTe = gm.getToolTransformWrtBase()

% Display stuff
% disp('iTj_0'); disp(iTj_0);
% disp("eTt"); disp(eTt);
% bTt = gm.getToolTransformWrtBase();
% disp('bTt q = 0'); disp(bTt);

%% Geometric model check
q = [0 0 0 0 0 0 0]';

% Updating transformation matrices for the new configuration 
% gm.updateDirectGeometry(q)
% 
% figure(1)
% plotFrame(eye(4), '<b>')
% hold on
% axis equal
% 
% tree = []; 
% 
% % Plot the motion of the robot 
% for j=1:gm.jointNumber
%     bTi = gm.getTransformWrtBase(j); 
%     tree(:, :, j) = bTi;
%     plotFrame(bTi, "<" +int2str(j) +">")
% end
% 
% bTt = gm.getToolTransformWrtBase();
% tree(:, :, gm.jointNumber+1) = bTt;
% plotFrame(bTt, '<tool>')
% 
% 
% x_coords = squeeze(tree(1, 4, :));
% y_coords = squeeze(tree(2, 4, :));
% z_coords = squeeze(tree(3, 4, :));
% 
% line(x_coords(1:end-1), y_coords(1:end-1), z_coords(1:end-1), ...
%     'LineWidth', 1, 'Color', 'w', 'Marker','o')
% line(x_coords(end-1:end), y_coords(end-1:end), z_coords(end-1:end), ...
%     'LineWidth', 1, 'Color', 'w', 'Marker','+', 'LineStyle','--')

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
disp("initial error and EE velocity")
[x, x_dot] = cc.getCartesianReferenceEE(bTg)

% Display stuff
% disp('bTg'); disp(bTg);
% plotFrame(bTg, '<target>')

%% Initial configuration
q0 = [pi/2, -pi/4, 0, -pi/4, 0, 0.15, pi/4]';
gm.updateDirectGeometry(q0);

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
qSteps = q0;
q = q0;
qDotSteps = zeros(gm.jointNumber,1);
xDotSteps = zeros(6,1);

reachedRequestPose = false;

for ti = t
    km.updateJacobian();
    % J = km.J_EEwrtB();
    J = km.J_TwrtB;

    [x, x_dot] = cc.getCartesianReferenceTool(bTg);
    xDotSteps = [xDotSteps, x_dot];
    
    qLocked = zeros(gm.jointNumber,1);
    qDotLocked = zeros(gm.jointNumber,1);

    for f = 1:gm.jointNumber

        q_dot = pinv(J)*x_dot;

        for j = 1:gm.jointNumber
            if qDotLocked(j) ~= 0
                q_dot(j) = qDotLocked(j);
                q(j) = qLocked(j);
            end
        end

        q_sim = q + q_dot.*dt;

        % for j = 1:length(q_sim)
        %     if q_sim(j) > q_max(j)
        %         qLocked(j) = q_max(j);
        %         qDotLocked(j) = (q_max(j) - q(j)) / dt;
        %         eJb(:, j) = zeros(6,1);
        %     elseif q_sim(j) < q_min(j)
        %         qLocked(j) = q_min(j);
        %         qDotLocked(j) = (q_min(j) - q(j)) / dt;
        %         eJb(:, j) = zeros(6,1);
        %     end
        % end
    end

    q = q_sim;

    for j = 1:gm.jointNumber
        if qDotLocked(j) ~= 0
            q_dot(j) = qDotLocked(j);
            q(j) = qLocked(j);
        end
    end

    gm.updateDirectGeometry(q);
    
    i = i + 1;
    qSteps(:, i) = q;
    qDotSteps(:, i) = q_dot;

    if(norm(x(1:3)) < 0.01 && norm(x(4:6)) < 0.001)
        disp("goal reached")
        reachedRequestPose = true;
        break
    end
end

disp("final error and EE velocity")
[x, x_dot] = cc.getCartesianReferenceEE(bTg)

%% Joint velocity plot

t =[0, t(1:length(qSteps)-1)];

figure(2)
% --- Joint Positions -----------------------------------------------------
subplot(1,2,1)
plot(t, qSteps', '-')
legend('r1','r2','r3','r4','r5','l1','r6', 'Location', 'best')
xlabel('Time (s)')
ylabel('\theta')
title('Joint Positions')
grid on

% --- Joint Velocities ----------------------------------------------------
subplot(1,2,2)
plot(t, qDotSteps', '-')
legend('r1','r2','r3','r4','r5','l1','r6', 'Location', 'best')
xlabel('Time (s)')
ylabel('d\theta/dt')
title('Joint Velocities')
grid on

figure(3)
% --- EE velocity wrt BASE ------------------------------------------------
v_EEwrtB = [];
for ti = 1:length(qSteps)
    gm.updateDirectGeometry(qSteps(:, ti));
    J_wrtBase = km.J_EEwrtB;
    velocities_wrtBase = J_wrtBase*qDotSteps(:, ti);
    v_EEwrtB(:, ti) = velocities_wrtBase;
end

subplot(1,2,1)
plot(t, v_EEwrtB', '-')
legend('\omega_x','\omega_y','\omega_z','v_x','v_y','v_z', 'Location', 'best')
xlabel('Time (s)')
ylabel('d\theta/dt')
title('Velocity of EE wrt Base')
grid on

% --- Tool velocity wrt BASE ----------------------------------------------
v_ToolwrtB = [];
for ti = 1:length(qSteps)
    gm.updateDirectGeometry(qSteps(:, ti));
    J_ToolwrtBase = km.getJacobianOfToolWrtBase();
    velocities_ToolwrtBase = J_ToolwrtBase*qDotSteps(:, ti);
    v_ToolwrtB(:, ti) = velocities_ToolwrtBase;
end

subplot(1,2,2)
plot(t, v_ToolwrtB, '-')
legend('\omega_x','\omega_y','\omega_z','v_x','v_y','v_z', 'Location', 'best')
xlabel('Time (s)')
ylabel('d\theta/dt')
title('Velocity of Tool wrt Base')
grid on

figure(4)
% --- Direction of the EE velocity ----------------------------------------
dir_xDotStep = [];
for i = xDotSteps
    dir_xDotStep = [dir_xDotStep, i/norm(i)];
end
subplot(1,2,1)
plot(t, dir_xDotStep', '-')
legend('r1','r2','r3','l1','l2','l3', 'Location', 'best')
xlabel('Time (s)')
ylabel('\theta')
title('Direction of the EE velocity')
grid on

% --- Direction of the Tool velocity --------------------------------------
dir_vTool = [];
for i = v_ToolwrtB
    dir_vTool = [dir_vTool, i/norm(i)];
end
subplot(1,2,2)
plot(t, dir_vTool', '-')
legend('r1','r2','r3','l1','l2','l3', 'Location', 'best')
xlabel('Time (s)')
ylabel('\theta')
title('Direction of the Tool velocity')
grid on

%% Motion plot
show_simulation = true;

% number of intermediate steps that will be plotted
samples = 30; 

% linear spacing:       
simIdx = ceil(linspace(1, length(qSteps)-1, samples));

% exponential spacing:
% idx_max = 3.5;    % trial-and-error, nel caso base con 10 samples 4 o 5 vanno bene
% idx = linspace(1, idx_max, samples);
% simIdx = [1, floor((exp(idx)/exp(idx_max))*length(qSteps)), length(qSteps)];

qSteps = qSteps(:, simIdx);

figure(1)
pm = plotManipulators(show_simulation);
pm.initMotionPlot(t);

plotFrame(bTg, '<T>')
q = qSteps(:, 1);
gm.updateDirectGeometry(q)

for i = 1:samples
    q = qSteps(:, i);

    % Updating transformation matrices for the new configuration 
    gm.updateDirectGeometry(q)
    
    plotFrame(eye(4), ' ')
    hold on

    % Plot the motion of the robot 
    for j=1:gm.jointNumber+1
        bTi(:,:,j) = gm.getTransformWrtBase(j); 
    end
    
    % Plot the motion of the robot 
    for j=1:gm.jointNumber
        bTi = gm.getTransformWrtBase(j); 
        tree(:, :, j) = bTi;
        % plotFrame(bTi, "<" +int2str(j) +">")
    end
    
    bTt = gm.getToolTransformWrtBase();
    tree(:, :, gm.jointNumber+1) = bTt;
    plotFrame(bTt, ' ')
    
    
    x_coords = squeeze(tree(1, 4, :));
    y_coords = squeeze(tree(2, 4, :));
    z_coords = squeeze(tree(3, 4, :));
    
    line(x_coords(1:end-1), y_coords(1:end-1), z_coords(1:end-1), ...
        'LineWidth', 1, 'Color', 'black', 'Marker','o')
    line(x_coords(end-1:end), y_coords(end-1:end), z_coords(end-1:end), ...
        'LineWidth', 1, 'Color', 'black', 'Marker','+', 'LineStyle','--')

end
