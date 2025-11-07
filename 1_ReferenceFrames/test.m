addpath('include');
addpath('utils');
addpath('utils/plot');

clear, close, clc;

%% 1.1 Angle-axis to rot

function err = testAngleAxisToRot(h, theta)
    R = AngleAxisToRot(h, theta);
    [h_test, theta_test] = RotToAngleAxis(R);
    
    rho = theta*h;
    rho_test = h_test*theta_test;
    err = norm(rho - rho_test);
end

% --- Q1.2 ---
h = [1; 0; 0];
theta = deg2rad(90);

err_rho_q12 = testAngleAxisToRot(h, theta)

% --- Q1.3 ---
h = [0; 0; 1];
theta = pi/3;

err_rho_q13 = testAngleAxisToRot(h, theta)

% --- Q1.4 ---
rho = [-pi/3; -pi/6; pi/3];

theta = norm(rho);
h = rho/theta;

err_rho_q14 = testAngleAxisToRot(h, theta)

%% 1.2 Rot to angle-axis

function s = testRotToAngleAxis(R)
    if isRot(R)
        [h, theta] = RotToAngleAxis(R);
        R_test = AngleAxisToRot(h, theta);
        
        alpha = subspace(R, R_test);
            % computes the principal angle (in radians) between the column 
            % spaces of the two argiment matrices
        
        s = cos(alpha); % 1 = identical subspace, 0 = orthogonal
    else
        disp("R is not a rotation matrix")
        s = NaN;
    end
end


% --- Q2.2 ---
R = [1 0 0 ; 0 0 -1; 0 1 0];
similarity_q22 = testRotToAngleAxis(R)


% --- Q2.3 ---
R = [0.5 -sqrt(3)/2 0; sqrt(3)/2 0.5 0; 0 0 1];
similarity_q23 = testRotToAngleAxis(R)


% --- Q2.4 ---
R = [1 0 0; 0 1 0; 0 0 1];
similarity_q24 = testRotToAngleAxis(R)


% --- Q2.5 ---
R = [-1 0 0; 0 -1 0; 0 0 1];
similarity_q25 = testRotToAngleAxis(R)


% --- Q2.6 ---
R = [-1 0 0; 0 1 0; 0 0 1];
similarity_q26 = testRotToAngleAxis(R)

%% 1.3 Euler to rot
% YPR = [psi; theta; phi];

function err = testYPRToRot(v)
    R = YPRToRot(v);
    YPR_test = RotToYPR(R);

    err = norm(v - YPR_test);
end

% --- Q3.2 ---
YPR = [0; 0; pi/2];
err_q32 = testYPRToRot(YPR)

% --- Q3.3 ---
YPR = [deg2rad(60); 0; 0];
err_q33 = testYPRToRot(YPR)

% --- Q3.4 ---
YPR = [pi/3; pi/2; pi/4];
err_q34 = testYPRToRot(YPR)

% since gimbal lock was detected -> plot frame
R = YPRToRot(YPR);

YPR_test = RotToYPR(R);
R_test = YPRToRot(YPR_test);

figure, 
subplot(1,2,1)
plotFrame(R), hold on
plotFrame(R_test), grid on, axis equal

% --- Q3.5 ---
YPR = [0; pi/2; -pi/12];
err_q35 = testYPRToRot(YPR)

% since gimbal lock was detected -> plot frame
R = YPRToRot(YPR);

YPR_test = RotToYPR(R);
R_test = YPRToRot(YPR_test);

subplot(1,2,2)
plotFrame(R), hold on
plotFrame(R_test), grid on, axis equal

% INTERESTING ASPECT: the two results are the same
% In case of gimbal lock, the first (yaw) and last (roll) rotations both 
% act about the same axis, so only their sum matters.
YPR_q34 = [pi/3; pi/2; pi/4];
YPR_q35 = [0; pi/2; -pi/12];

% thus
YPR_q34_prime = [YPR_q34(1) - YPR_q34(3); pi/2; 0];
YPR_q35_prime = [YPR_q35(1) - YPR_q35(3); pi/2; 0];

diff = mod(abs(YPR_q34_prime(1) - YPR_q35_prime(1)), 2*pi);
    % w/ small negative value (i.e -1e-17) mod returns 6.2831 which is
    % exacly 2*pi

if kEq(diff, 0)
    disp("The two rotation matrices are the same")
end

%% Frame rotation visualization
clear, clc;
close all; 

t0 = eye(3);

YPR_q34 = [pi/3; pi/2; pi/4];

figure
subplot(2, 3, 1), plotFrame(Rx(YPR_q34(3))*t0), axis equal
subplot(2, 3, 1), plotFrame(t0, ["r--" "g--" "b--"])

subplot(2, 3, 2), plotFrame(Ry(YPR_q34(2))*Rx(YPR_q34(3))*t0), axis equal
subplot(2, 3, 2), plotFrame(Rx(YPR_q34(3))*t0, ["r--" "g--" "b--"])

subplot(2, 3, 3), plotFrame(Rz(YPR_q34(1))*Ry(YPR_q34(2))*Rx(YPR_q34(3))*t0), axis equal
subplot(2, 3, 3), plotFrame(Ry(YPR_q34(2))*Rx(YPR_q34(3))*t0, ["r--" "g--" "b--"])

YPR_q35 = [0; pi/2; -pi/12];
subplot(2, 3, 4), plotFrame(Rx(YPR_q35(3))*t0), axis equal
subplot(2, 3, 4), plotFrame(t0, ["r--" "g--" "b--"])

subplot(2, 3, 5), plotFrame(Ry(YPR_q35(2))*Rx(YPR_q35(3))*t0), axis equal
subplot(2, 3, 5), plotFrame(Rx(YPR_q35(3))*t0, ["r--" "g--" "b--"])

subplot(2, 3, 6), plotFrame(Rz(YPR_q35(1))*Ry(YPR_q35(2))*Rx(YPR_q35(3))*t0), axis equal
subplot(2, 3, 6), plotFrame(Ry(YPR_q35(2))*Rx(YPR_q35(3))*t0, ["r--" "g--" "b--"])
%% 1.4 Rot to Euler
function s = testRotToYPR(R)
    if isRot(R)
        YPR = RotToYPR(R);
        R_test = YPRToRot(YPR);
        
        alpha = subspace(R, R_test);
            % computes the principal angle (in radians) between the column 
            % spaces of the two argiment matrices
        
        s = cos(alpha); % 1 = identical subspace, 0 = orthogonal
    else
        disp("R is not a rotation matrix")
        s = NaN;
    end
end


% --- Q4.2 ---
R = [1 0 0; 0 0 -1; 0 1 0];
err_q42 = testRotToYPR(R)

% --- Q4.3 ---
R = [1/2 -sqrt(3)/2 0; sqrt(3)/2 1/2 0; 0 0 1];
err_q43 = testRotToYPR(R)

% --- Q4.4 ---
R = [0          -sqrt(2)/2  sqrt(2)/2; ...
    0.5         sqrt(6)/4   sqrt(6)/4; ...
    -sqrt(3)/2  sqrt(2)/4   sqrt(2)/4];
err_q44 = testRotToYPR(R)

%% 1.5 Rot to angle-axis with eigenvectors

% --- Q5.1 ---
R = [1 0 0; 0 0 -1; 0 1 0];

[h, theta] = RotToAngleAxis(R)

[V,D] = eig(R);
V = real(V);
D = real(D);

for i = 1:3
    if kEq(D(i, i), 1)
        h_star = V(:, i)
    end
end

theta_star = acos((trace(R)-1)/2)

% --- Q5.2 ---
R = 1/9 * [4 -4 -7; 8 1 4; -1 -8 4];

[h, theta] = RotToAngleAxis(R)

[V,D] = eig(R);
V = real(V);
D = real(D);

for i = 1:3
    if kEq(D(i, i), 1)
        h_star = V(:, i)
    end
end

theta_star = acos((trace(R)-1)/2)