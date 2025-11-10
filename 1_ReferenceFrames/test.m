addpath('include');
addpath('utils');
addpath('utils/plot');

clear, close, clc;

%% 1 Angle-axis to rot

disp("---1---")

function s = testAngleAxisToRot(h, theta)
    % my function
    R_test = AngleAxisToRot(h, theta);
    
    % builtin function
    ax = [h(:); theta]'; 
    R = axang2rotm(ax);

    R_diff = R' * R_test;
    s = (trace(R_diff) - 1) / 2;
end
% --- Q1.2 ---
h = [1; 0; 0];
theta = deg2rad(90);

similarity_q12 = testAngleAxisToRot(h, theta)

% --- Q1.3 ---
h = [0; 0; 1];
theta = pi/3;

similarity_q13 = testAngleAxisToRot(h, theta)

% --- Q1.4 ---
rho = [-pi/3; -pi/6; pi/3];

theta = norm(rho);
h = rho/theta;

similarity_q14 = testAngleAxisToRot(h, theta)

%% 2 Rot to angle-axis

disp("---2---")

function err = testRotToAngleAxis(R)
    if isRot(R)
        % my function
        [h_test, theta_test] = RotToAngleAxis(R);
        rho_test = h_test*theta_test;

        % builtin function
        v = axang(so3(R));
        h = v(1:3)'/norm(v(1:3));
        theta = v(4);
        rho = h*theta;
        
        err = norm(rho - rho_test);
    else
        disp("R is not a rotation matrix")
        err = NaN;
    end
end


% --- Q2.2 ---
R = [1 0 0 ; 0 0 -1; 0 1 0];
err_q22 = testRotToAngleAxis(R)


% --- Q2.3 ---
R = [0.5 -sqrt(3)/2 0; sqrt(3)/2 0.5 0; 0 0 1];
err_q23 = testRotToAngleAxis(R)


% --- Q2.4 ---
R = [1 0 0; 0 1 0; 0 0 1];
err_q24 = testRotToAngleAxis(R)


% --- Q2.5 ---
R = [-1 0 0; 0 -1 0; 0 0 1];
err_q25 = testRotToAngleAxis(R)


% --- Q2.6 ---
R = [-1 0 0; 0 1 0; 0 0 1];
err_q26 = testRotToAngleAxis(R)

%% 3 Euler to rot
% YPR = [phi; theta; psi];

disp("---3---")

function s = testYPRToRot(v)
    % my function
    R_test = YPRToRot(v);

    % builtin function
    R = rotm(so3(v',"eul"));

    R_diff = R' * R_test;
    s = (trace(R_diff) - 1) / 2;
end

% --- Q3.2 ---
YPR = [0; 0; pi/2];
similarity_q32 = testYPRToRot(YPR)

% --- Q3.3 ---
YPR = [deg2rad(60); 0; 0];
similarity_q33 = testYPRToRot(YPR)

% --- Q3.4 ---
YPR = [pi/3; pi/2; pi/4];
similarity_q34 = testYPRToRot(YPR)
% gimbal lock detected

% --- Q3.5 ---
YPR = [0; pi/2; -pi/12];
similarity_q35 = testYPRToRot(YPR)
% gimbal lock detected

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

%% 4 Rot to Euler

disp("---4---")

function delta = testRotToYPR(R)
    if isRot(R)
        % my function
        YPR_test = RotToYPR(R);

        % builtin function
        YPR = eul(so3(R))';

        delta = asin(norm(cross(YPR, YPR_test)));

    else
        disp("R is not a rotation matrix")
        delta = NaN;
    end
end


% --- Q4.2 ---

R = [1 0 0; 0 0 -1; 0 1 0];
delta_q42 = testRotToYPR(R)

% --- Q4.3 ---
R = [1/2 -sqrt(3)/2 0; sqrt(3)/2 1/2 0; 0 0 1];
delta_q43 = testRotToYPR(R)

% --- Q4.4 ---
R = [0          -sqrt(2)/2  sqrt(2)/2; ...
    0.5         sqrt(6)/4   sqrt(6)/4; ...
    -sqrt(3)/2  sqrt(2)/4   sqrt(2)/4];
delta_q44 = testRotToYPR(R)

%% 5 Rot to angle-axis with eigenvectors

disp("---5---")

% --- Q5.1 ---
R = [1 0 0; 0 0 -1; 0 1 0];

[h, theta] = RotToAngleAxis(R);

[V,D] = eig(R);
V = real(V);
D = real(D);

for i = 1:3
    if kEq(D(i, i), 1)
        h_star = V(:, i);
    end
end

theta_star = acos((trace(R)-1)/2);

rho_star = h_star*theta_star;
rho = h*theta;

err_q51 = norm(rho - rho_star)

% --- Q5.2 ---
R = 1/9 * [4 -4 -7; 8 1 4; -1 -8 4];

[h, theta] = RotToAngleAxis(R);

[V,D] = eig(R);
V = real(V);
D = real(D);

for i = 1:3
    if kEq(D(i, i), 1)
        h_star = V(:, i);
    end
end

theta_star = acos((trace(R)-1)/2);

rho_star = h_star*theta_star;
rho = h*theta;

err_q52 = norm(rho - rho_star)

%% Frame rotation visualization
% t0 = eye(3);
% 
% YPR_q34 = [pi/3; pi/2; pi/4];
% 
% figure
% subplot(2, 3, 1), plotFrame(Rx(YPR_q34(3))*t0, ""), axis equal
% subplot(2, 3, 1), plotFrame(t0, "", ["r--" "g--" "b--"])
% 
% subplot(2, 3, 2), plotFrame(Ry(YPR_q34(2))*Rx(YPR_q34(3))*t0, ""), axis equal
% subplot(2, 3, 2), plotFrame(Rx(YPR_q34(3))*t0, "", ["r--" "g--" "b--"])
% 
% subplot(2, 3, 3), plotFrame(Rz(YPR_q34(1))*Ry(YPR_q34(2))*Rx(YPR_q34(3))*t0, ""), axis equal
% subplot(2, 3, 3), plotFrame(Ry(YPR_q34(2))*Rx(YPR_q34(3))*t0, "", ["r--" "g--" "b--"])
% 
% YPR_q35 = [0; pi/2; -pi/12];
% subplot(2, 3, 4), plotFrame(Rx(YPR_q35(3))*t0, ""), axis equal
% subplot(2, 3, 4), plotFrame(t0, "", ["r--" "g--" "b--"])
% 
% subplot(2, 3, 5), plotFrame(Ry(YPR_q35(2))*Rx(YPR_q35(3))*t0, ""), axis equal
% subplot(2, 3, 5), plotFrame(Rx(YPR_q35(3))*t0, "", ["r--" "g--" "b--"])
% 
% subplot(2, 3, 6), plotFrame(Rz(YPR_q35(1))*Ry(YPR_q35(2))*Rx(YPR_q35(3))*t0, ""), axis equal
% subplot(2, 3, 6), plotFrame(Ry(YPR_q35(2))*Rx(YPR_q35(3))*t0, "", ["r--" "g--" "b--"])