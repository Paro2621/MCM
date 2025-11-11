addpath('include');
addpath('utils');
addpath('utils/plot');

clear, close, clc;

%% 1.1 Angle-axis to rot

% --- Q1.2 ---
h = [1; 0; 0];
theta = deg2rad(90);

R_q12 = AngleAxisToRot(h, theta)

% --- Q1.3 ---
h = [0; 0; 1];
theta = pi/3;

R_q13 = AngleAxisToRot(h, theta)

% --- Q1.4 ---
rho = [-pi/3; -pi/6; pi/3];

theta = norm(rho);
h = rho/theta;

R_q14 = AngleAxisToRot(h, theta)

%% 1.2 Rot to angle-axis

% --- Q2.2 ---
R = [1 0 0 ; 0 0 -1; 0 1 0];
[h_q22, theta_q22] = RotToAngleAxis(R)

% --- Q2.3 ---
R = [0.5 -sqrt(3)/2 0; sqrt(3)/2 0.5 0; 0 0 1];
[h_q23, theta_q23] = RotToAngleAxis(R)

% --- Q2.4 ---
R = [1 0 0; 0 1 0; 0 0 1];
[h_q24, theta_q24] = RotToAngleAxis(R)

% --- Q2.5 ---
R = [-1 0 0; 0 -1 0; 0 0 1];
[h_q25, theta_q25] = RotToAngleAxis(R)

% --- Q2.6 ---
R = [-1 0 0; 0 1 0; 0 0 1];
% the aforedmentioned R is not a rotation matrix

%% 1.3 Euler to rot
% YPR = [psi; theta; phi];

% --- Q3.2 ---
YPR = [0; 0; pi/2];
R_q32 = YPRToRot(YPR)

% --- Q3.3 ---
YPR = [deg2rad(60); 0; 0];
R_q33 = YPRToRot(YPR)

% --- Q3.4 ---
YPR_q34 = [pi/3; pi/2; pi/4];
R_q34 = YPRToRot(YPR_q34)
% [!] GIMBAL LOCK DETECTED

% --- Q3.5 ---
YPR_q35 = [0; pi/2; -pi/12];
R_q35 = YPRToRot(YPR_q35)
% [!] GIMBAL LOCK DETECTED

% NOTE: R_q35 and R_q34 are the same! 
% In case of gimbal lock, the first (yaw) and last (roll) rotations both 
% act about the same axis, so only their sum matters.
YPR_q34_prime = [YPR_q34(1) - YPR_q34(3); pi/2; 0]
YPR_q35_prime = [YPR_q35(1) - YPR_q35(3); pi/2; 0]
% so, it's no surprise that the two sets of YPR refer to the same rotation

%% 1.4 Rot to Euler

% --- Q4.2 ---
R = [1 0 0; 0 0 -1; 0 1 0];
YPR_q42 = RotToYPR(R)

% --- Q4.3 ---
R = [1/2 -sqrt(3)/2 0; sqrt(3)/2 1/2 0; 0 0 1];
YPR_q43 = RotToYPR(R)

% --- Q4.4 ---
R = [0          -sqrt(2)/2  sqrt(2)/2; ...
    0.5         sqrt(6)/4   sqrt(6)/4; ...
    -sqrt(3)/2  sqrt(2)/4   sqrt(2)/4];

YPR_q44 = RotToYPR(R)
% gimbal lock detected

%% 1.5 Rot to angle-axis with eigenvectors
% [!] Attention, since the eigenvector can be scaled for any real lambda,
% negative and positive, this methods do not make distintion between [-avt]
% and [+avt], thus, it is not suitable for this applcation w/out additional
% checks. The use of RotToAngleAxis() is suggested

% --- Q5.1 ---
R = [1 0 0; 0 0 -1; 0 1 0];

[h_q51, theta_q51] = RotToAngleAxis(R)

[V,D] = eig(R);
V = real(V);
D = real(D);

for i = 1:3
    if kEq(D(i, i), 1)
        h_star_q51 = V(:, i)
    end
end

theta_star_q51 = acos((trace(R)-1)/2)

% --- Q5.2 ---
R = 1/9 * [4 -4 -7; 8 1 4; -1 -8 4];

[h_q52, theta_q52] = RotToAngleAxis(R)

[V,D] = eig(R);
V = real(V);
D = diag(real(D));

for i = 1:3
    if kEq(D(i), 1)
        h_star_q52 = V(:, i)
    end
end

theta_star_q52 = acos((trace(R)-1)/2)