addpath('include');
addpath('utils');
addpath('utils/plot');

clear, close, clc;

%% Geometria con terne calcolate a mano
% lunghezza in decimetri 

figure
T_o = tFactory([1 0 0; 0 1 0; 0 0 1]', [0 0 0]');
plotFrame(T_o), hold on, axis equal

o_T_a = tFactory([1 0 0; 0 1 0; 0 0 1]', [0 0 1.75]');
plotFrame(T_o*o_T_a)

a_T_b = tFactory([-1 0 0; 0 0 1; 0 1 0]', [0 0 0.98]');
plotFrame(T_o*o_T_a*a_T_b),

b_T_c = tFactory([0 0 -1; 0 1 0; 1 0 0]', [1.05 0 0]');
plotFrame(T_o*o_T_a*a_T_b*b_T_c),

c_T_cp = tFactory([1 0 0; 0 1 0; 0 0 1]', [0 0 3.265]');
plotFrame(T_o*o_T_a*a_T_b*b_T_c*c_T_cp, ["--r", "--g", "--b"]),

cp_T_d = tFactory([0 0 -1; 0 -1 0; -1 0 0]', [0 1.455 0]');
plotFrame(T_o*o_T_a*a_T_b*b_T_c*c_T_cp*cp_T_d),

d_T_e = tFactory([0 -1 0; 0 0 -1; 1 0 0]', [0.35 0 0]');
plotFrame(T_o*o_T_a*a_T_b*b_T_c*c_T_cp*cp_T_d*d_T_e),

e_T_f = tFactory([0 0 1; 1 0 0; 0 1 0]', [0 0 3.85]');
plotFrame(T_o*o_T_a*a_T_b*b_T_c*c_T_cp*cp_T_d*d_T_e*e_T_f),

f_T_g = tFactory([0 1 0; 0 0 1; 1 0 0]', [1.53 0 0]');
plotFrame(T_o*o_T_a*a_T_b*b_T_c*c_T_cp*cp_T_d*d_T_e*e_T_f*f_T_g),
view([-14.10 10.69])

%% Geometria in notazione DH



