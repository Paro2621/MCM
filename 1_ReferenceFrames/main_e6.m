addpath('include');
addpath('utils');
addpath('utils/plot');

clear, close, clc;

%% Geometria con terne calcolate a mano
% lunghezza in decimetri 

figure
To = tFactory([1 0 0; 0 1 0; 0 0 1]', [0 0 0]');
plotFrame(To, '<0>'), hold on, axis equal

oTa = tFactory([1 0 0; 0 1 0; 0 0 1]', [0 0 1.75]');
plotFrame(oTa, '<1>')

aTb = tFactory([-1 0 0; 0 0 1; 0 1 0]', [0 0 0.98]');
plotFrame(oTa*aTb, '<2>')

bTc = tFactory([0 0 -1; 0 1 0; 1 0 0]', [1.05 0 0]');
plotFrame(oTa*aTb*bTc, '<3>')

cTcp = tFactory([1 0 0; 0 1 0; 0 0 1]', [0 0 3.265]');
% plotFrame(oTa*aTb*bTc*cTcp, '<3*>', ["--r", "--g", "--b"])

cpTd = tFactory([0 0 -1; 0 -1 0; -1 0 0]', [0 1.455 0]');
plotFrame(oTa*aTb*bTc*cTcp*cpTd, '<4>')

dTe = tFactory([0 -1 0; 0 0 -1; 1 0 0]', [0.35 0 0]');
plotFrame(oTa*aTb*bTc*cTcp*cpTd*dTe, '<5>')

eTf = tFactory([0 0 1; 1 0 0; 0 1 0]', [0 0 3.85]');
plotFrame(oTa*aTb*bTc*cTcp*cpTd*dTe*eTf, '<6>')

fTg = tFactory([0 1 0; 0 0 1; 1 0 0]', [1.53 0 0]');
plotFrame(oTa*aTb*bTc*cTcp*cpTd*dTe*eTf*fTg, '<7>'),
view([-14.10 10.69])

oTa*aTb*bTc*cTcp*cpTd*dTe*eTf*fTg;

%% w/out intermediate step
% the results are the same as before but now enfathizing the fact that the
% rotation between an initial and final frame

x = [1 0 0]';
y = [0 1 0]';
z = [0 0 1]';

figure
To = tFactory([x y z], [0 0 0]');
plotFrame(To, '<0>'), hold on, axis equal

oTa = tFactory([x y z], [0 0 1.75]');
plotFrame(To*oTa, '<1>')

aTb = tFactory([-x z y], [0 0 0.98]');
plotFrame(To*oTa*aTb, '<2>'),

bTc = tFactory([-z y x], [1.05 0 0]');
plotFrame(To*oTa*aTb*bTc, '<3>'),

c_T_d = tFactory([-z -y -x], [0 1.455 3.265]');
plotFrame(To*oTa*aTb*bTc*c_T_d, '<4>'),

dTe = tFactory([-y -z x], [0.35 0 0]');
plotFrame(To*oTa*aTb*bTc*c_T_d*dTe, '<5>'),

eTf = tFactory([z x y], [0 0 3.85]');
plotFrame(To*oTa*aTb*bTc*c_T_d*dTe*eTf, '<6>'),

fTg = tFactory([y z x], [1.53 0 0]');
plotFrame(To*oTa*aTb*bTc*c_T_d*dTe*eTf*fTg, '<7>'),
view([-14.10 10.69])

%% Geometria in notazione DH
% close all; 
% clear, clc; 
% 
% robot = loadrobot("fanucLRMate200ib");
% % showdetails(robot);
% 
% config = homeConfiguration(robot);
% 
% figure
% show(robot);



