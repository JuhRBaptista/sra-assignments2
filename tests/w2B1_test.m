close all;
clear all;
rosshutdown;

addpath(fullfile(pwd, 'utils'));
addpath(fullfile(pwd, 'controllers'));

% Control Parameters
params.maxIterations = 500;     
params.rate = 5;   
params.target = [1, 1, pi];
params.start = [0, 0, 0];
params.kpRho = 1.5;
params.kpBeta = -0.4;
params.kpAlpha = 2;
params.toleranceErrorAngle = 0.4;
params.toleranceErrorDist = 0.05;
params.vMax = 0.18;
params.wMax = 2.85;

tbot = connectRobot("sim");

resetOdometry(tbot, params.start);

PoseToPoseControl(tbot, params);