% INITIALIZATION
close all;
clear all;

addpath(fullfile(pwd, 'utils'));
addpath(fullfile(pwd, 'controllers'));

% DEFINITIONS
params.maxIterations = 150;
params.target = [1, 1];
params.toleranceError = 0.2;
params.kV = 2;
params.kW = 1;
params.rate = 5;

% INITIALIZE TURTLEBOT POSE
tbot = connectRobot("sim");
tbot.setPose(0, 0, 0.00);

% Control
PointToPointControl(tbot, params);