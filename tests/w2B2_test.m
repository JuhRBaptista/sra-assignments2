clc;
clear;
close all;
rosshutdown;

addpath(fullfile(pwd, 'utils'));
addpath(fullfile(pwd, 'controllers'));

tbot = connectRobot("sim");

tbot.setPose(0, 0, 0);

% Controller parameters
params.kv = 2.0;
params.ki = 0.1;
params.ks = 3.0;
params.rate = 5;
params.d_star = 0;

params.dt = 0.05;
params.T = 60;

params.maxIterations = 350;

x_center = 0;
y_center = 0;
R = 1;
N = 100;

theta = linspace(0,2*pi,N);

x_path = x_center + R*cos(theta);
y_path = y_center + R*sin(theta);

path = [x_path' y_path'];

PathTracking(tbot, params, path);
