clc;
clear;
close all;
rosshutdown;

addpath(fullfile(pwd, '../utils'));
addpath(fullfile(pwd, '../controllers'));

tbot = connectRobot("sim");

tbot.setPose(0, 0, 0);

robot_radius = 0.105;   % in meters
scale = 20;             % Map scale
robot_pixels = round(robot_radius * scale);

map = loadMap("ymap");

%Inflate objects boundaries according to robot dimension
se = strel('disk', robot_pixels);
map = imdilate(map, se);

% 
% path = pathPlanning(map);


% Controller parameters
kv = 2.5;
ki = 0.1;
ks = 1.0;
rate = 5;
d_star = 0;

dt = 0.05;
T = 60;

% Trajectory
origin = 0;   % Origin of the grid map
scale = 20;     % Scale of the grid map compared to the real world

data = load("variables/path_ymap.mat"); % Extract path
path = data.path;       

N = length(path);

xMap = path(:,1); 
yMap = path(:,2); 

% Convert map coordinates to global map coordinates 
xWorld = (xMap - origin) / scale;
yWorld = (yMap - origin) / scale;

pathWorld = [xWorld, yWorld];

initialPose = pathWorld(1, :);

tbot.setPose(initialPose(1), initialPose(2), 0);

while true
    [x, y, theta, timestamp] = tbot.readPose();

    if abs(initialPose(1) - x) < 0.02 && abs(initialPose(2) - y) < 0.02
        break;
    end

    pause(0.1);
end


% Variables
e_int = 0;
target_index = 1;

traj = [];

figure
hold on

% Plot do caminho desejado
hpath = plot(pathWorld(:,1), pathWorld(:,2), 'r--', 'LineWidth', 2);

% Trajetória do robô (vai sendo atualizada)
htraj = plot(0, 0, 'b', 'LineWidth', 2);

% Posição atual do robô
hrobot = plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

htarget = plot(0, 0, 'go', 'MarkerSize', 10, 'LineWidth', 2);

axis equal
grid on

legend('Desired Path', 'Robot Trajectory', 'Robot Position')
xlabel('x (m)')
ylabel('y (m)')
title('Real-time trajectory tracking')

% Control loop
for t = 0:dt:T

    % Get robot pose from odometry
    [x, y, theta, timestamp] = tbot.readPose();
    theta = normalizeAngle(theta);

    % Target point
    % x_star = pathWorld(target_index,1);
    % y_star = pathWorld(target_index,2);
    % 
    [Fa, Fr] = VFF([x, y], pathWorld(target_index, :), map, 10);
    F = Fa + Fr;
    
    F_norm = norm(F);

    if F_norm < 1e-6
        dir = [0, 0];
    else
        dir = F / F_norm;
    end
    
    L = min(0.2, norm(pathWorld(target_index,:) - [x y])); % ← IMPORTANT
    
    x_star = x + L * dir(1);
    y_star = y + L * dir(2);

    % Tracking error
    e = sqrt((x_star-x)^2 + (y_star-y)^2) - d_star;

    % Integral error
    e_int = e_int + e*dt;

    % Linear velocity
    linear_velocity = kv*e + ki*e_int;

    % Desired angle
    phi_star = atan2(y_star - y , x_star - x);

    % Angular error normalization
    ang_error = atan2(sin(phi_star-theta),cos(phi_star-theta));

    % Angular velocity
    angular_velocity = ks*ang_error;


    % Send command to robot
    tbot.setVelocity(linear_velocity, angular_velocity);

    % Save trajectory
    traj = [traj; x y];

    % Atualiza trajetória
    set(htraj, 'XData', traj(:,1), 'YData', traj(:,2));
    
    % Atualiza posição atual do robô
    set(htarget, 'XData', x_star, 'YData', y_star);
    set(hrobot, 'XData', x, 'YData', y);
    
    drawnow

    % Switch to next path point
    %dist = sqrt((x_star-x)^2 + (y_star-y)^2);
    dist = norm([x y] - pathWorld(target_index,:));
    if dist < 0.1 && target_index < N
        target_index = target_index + 1;
    end

    if target_index >= N && dist < 0.1
        break;
    end

    pause(dt)
end
disp(t);

% Stop robot
tbot.setVelocity(0,0);