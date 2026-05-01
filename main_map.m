rosshutdown;
clear classes;
clc;
pause(1); % dá tempo pro ROS desligar

params.kv     = 2.0;
params.ki     = 0.1;
params.ks     = 3.0;
params.d_star = 0;
params.rate   = 20;
params.dt     = 0.05;
params.T      = 200;

% Parameters
params.scale   = 20;
params.origin  = 0;
params.mapSize = 80;
params.rate    = 5;
avoidance = "none";

% Path
data     = load("variables/path_ymap4.mat");
path     = data.path;
xWorld   = (path(:,1) - params.origin) / params.scale;
yWorld   = (path(:,2) - params.origin) / params.scale;
pathWorld = [xWorld, yWorld];

% Inicializa robô
tbot = connectRobot("sim");
tbot.setPose(pathWorld(1,1), pathWorld(1,2), pi);

save_path = "map";

tbot.stop();

basicSLAM(tbot, params, path, avoidance, save_path);
