clc; clear; close all; rosshutdown;

%path = pathPlanning("fmap", "variables/path_fmap.mat");

% Parâmetros
params.kv     = 2.0;
params.ki     = 0.1;
params.ks     = 3.0;
params.d_star = 0;
params.rate   = 20;
params.dt     = 0.05;
params.T      = 60;
map    = loadMap("ymap");
params.map = map;
params.origin = 0;
params.scale  = 20;

avoidance = "vfh";

% Path
data     = load("variables/path_ymap.mat");
path     = data.path;
xWorld   = (path(:,1) - params.origin) / params.scale;
yWorld   = (path(:,2) - params.origin) / params.scale;
pathWorld = [xWorld, yWorld];

% Inicializa robô
tbot = connectRobot("sim");
tbot.setPose(pathWorld(1,1), pathWorld(1,2), pi);

% Prepara figura e devolve os handles
handles = setupPlot(map, pathWorld, params.origin, params.scale);

% Tracking
PathTracking(tbot, params, pathWorld, handles, avoidance);

tbot.setVelocity(0, 0);
