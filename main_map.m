rosshutdown;
clear classes;
clc;
pause(1);

% Parâmetros de controlo
params.kv     = 2.0;
params.ki     = 0.1;
params.ks     = 3.0;
params.d_star = 0;
params.dt     = 0.05;
params.T      = 2000;

% Parâmetros do mapa 
params.scale   = 20;
params.origin  = 0;
params.mapSize = 80;
params.rate    = 50;   % só uma definição
params.plotSkip = 1;


avoidance = "none";

% Carregar e converter path para coordenadas mundo
data      = load("variables/path_csqmap.mat");
path      = data.path;
xWorld    = (path(:,1) - params.origin) / params.scale;
yWorld    = (path(:,2) - params.origin) / params.scale;
pathWorld = [xWorld, yWorld];   % <-- isto é o que basicSLAM precisa

% --- Inicializar robô ---
tbot = connectRobot("sim");
tbot.setPose(pathWorld(1,1), pathWorld(1,2), pi/2);
tbot.stop();

% --- Correr SLAM + navegação ---
mapBuildingWithTracking(tbot, params, pathWorld, "variables/map", "vff");