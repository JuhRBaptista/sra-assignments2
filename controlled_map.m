rosshutdown;

tbot = connectRobot("sim");

params.scale   = 20;
params.origin  = 0;
params.mapSize = 80;
params.rate    = 5;
params.plotSkip = 5;

tbot.setPose(0.3, 0.3, pi/2);
mapBuilding(tbot, params, "save")