rosshutdown;

tbot = connectRobot("sim");

params.scale   = 20;
params.origin  = 0;
params.mapSize = 80;
params.rate    = 5;
params.plotSkip = 10;

tbot.setPose(2, 2, 0);
mapBuilding2(tbot, params, "save")