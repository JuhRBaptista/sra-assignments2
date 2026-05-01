rosshutdown;

tbot = connectRobot("sim");

params.scale   = 20;
params.origin  = 0;
params.mapSize = 80;
params.rate    = 5;

mapBuilding(tbot, params, "save")