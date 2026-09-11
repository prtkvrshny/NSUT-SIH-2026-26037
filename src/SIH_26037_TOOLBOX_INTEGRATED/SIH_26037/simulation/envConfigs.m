function cfg = envConfigs(envName)
% envConfigs  Central definition of the 5 Indian-road environments.
% Implements Phase F items 22-26. A SINGLE planner
% (planning/adaptivePlanner.m) is reused for every environment; only
% this config differs, per the "one core engine, not five hardcoded
% demos" requirement -- simulation/runScenario.m is the one function
% that dispatches on envName via this file.
%
%   cfg = envConfigs(envName)
%
% envName: 'village' | 'intersection' | 'highway' | 'market' | 'cattle'
%
% Output cfg fields:
%   .corridorHalfWidth, .roadLength
%   .egoStart [x y theta v], .egoGoalX, .targetSpeed
%   .obstacles  - struct array, see initObstacles.m
%   .simDuration, .numTrials

cfg.corridorHalfWidth = 4.0;
cfg.roadLength = 200;
cfg.egoStart = [0, 0, 0, 5];
cfg.egoGoalX = cfg.roadLength - 10;
cfg.simDuration = 40;
cfg.numTrials = 5;

switch envName
    case 'village'
        cfg.targetSpeed = 6;
        cfg.obstacles    = mkObs('cattle',      40,  1.5, pi,    0.0, 'randomWalk',   0, struct('nominalSpeed',1.0));
        cfg.obstacles(2) = mkObs('pedestrian',  70, -2.0, pi/2,  1.2, 'crossing',     0);
        cfg.obstacles(3) = mkObs('pushcart',    90,  1.0, 0,     1.0, 'straightLine', 0);
        cfg.obstacles(4) = mkObs('bike',       120, -1.5, 0,     4.0, 'weaving',      5);

    case 'intersection'
        cfg.targetSpeed = 8;
        cfg.obstacles    = mkObs('auto',  60, -3.0,  pi/2, 6.0, 'crossing', 0);
        cfg.obstacles(2) = mkObs('bike',  65,  3.5, -pi/2, 5.0, 'crossing', 2);
        cfg.obstacles(3) = mkObs('pedestrian', 80, 0, pi/2, 1.3, 'crossing', 4);

    case 'highway'
        cfg.targetSpeed = 16;
        cfg.obstacles    = mkObs('auto', 100,  1.0, 0, 12.0, 'straightLine', 0);
        cfg.obstacles(2) = mkObs('bike',  60, -1.0, 0,  9.0, 'weaving',      0);
        cfg.obstacles(3) = mkObs('auto', 150,  0.5, 0, 14.0, 'straightLine', 6);

    case 'market'
        cfg.targetSpeed = 3;
        cfg.corridorHalfWidth = 3.0;
        % Market target speed is deliberately low (dense foot traffic);
        % a 200 m road at ~3 m/s cruise needs >=67 s just to traverse
        % even with zero obstacle-induced slowdown, so shorten the road
        % and extend the sim budget rather than leave an unwinnable
        % "completion %" metric. Found via Octave test runs, see NOTES.md.
        cfg.roadLength = 120;
        cfg.egoGoalX = cfg.roadLength - 10;
        cfg.simDuration = 55;
        cfg.obstacles    = mkObs('pedestrian', 20,  1.0, pi,   1.0, 'randomWalk',   0, struct('nominalSpeed',0.8));
        cfg.obstacles(2) = mkObs('pedestrian', 35, -1.5, pi/2, 1.1, 'crossing',     0);
        cfg.obstacles(3) = mkObs('pushcart',   50,  0.5, 0,    0.8, 'straightLine', 0);
        cfg.obstacles(4) = mkObs('pedestrian', 65, -0.5, 0,    0.0, 'randomWalk',   3, struct('nominalSpeed',0.9));
        cfg.obstacles(5) = mkObs('pushcart',   80,  2.0, pi,   0.7, 'straightLine', 0);

    case 'cattle'
        cfg.targetSpeed = 5;
        % Dense close-range erratic obstacles slow average progress well
        % below targetSpeed; 40 s left this short of the goal in testing.
        cfg.simDuration = 55;
        cfg.obstacles    = mkObs('cattle', 50,  0.0, 0,  0.0, 'erratic', 0, struct('nominalSpeed',1.2));
        cfg.obstacles(2) = mkObs('cattle', 55, -2.0, 0,  0.0, 'erratic', 0, struct('nominalSpeed',1.0));
        cfg.obstacles(3) = mkObs('cattle', 90,  1.5, pi, 0.0, 'erratic', 8, struct('nominalSpeed',1.1));
        cfg.obstacles(4) = mkObs('bike',  120, -1.0, 0,  4.0, 'weaving', 0);

    otherwise
        error('envConfigs:unknown', 'Unknown environment "%s"', envName);
end
end

function o = mkObs(type, x0, y0, theta0, v0, behavior, spawnTime, extraParams)
if nargin < 8, extraParams = struct(); end
o.type = type; o.x0 = x0; o.y0 = y0; o.theta0 = theta0; o.v0 = v0;
o.behavior = behavior; o.spawnTime = spawnTime; o.params = extraParams;
end
