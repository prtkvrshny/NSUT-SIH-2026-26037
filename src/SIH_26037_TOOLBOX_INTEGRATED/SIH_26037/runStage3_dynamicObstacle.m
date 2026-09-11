% runStage3_dynamicObstacle.m
% Stage 3: one DYNAMIC obstacle. Current state -> predict future position
% -> check future collision -> replan. First stage where the planner must
% react to where an obstacle WILL be, not just where it IS.

setupProject();

roadLength = 120;
centerline = [0 0; roadLength 0];
veh = initVehicleState(0, 0, 0, 6);
dt = 0.02;
nSteps = round(25/dt);

% Bike crossing the road path perpendicular to the direction of travel.
obsCfg = struct('type','bike','x0',60,'y0',-6,'theta0',pi/2,'v0',4.0, ...
                 'behavior','straightLine','spawnTime',0,'params',struct());
obstacles = initObstacles(obsCfg);
tracks = initTrackList();

xs = zeros(nSteps,1); ys = zeros(nSteps,1); modes = cell(nSteps,1);
obsXs = zeros(nSteps,1); obsYs = zeros(nSteps,1);
plannerState = [];
planParams = struct('corridorHalfWidth',4,'targetSpeed',7);
roadInfo.centerline = centerline; roadInfo.halfWidth = 4;

lastStep = nSteps;
for step = 1:nSteps
    t = (step-1)*dt;
    obstacles = obstacleUpdate(obstacles, dt, t, roadInfo);

    fused = struct('x',{},'y',{},'bearing',{},'rangeRate',{},'type',{},'confidence',{},'trueId',{});
    for k = 1:numel(obstacles)
        o = obstacles(k);
        f.x=o.x; f.y=o.y; f.bearing=atan2(o.y-veh.y,o.x-veh.x)-veh.theta;
        f.rangeRate=NaN; f.type=o.type; f.confidence=1; f.trueId=o.id;
        fused(end+1) = f;
    end
    tracks = trackObstacles(tracks, fused, veh, dt, struct());

    [ctrl, plannerState, diag] = adaptivePlanner(veh, tracks, centerline, plannerState, planParams);
    veh = bicycleModel(veh, ctrl.a, ctrl.delta, dt);

    xs(step)=veh.x; ys(step)=veh.y; modes{step}=diag.mode;
    obsXs(step)=obstacles(1).x; obsYs(step)=obstacles(1).y;
    if veh.x >= roadLength-5
        lastStep = step;
        break;
    end
end
xs=xs(1:lastStep); ys=ys(1:lastStep); modes=modes(1:lastStep);
obsXs=obsXs(1:lastStep); obsYs=obsYs(1:lastStep);

minDist = min(hypot(xs-obsXs, ys-obsYs));
figure; plot(xs,ys,'b-','LineWidth',1.5); hold on; plot(obsXs,obsYs,'r-','LineWidth',1.5);
xlabel('x [m]'); ylabel('y [m]'); axis equal; grid on;
legend('ego','bike (dynamic obstacle)');
title('Stage 3: dynamic obstacle -- predict & replan');

fprintf('Stage 3 complete: min ego-obstacle distance over run = %.2f m\n', minDist);
