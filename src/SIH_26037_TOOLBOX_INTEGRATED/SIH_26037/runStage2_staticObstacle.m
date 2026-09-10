% runStage2_staticObstacle.m
% Stage 2: one static obstacle. Detect (ground truth here -- sensors are
% added in Stage 5) -> generate candidate paths -> choose a safe path ->
% follow it. First real version of the adaptive planner.

setupProject();

roadLength = 100;
centerline = [0 0; roadLength 0];
veh = initVehicleState(0, 0, 0, 5);
dt = 0.02;
nSteps = round(35/dt);

obsCfg = struct('type','auto','x0',50,'y0',0.5,'theta0',0,'v0',0, ...
                 'behavior','static','spawnTime',0,'params',struct());
obstacles = initObstacles(obsCfg);
tracks = initTrackList();

xs = zeros(nSteps,1); ys = zeros(nSteps,1); modes = cell(nSteps,1);
plannerState = [];
planParams = struct('corridorHalfWidth',4,'targetSpeed',6);
roadInfo.centerline = centerline; roadInfo.halfWidth = 4;

lastStep = nSteps;
for step = 1:nSteps
    t = (step-1)*dt;
    obstacles = obstacleUpdate(obstacles, dt, t, roadInfo);

    % Ground-truth "detection" -- no sensor noise yet, see Stage 5.
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
    if veh.x >= roadLength-5
        lastStep = step;
        break;
    end
end
xs=xs(1:lastStep); ys=ys(1:lastStep); modes=modes(1:lastStep);

figure; plot(xs,ys,'b-','LineWidth',1.5); hold on;
plot(obstacles(1).x, obstacles(1).y, 'rs', 'MarkerSize',12,'MarkerFaceColor','r');
xlabel('x [m]'); ylabel('y [m]'); axis equal; grid on;
title('Stage 2: single static obstacle avoidance');

fprintf('Stage 2 complete: final mode=%s, reached x=%.1f m\n', modes{end}, xs(end));
