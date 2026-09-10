% runStage4_indianRoadBrain.m
% Stage 4: the "Indian road brain" -- multiple obstacle types
% (pedestrian, bike, auto, pushcart, cattle), each with distinct
% size/speed/behavior, no lane-following assumption. Demonstrates the
% traversability map, prediction, risk scoring and adaptive planning
% working together on a mixed obstacle set (ground-truth perception --
% sensors are added in Stage 5).

setupProject();

roadLength = 150;
centerline = [0 0; roadLength 0];
veh = initVehicleState(0, 0, 0, 5);
dt = 0.02;
nSteps = round(35/dt);

obsCfgs(1) = struct('type','pedestrian','x0',40,'y0',-2,'theta0',pi/2,'v0',1.2,'behavior','crossing','spawnTime',0,'params',struct());
obsCfgs(2) = struct('type','cattle','x0',65,'y0',0.5,'theta0',0,'v0',0,'behavior','erratic','spawnTime',0,'params',struct('nominalSpeed',1.0));
obsCfgs(3) = struct('type','bike','x0',90,'y0',-1.5,'theta0',0,'v0',4.5,'behavior','weaving','spawnTime',0,'params',struct());
obsCfgs(4) = struct('type','pushcart','x0',110,'y0',1.0,'theta0',0,'v0',0.8,'behavior','straightLine','spawnTime',0,'params',struct());
obstacles = initObstacles(obsCfgs);
tracks = initTrackList();

plannerState = [];
planParams = struct('corridorHalfWidth',4,'targetSpeed',6);
roadInfo.centerline = centerline; roadInfo.halfWidth = 4;

xs=zeros(nSteps,1); ys=zeros(nSteps,1); modes=cell(nSteps,1); risks=zeros(nSteps,1);
lastStep = nSteps;

for step = 1:nSteps
    t=(step-1)*dt;
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

    xs(step)=veh.x; ys(step)=veh.y; modes{step}=diag.mode; risks(step)=diag.maxRisk;
    if veh.x >= roadLength-5
        lastStep = step;
        break;
    end
end
xs=xs(1:lastStep); ys=ys(1:lastStep); modes=modes(1:lastStep); risks=risks(1:lastStep);

figure;
subplot(2,1,1);
plot(xs,ys,'b-','LineWidth',1.5); hold on;
for k=1:numel(obstacles)
    plot(obstacles(k).x, obstacles(k).y, 'rx','MarkerSize',10);
    text(obstacles(k).x, obstacles(k).y, [' ' obstacles(k).type]);
end
axis equal; grid on; xlabel('x [m]'); ylabel('y [m]');
title('Stage 4: mixed obstacle types, no lane assumption');
subplot(2,1,2);
plot((0:numel(risks)-1)*dt, risks, 'r-'); xlabel('t [s]'); ylabel('max risk');
grid on; title('risk score over time');

fprintf('Stage 4 complete: final mode=%s, peak risk=%.2f\n', modes{end}, max(risks));
