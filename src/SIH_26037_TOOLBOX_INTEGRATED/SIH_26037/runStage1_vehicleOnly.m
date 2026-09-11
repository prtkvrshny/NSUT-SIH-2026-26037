% runStage1_vehicleOnly.m
% Stage 1: bicycle-model vehicle on an empty road, start -> goal, plotted.
% No AI, no sensors, no obstacles -- proves the base simulation loop
% (vehicle/bicycleModel.m + vehicle/purePursuitControl.m) before anything
% else is layered on top.

setupProject();

roadLength = 100;
centerline = [0 0; roadLength 0];
veh = initVehicleState(0, 0, 0, 5);
dt = 0.02;
nSteps = round(20/dt);

xs = zeros(nSteps,1); ys = zeros(nSteps,1);
lastStep = nSteps;
for step = 1:nSteps
    [a, delta, ~] = purePursuitControl(veh, centerline, 8, 6);
    veh = bicycleModel(veh, a, delta, dt);
    xs(step) = veh.x; ys(step) = veh.y;
    if veh.x >= roadLength - 5
        lastStep = step;
        break;
    end
end
xs = xs(1:lastStep); ys = ys(1:lastStep);

figure; plot(xs, ys, 'b-', 'LineWidth', 1.5); hold on;
plot(centerline(:,1), centerline(:,2), 'k--');
xlabel('x [m]'); ylabel('y [m]'); axis equal; grid on;
title('Stage 1: vehicle on empty road, start -> goal');
legend('vehicle path','road centerline');

fprintf('Stage 1 complete: reached x=%.1f m (goal was %.1f m)\n', xs(end), roadLength-5);
