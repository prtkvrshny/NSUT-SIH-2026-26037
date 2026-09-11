% runStage5_sensors.m
% Stage 5: sensors added ONE AT A TIME: ground truth -> LiDAR -> radar
% -> camera -> full fusion + tracking + prediction + risk + adaptive
% planner. Each mode below runs the same 'village' scenario so results
% can be compared directly.
%
% NOTE: because this file cannot be executed in MATLAB here (see
% constraints in NOTES.md -- it HAS been run in GNU Octave, a different
% but closely related environment, during development), the spirit of
% "each proven working before the next" is best honored by running one
% sensorMode at a time yourself in MATLAB and inspecting the plot/console
% output, rather than trusting this whole script blind end to end.

setupProject();

modes = {'groundtruth','lidar','radar','camera','fusion'};
results = struct('mode',{},'collision',{},'completed',{},'meanLatencyMs',{});

for m = 1:numel(modes)
    thisMode = modes{m};
    fprintf('\n=== Stage 5 sensor mode: %s ===\n', thisMode);
    [log, metrics] = runScenario('village', struct('sensorMode', thisMode, 'seed', 1, 'showPlot', true));
    fprintf('collision=%d completed=%d meanReplanLatency=%.4f s\n', ...
            metrics.collision, metrics.completed, metrics.meanReplanLatency);
    r.mode = thisMode; r.collision = metrics.collision;
    r.completed = metrics.completed; r.meanLatencyMs = 1000*metrics.meanReplanLatency;
    results(end+1) = r;
end

disp(struct2table(results));
