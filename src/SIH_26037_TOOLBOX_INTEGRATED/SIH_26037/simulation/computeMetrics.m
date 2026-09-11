function metrics = computeMetrics(log, dtSim)
% computeMetrics  Compute the Phase G per-run metrics from a
% simulation/runScenario.m log.
% Implements Phase G items 27-33.
%
%   metrics = computeMetrics(log, dtSim)

metrics.collision = log.collision;                     % item 27 (per-run; aggregated into a rate by runDemo.m)
metrics.completed = log.reachedGoal && ~log.collision;  % item 28 (per-run; aggregated into a % by runDemo.m)

validLatency = log.planLatency(~isnan(log.planLatency));
if isempty(validLatency)
    metrics.meanReplanLatency = NaN;
    metrics.maxReplanLatency = NaN;
else
    metrics.meanReplanLatency = mean(validLatency);     % item 29 (real wall-clock, from tic/toc in adaptivePlanner.m)
    metrics.maxReplanLatency = max(validLatency);
end

% Path smoothness proxy: mean absolute steering-angle rate (lower =
% smoother). See NOTES.md for why this, rather than path curvature
% directly, was chosen.
if numel(log.delta) > 1
    deltaRate = diff(log.delta) / dtSim;
    metrics.pathSmoothness = mean(abs(deltaRate));      % item 30
else
    metrics.pathSmoothness = 0;
end

finiteTTC = log.ttc(isfinite(log.ttc));
if isempty(finiteTTC)
    metrics.minTTC = NaN;
else
    metrics.minTTC = min(finiteTTC);                    % item 31
end

metrics.emergencyBrakingEvents = log.emergencyEvents;   % item 32
metrics.avgSpeed = mean(log.vehV);                      % item 33
end
