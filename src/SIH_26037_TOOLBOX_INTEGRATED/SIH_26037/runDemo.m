function runDemo()
% runDemo  Top-level entry point: runs all 5 Indian-road environments
% (Phase F) through the single shared adaptive planner, computes and
% aggregates Phase G metrics into one summary table, and saves
% figures/CSVs under results/.
%
% Run the individual runStage1_*.m ... runStage5_*.m scripts first to
% build confidence incrementally before running this full demo -- see
% README.txt.

setupProject();

envNames = {'village','intersection','highway','market','cattle'};
allRows = {};

if ~exist('results','dir'), mkdir('results'); end

for e = 1:numel(envNames)
    envName = envNames{e};
    cfg = envConfigs(envName);
    numTrials = cfg.numTrials;

    collisions = 0; completions = 0;
    latSum = 0; validLatRuns = 0;
    smoothSum = 0;
    ttcMin = Inf; anyTTC = false;
    brakeSum = 0; speedSum = 0;

    for trial = 1:numTrials
        [~, metrics] = runScenario(envName, struct('seed', trial, 'showPlot', (trial==1)));
        fprintf('[%s] trial %d: collision=%d completed=%d meanLatency=%.4f s\n', ...
                envName, trial, metrics.collision, metrics.completed, metrics.meanReplanLatency);

        collisions = collisions + metrics.collision;
        completions = completions + metrics.completed;
        if ~isnan(metrics.meanReplanLatency)
            latSum = latSum + metrics.meanReplanLatency;
            validLatRuns = validLatRuns + 1;
        end
        smoothSum = smoothSum + metrics.pathSmoothness;
        if ~isnan(metrics.minTTC)
            ttcMin = min(ttcMin, metrics.minTTC);
            anyTTC = true;
        end
        brakeSum = brakeSum + metrics.emergencyBrakingEvents;
        speedSum = speedSum + metrics.avgSpeed;
    end

    row.scenario = envName;
    row.collisionRatePct = 100 * collisions / numTrials;
    row.completionPct = 100 * completions / numTrials;
    row.meanReplanLatencyMs = 1000 * (latSum / max(validLatRuns,1));
    row.meanPathSmoothness = smoothSum / numTrials;
    if anyTTC
        row.minTTC = ttcMin;
    else
        row.minTTC = NaN;
    end
    row.avgEmergencyBrakingEvents = brakeSum / numTrials;
    row.avgSpeed = speedSum / numTrials;
    allRows{end+1} = row;
end

summaryTable = struct2table([allRows{:}]);
disp(summaryTable);
writetable(summaryTable, fullfile('results','summary_metrics.csv'));
fprintf('\nSaved results/summary_metrics.csv and per-scenario trajectory plots.\n');
end
