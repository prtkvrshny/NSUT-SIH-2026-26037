function [ctrl, plannerState, diag] = adaptivePlanner(veh, tracks, centerline, plannerState, params)
% adaptivePlanner  Top-level per-cycle adaptive path planner.
% Implements Phase E items 17,18,21 as the orchestrator; calls into
% traversabilityMap (D13-14), riskScore (D15-16), generateCandidatePaths
% / selectSafePath (E19), emergencyBraking (E20).
%
% *** This is the primary site of Hard-requirement bug fix #1 ***: the
% "is it still safe to keep/hold this path" check below (via
% selectSafePathSingle -> selectSafePath) re-evaluates EVERY tracked
% obstacle every cycle, never just the single obstacle that originally
% triggered a stop or a lane change. There is no per-obstacle "resolved"
% flag anywhere in this file that would let a second, unrelated obstacle
% go unchecked while the vehicle is held on an old decision.
%
%   [ctrl, plannerState, diag] = adaptivePlanner(veh, tracks, centerline, plannerState, params)
%
% plannerState (persisted across calls by the caller):
%   .hasPath, .currentPath (candidate struct), .lastRisk (scalar)
%
% Output:
%   ctrl.a, ctrl.delta - control commands for this tick
%   plannerState       - updated for next tick
%   diag               - struct with risk map, chosen path, mode, timing

if nargin < 5, params = struct(); end
if isempty(plannerState)
    plannerState = struct('hasPath', false, 'currentPath', [], 'lastRisk', 0);
end

tPlanStart = tic; % Phase G item 29: replanning latency, real wall clock

% --- Relevance filter -----------------------------------------------
% Only tracks within `relevantRange` of the ego are used for LOCAL path
% safety decisions below (default 60 m: candidate paths only extend
% ~30 m ahead, and even a fast auto at the project's configured max of
% ~14 m/s cannot cross 60 m within the horizons used here). This is a
% legitimate planning design choice (don't let something 140 m away
% dictate a swerve happening 10 m ahead) that ALSO shields the planner
% from a real sensor limitation documented in sensors/trackObstacles.m:
% long-range radar detections carry several metres of bearing-driven
% position noise, which a single-frame finite-difference can turn into
% implausible "ghost" velocities. Since planning re-runs at 10 Hz, a
% genuinely fast-closing object re-enters this 60 m zone with several
% planning cycles to spare before it could physically reach the ego at
% any speed configured in this project -- see NOTES.md for the honest
% limits of this mitigation (it does not fix the noise, it bounds its
% blast radius).
relevantRange = getOrDefault(params, 'relevantRange', 60);
relevantTracks = tracks(arrayfun(@(tr) hypot(tr.x-veh.x, tr.y-veh.y) <= relevantRange, tracks));

% --- Prediction, TTC, risk (Phase C & D) -- loops over ALL relevant tracks ---
maxRisk = 0;
worstTTC = Inf;
for i = 1:numel(relevantTracks)
    t_ttc = timeToCollision(veh, relevantTracks(i), 6.0, 0.1, 1.5);
    worstTTC = min(worstTTC, t_ttc);
    r = riskScore(veh, relevantTracks(i), t_ttc, params);
    maxRisk = max(maxRisk, r);
end

% *** TOOLBOX ADD-ON dispatch (see TOOLBOX_INTEGRATION.md) *** --
% default 'plain' calls the exact same line as before this add-on;
% only an explicit params.traversabilityBackend='toolbox' opts into
% the Navigation Toolbox occupancyMap wrapper (which reuses this same
% plain classification internally and can only ever add a
% visualization object on top of it, never change it -- see
% planning/traversabilityMapToolbox.m's header).
travBackend = getOrDefault(params, 'traversabilityBackend', 'plain');
if strcmp(travBackend, 'toolbox')
    travMap = traversabilityMapToolbox(veh, centerline, relevantTracks, params);
else
    travMap = traversabilityMap(veh, centerline, relevantTracks, params);
end

riskChangeThresh = getOrDefault(params,'riskChangeThresh',0.15);
riskChanged = abs(maxRisk - plannerState.lastRisk) > riskChangeThresh;

% --- Is the CURRENT path (if any) still safe against ALL tracks? ---
% (bug-fix #1: identical all-obstacles check as full replanning, via the
% same selectSafePath.m call, applied to a 1-candidate list)
currentPathFeasible = false;
if plannerState.hasPath
    currentPathFeasible = selectSafePathSingle(veh, plannerState.currentPath, relevantTracks, params);
end

% --- Is the CURRENT path about to run out? ---
% *** Bug fix (loop/orbit failure once obstacles clear) ***: every
% candidate path only extends ~30 m past wherever it was generated
% (generateCandidatePaths.m's horizonDist). Nothing previously tracked
% how much of that distance the vehicle had actually already driven, so
% a path could keep being "followed" long after the vehicle had passed
% its end -- see the detailed comments in replanDecision.m and
% predictVehiclePosition.m. `remainingPathDist` here uses the vehicle's
% ACTUAL projected position along the currently-followed path (not the
% path's original start), so it correctly shrinks as the vehicle drives,
% even though plannerState.currentPath.xy/.s themselves are fixed at
% whatever they were when that path was chosen.
remainingPathDist = Inf;
if plannerState.hasPath && ~isempty(plannerState.currentPath)
    curPath = plannerState.currentPath;
    sNow = frenetUtils('projectOnPath', curPath.xy, curPath.s, veh.x, veh.y);
    remainingPathDist = curPath.s(end) - sNow;
end
pathReplanMargin = getOrDefault(params, 'pathReplanMargin', 10);
pathRunningOut = remainingPathDist < pathReplanMargin;

doReplan = replanDecision(plannerState, currentPathFeasible, riskChanged, pathRunningOut);

mode = 'follow';
chosenPath = plannerState.currentPath;

if doReplan
    % *** Adaptive transition distance ***: shrink the lateral-maneuver
    % distance toward the nearest relevant obstacle so candidates
    % actually complete their swerve before reaching it, instead of
    % always spreading the maneuver over the full, fixed horizonDist
    % (which left every candidate barely started on close obstacles --
    % see planning/generateCandidatePaths.m and NOTES.md for how this
    % was found). Floor/ceiling keep it physically sane: never demand a
    % swerve tighter than minTransition, never stretch it needlessly far
    % when nothing is close.
    minTransition = getOrDefault(params,'minTransitionDist',5);
    maxTransition = getOrDefault(params,'maxTransitionDist',15);
    safetyMargin  = getOrDefault(params,'transitionSafetyMargin',3);
    nearestDist = Inf;
    for i = 1:numel(relevantTracks)
        nearestDist = min(nearestDist, hypot(relevantTracks(i).x-veh.x, relevantTracks(i).y-veh.y));
    end
    if isinf(nearestDist)
        transitionDist = maxTransition;
    else
        transitionDist = max(minTransition, min(maxTransition, nearestDist - safetyMargin));
    end
    candParams = params;
    candParams.transitionDist = transitionDist;

    candidates = generateCandidatePaths(veh, centerline, candParams);
    offsetParams = params;
    if plannerState.hasPath
        offsetParams.currentOffset = plannerState.currentPath.offset;
    else
        [~, d0] = frenetUtils('cart2frenet', centerline, veh.x, veh.y);
        offsetParams.currentOffset = d0;
    end
    [chosenNew, ~, ~] = selectSafePath(veh, candidates, relevantTracks, offsetParams);

    if isempty(chosenNew)
        mode = 'emergency_brake';
        chosenPath = [];
    else
        chosenPath = chosenNew;
        mode = 'replanned';
    end
end

if isempty(chosenPath)
    mode = 'emergency_brake';
end

if strcmp(mode, 'emergency_brake')
    [a, delta] = emergencyBraking(veh, getOrDefault(params,'maxDecel',7.0));
    plannerState.hasPath = false;
    plannerState.currentPath = [];
else
    targetSpeed = getOrDefault(params,'targetSpeed',8.0);
    [a, delta, ~] = purePursuitControl(veh, chosenPath.xy, targetSpeed, getOrDefault(params,'lookahead',6));
    plannerState.hasPath = true;
    plannerState.currentPath = chosenPath;
end

plannerState.lastRisk = maxRisk;

ctrl.a = a;
ctrl.delta = delta;

diag.mode = mode;
diag.maxRisk = maxRisk;
diag.worstTTC = worstTTC;
diag.travMap = travMap;
diag.chosenPath = chosenPath;
diag.planLatency = toc(tPlanStart); % Phase G item 29
diag.numTracks = numel(tracks);
diag.numRelevantTracks = numel(relevantTracks);
diag.remainingPathDist = remainingPathDist; % debugging/plots: see pathRunningOut fix above
end

function feasible = selectSafePathSingle(veh, path, tracks, params)
% Helper: check ONE existing path (not a fresh candidate set) against
% every tracked obstacle, via the exact same all-obstacles logic used
% for full replanning (bug-fix #1 consistency between the two call sites).
if isempty(path)
    feasible = false; return;
end
[~, feas, ~] = selectSafePath(veh, path, tracks, params);
feasible = any(feas);
end

function v = getOrDefault(s,f,default)
if isfield(s,f) && ~isempty(s.(f)), v = s.(f); else, v = default; end
end
