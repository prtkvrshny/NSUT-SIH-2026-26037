function [chosen, allFeasible, diagnostics] = selectSafePath(veh, candidates, tracks, params)
% selectSafePath  Score every candidate path against EVERY tracked
% obstacle and pick the best feasible one.
% Implements Phase E items 19, 21.
%
% *** Implements Hard-requirement bug fix #1 ("single-threat tracking") ***
% Every candidate is checked against the FULL track list, not a single
% "the threat" obstacle. A candidate is only feasible if it collides
% with NONE of the currently tracked obstacles over the prediction
% horizon -- there is no code path here that evaluates safety against
% just one obstacle and stops.
%
%   [chosen, allFeasible, diagnostics] = selectSafePath(veh, candidates, tracks, params)
%
% This same function is used both for full replanning (a fresh set of
% candidates from generateCandidatePaths.m) and, from
% planning/adaptivePlanner.m, to re-check whether the SINGLE currently-
% followed path is still safe -- both call sites therefore get the
% identical all-obstacles check, so "is my current path still OK" can
% never silently degrade into a single-obstacle check.
%
% Output:
%   chosen      - selected candidate struct (empty if none feasible)
%   allFeasible - logical vector, one per candidate
%   diagnostics - struct array, per candidate: .cost, .collides, .minDist

if nargin < 4, params = struct(); end
horizon        = getOrDefault(params,'horizon',6.0);
dtStep         = getOrDefault(params,'dtStep',0.2);
smoothWeight   = getOrDefault(params,'smoothWeight',0.3);
progressWeight = getOrDefault(params,'progressWeight',0.4);
currentOffset  = getOrDefault(params,'currentOffset',0);

horizonTimes = 0:dtStep:horizon;
allFeasible = false(1, numel(candidates));
diagnostics = struct('cost',{},'collides',{},'minDist',{});

for c = 1:numel(candidates)
    cand = candidates(c);
    egoPred = predictVehiclePosition(veh, cand.xy, cand.s, veh.v, horizonTimes);

    collidesAny = false;
    worstMinDist = Inf;
    for ti = 1:numel(tracks)   % <-- loop over ALL tracks (bug-fix #1)
        obsPred = predictObstaclePosition(tracks(ti), horizonTimes);
        [collidesThis, ~, minDistThis] = checkFutureCollision(egoPred, horizonTimes, obsPred);
        worstMinDist = min(worstMinDist, minDistThis);
        if collidesThis
            collidesAny = true;
            % Deliberately do NOT break here: we still want worstMinDist
            % computed against every obstacle for diagnostics, even
            % though we already know this candidate is infeasible.
        end
    end

    smoothCost = smoothWeight * abs(cand.offset - currentOffset);
    progressCost = progressWeight * abs(cand.offset);
    if collidesAny
        cost = Inf;
    else
        cost = smoothCost + progressCost;
    end

    allFeasible(c) = ~collidesAny;
    diagnostics(c) = struct('cost', cost, 'collides', collidesAny, 'minDist', worstMinDist);
end

feasibleIdx = find(allFeasible);
if isempty(feasibleIdx)
    chosen = [];
    return;
end
costs = [diagnostics(feasibleIdx).cost];
[~, bestLocal] = min(costs);
chosen = candidates(feasibleIdx(bestLocal));
end

function v = getOrDefault(s,f,default)
if isfield(s,f) && ~isempty(s.(f)), v = s.(f); else, v = default; end
end
