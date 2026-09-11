function doReplan = replanDecision(plannerState, currentPathFeasible, riskChanged, pathRunningOut)
% replanDecision  Decide whether to regenerate candidate paths this
% planning tick, vs. continue following the previously-chosen path.
% Implements Phase E items 17-18.
%
%   doReplan = replanDecision(plannerState, currentPathFeasible, riskChanged, pathRunningOut)
%
% The planner itself is invoked every tick at a fixed rate (Phase E item
% 17: 10 Hz, enforced by the timing loop in simulation/runScenario.m --
% this function does not do timing). Within each tick we still avoid
% needlessly regenerating a perfectly good path (path churn hurts
% smoothness), UNLESS:
%   - there is no current path yet, OR
%   - the current path is no longer collision-free against the FULL
%     current track list (bug-fix #1: this input already reflects an
%     all-obstacles check performed by the caller via selectSafePath.m), OR
%   - risk has materially changed since the last cycle (item 18), OR
%   - the vehicle has driven far enough along the current path that it's
%     about to run out (see below).
%
% *** Bug fix (loop/orbit failure once obstacles clear) ***: this 4th
% condition was previously MISSING entirely. Every candidate path only
% extends ~30 m ahead of wherever it was generated
% (generateCandidatePaths.m's horizonDist); nothing here ever asked "is
% the vehicle about to reach the end of what it's following". If risk
% stays flat (e.g. no obstacles within relevantRange -- exactly what
% happens for a long final stretch once the last obstacle has been
% passed/cleared) and the stale path still reads as "feasible" (see the
% predictVehiclePosition.m fix note for why it could get stuck reading
% as feasible far past its real extent), doReplan stayed false
% indefinitely: the SAME ~30 m path kept being "followed" long after the
% vehicle had actually driven past its end, and pure pursuit
% (vehicle/purePursuitControl.m) ended up orbiting that path's fixed
% final waypoint -- a closed loop in the trajectory, most visible in the
% 'cattle' scenario because its obstacle set clears well before its road
% ends, leaving exactly this kind of long uneventful final stretch. A
% path running low on remaining distance is a legitimate reason to
% replan regardless of risk or feasibility, just like the other three.

if isempty(plannerState) || ~isfield(plannerState,'hasPath') || ~plannerState.hasPath
    doReplan = true;
    return;
end
if ~currentPathFeasible
    doReplan = true;
    return;
end
if riskChanged
    doReplan = true;
    return;
end
if nargin >= 4 && pathRunningOut
    doReplan = true;
    return;
end
doReplan = false;
end
