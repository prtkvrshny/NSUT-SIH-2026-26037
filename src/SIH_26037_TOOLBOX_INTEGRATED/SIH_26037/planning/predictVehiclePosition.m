function pred = predictVehiclePosition(veh, candidateXY, candidateS, speedProfile, horizonTimes)
% predictVehiclePosition  Map a candidate path to predicted (x,y) at a
% set of future times, assuming a (possibly constant) speed profile.
% Implements Phase C item 10.
%
%   pred = predictVehiclePosition(veh, candidateXY, candidateS, speedProfile, horizonTimes)
%
% Inputs:
%   veh          - current vehicle state (used for current speed if
%                  speedProfile is empty)
%   candidateXY  - Mx2 path points (from generateCandidatePaths)
%   candidateS   - Mx1 arc-length values matching candidateXY
%   speedProfile - scalar constant speed, or [] to use veh.v
%   horizonTimes - vector of times [s] to predict at (0 = now)
%
% Output: pred struct array with fields x, y, matching horizonTimes.

if isempty(speedProfile)
    speedProfile = max(veh.v, 0.5); % avoid a zero-speed stall in prediction
end

% *** Bug fix (loop/orbit failure on stale paths) ***: this used to be
% `s0 = candidateS(1)`, i.e. "assume the vehicle is at the START of
% whichever path was passed in". That's true the instant a candidate is
% freshly generated (generateCandidatePaths.m always starts sampling at
% the vehicle's current position) -- but this same function is also
% called every planning cycle, via selectSafePath.m ->
% adaptivePlanner.m's selectSafePathSingle, to re-validate the SINGLE
% PREVIOUSLY-CHOSEN path the vehicle is already partway (or, once it has
% driven the full ~30 m horizon, entirely) past. Blindly restarting the
% prediction from the path's first sample re-evaluated the same
% long-since-passed stretch of road as "safe" forever, no matter how far
% the vehicle had actually travelled -- so the planner never noticed the
% path had run out, kept "following" it, and pure pursuit ended up
% orbiting the path's fixed final waypoint (visible as a closed loop in
% the trajectory plot). Anchoring to the vehicle's ACTUAL current
% position along this path fixes both symptoms: the prediction now
% reflects where the vehicle really is, and once it's at/near the path's
% end, sTarget clamps there too -- which combined with the
% remaining-path-distance check added in planning/adaptivePlanner.m
% forces a timely replan instead of an indefinite stale-path follow.
s0 = frenetUtils('projectOnPath', candidateXY, candidateS, veh.x, veh.y);
n = numel(horizonTimes);
pred(n) = struct('x',0,'y',0);
for i = 1:n
    sTarget = s0 + speedProfile*horizonTimes(i);
    sTarget = max(candidateS(1), min(sTarget, candidateS(end)));
    xy = interp1(candidateS, candidateXY, sTarget, 'linear', 'extrap');
    pred(i).x = xy(1);
    pred(i).y = xy(2);
end
end
