function [collides, tCollision, minDist] = checkFutureCollision(egoPred, egoTimes, obstaclePred)
% checkFutureCollision  Check whether a predicted ego trajectory
% intersects a predicted obstacle trajectory within a safety margin.
% Implements Phase C item 12.
%
%   [collides, tCollision, minDist] = checkFutureCollision(egoPred, egoTimes, obstaclePred)
%
% Inputs:
%   egoPred      - struct array with .x,.y, one per time in egoTimes
%   egoTimes     - times [s] matching egoPred (0 = now)
%   obstaclePred - struct array from predictObstaclePosition, evaluated
%                  at the SAME horizonTimes as egoTimes (caller's
%                  responsibility to keep these aligned)
%
% Output:
%   collides   - true/false
%   tCollision - time of first predicted collision (NaN if none)
%   minDist    - minimum ego-obstacle distance found over the horizon

collides = false;
tCollision = NaN;
minDist = Inf;

n = min(numel(egoPred), numel(obstaclePred));
for i = 1:n
    d = hypot(egoPred(i).x - obstaclePred(i).x, egoPred(i).y - obstaclePred(i).y);
    if d < minDist, minDist = d; end
    safeDist = obstaclePred(i).radius + 1.0; % + ego half-width allowance
    if d < safeDist
        collides = true;
        tCollision = egoTimes(i);
        return;
    end
end
end
