function [a, delta, info] = purePursuitControl(veh, pathXY, targetSpeed, lookahead)
% purePursuitControl  Pure-pursuit path follower + P speed controller.
% Infrastructure used across every stage for path following (not itself
% a numbered checklist item).
%
%   [a, delta, info] = purePursuitControl(veh, pathXY, targetSpeed, lookahead)
%
% Inputs:
%   veh         - vehicle state struct
%   pathXY      - Nx2 array of [x y] waypoints, in travel order
%   targetSpeed - desired speed [m/s]
%   lookahead   - lookahead distance [m] (default 5)
%
% Outputs:
%   a, delta - acceleration & steering commands
%   info     - struct with .lookaheadPoint, .idx (debugging/plots)

if nargin < 4 || isempty(lookahead)
    lookahead = 5;
end

info = struct('lookaheadPoint', [veh.x veh.y], 'idx', 1);

if isempty(pathXY) || size(pathXY,1) < 1
    a = -3.0;
    delta = 0;
    return;
end

d = hypot(pathXY(:,1) - veh.x, pathXY(:,2) - veh.y);
[~, nearestIdx] = min(d);

idx = nearestIdx;
cumDist = 0;
while idx < size(pathXY,1) && cumDist < lookahead
    cumDist = cumDist + hypot(pathXY(idx+1,1) - pathXY(idx,1), ...
                               pathXY(idx+1,2) - pathXY(idx,2));
    idx = idx + 1;
end
targetPt = pathXY(idx,:);
info.lookaheadPoint = targetPt;
info.idx = idx;

dx = targetPt(1) - veh.x;
dy = targetPt(2) - veh.y;
alpha = atan2(dy, dx) - veh.theta;
alpha = atan2(sin(alpha), cos(alpha));
Ld = max(hypot(dx, dy), 0.1);
delta = atan2(2*veh.L*sin(alpha), Ld);

kP = 1.0;
a = kP * (targetSpeed - veh.v);
a = max(min(a, 3.0), -6.0);
end
