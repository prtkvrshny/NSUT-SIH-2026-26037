function ttc = timeToCollision(veh, track, horizon, dtStep, collisionRadius)
% timeToCollision  Brute-force forward-simulated TTC between ego
% (continuing straight at current heading/speed -- a conservative
% "no evasive action" baseline) and one tracked obstacle (constant
% velocity). Implements Phase C item 11.
%
%   ttc = timeToCollision(veh, track, horizon, dtStep, collisionRadius)
%
% Returns Inf if no collision is predicted within `horizon` seconds.
%
% NOTE: this deliberately assumes the ego's CURRENT motion continues (no
% braking/steering) -- it answers "how much time do I have if nothing
% changes", which is the right question for a risk score. Whether a
% specific CANDIDATE path collides is a separate question, answered by
% planning/checkFutureCollision.m.

if nargin < 4 || isempty(dtStep), dtStep = 0.1; end
if nargin < 5 || isempty(collisionRadius), collisionRadius = 1.5; end

ttc = Inf;
egoX = veh.x; egoY = veh.y;
egoVx = veh.v*cos(veh.theta); egoVy = veh.v*sin(veh.theta);

t = 0;
while t <= horizon
    ex = egoX + egoVx*t;
    ey = egoY + egoVy*t;
    ox = track.x + track.vx*t;
    oy = track.y + track.vy*t;
    if hypot(ex-ox, ey-oy) <= collisionRadius
        ttc = t;
        return;
    end
    t = t + dtStep;
end
end
