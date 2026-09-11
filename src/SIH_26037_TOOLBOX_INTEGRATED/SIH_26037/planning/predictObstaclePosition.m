function pred = predictObstaclePosition(track, horizonTimes)
% predictObstaclePosition  Constant-velocity prediction with growing
% positional uncertainty, per tracked obstacle.
% Implements Phase C item 9.
%
%   pred = predictObstaclePosition(track, horizonTimes)
%
% Inputs:
%   track        - single track struct (x,y,vx,vy,type,confirmed,age)
%   horizonTimes - scalar or vector of future times [s] (0 = now)
%
% Output: pred struct array (same size as horizonTimes) with fields
%   x, y, radius (positional uncertainty radius, growing with time and
%   the object's type-based "unpredictability").
%
% Type-dependent unpredictability models how much a real trajectory can
% diverge from constant-velocity extrapolation: pedestrians/cattle can
% change direction abruptly; autos/bikes are more committed to their
% current heading over short horizons (Phase D item 15 tie-in).
%
% *** Bug-fix #2 interaction ***: an UNCONFIRMED track's velocity is a
% worst-case *guess*, not a measurement, so its uncertainty grows faster
% until the track is confirmed (see sensors/trackObstacles.m).

unpredictability = struct('pedestrian', 1.4, 'bike', 0.8, 'auto', 0.5, ...
                           'pushcart', 0.6, 'cattle', 1.6, 'unknown', 1.5);
if isfield(unpredictability, track.type)
    k = unpredictability.(track.type);
else
    k = unpredictability.unknown;
end

if ~track.confirmed
    k = k * 1.5;
end

% baseRadius: nominal object footprint allowance.
% growthRate: metres of EXTRA uncertainty per second at k=1, tuned so a
% confirmed, average-unpredictability object reaches a still-physically-
% plausible ~1.5 m of lateral spread by the end of a 6 s horizon.
% maxRadius: hard cap. Without one, a long horizon on an unconfirmed,
% high-unpredictability track (e.g. cattle, k up to 1.6*1.5=2.4) grows
% without bound and can exceed the width of the entire road corridor,
% which makes every candidate path look "blocked" regardless of where
% the real obstacle is -- this happened during desk/Octave testing (see
% NOTES.md) and is exactly the failure this cap prevents.
baseRadius = 0.5;
growthRate = 0.15;
maxRadius = 4.0;

n = numel(horizonTimes);
pred(n) = struct('x',0,'y',0,'radius',0);
for i = 1:n
    t = horizonTimes(i);
    pred(i).x = track.x + track.vx*t;
    pred(i).y = track.y + track.vy*t;
    pred(i).radius = min(baseRadius + growthRate*k*t, maxRadius);
end
end
