function detections = virtualRadar(veh, obstacles, params)
% virtualRadar  Simulated radar: range/bearing/range-rate returns.
% Implements Phase B items 5-6, with a modality tuned for velocity.
%
%   detections = virtualRadar(veh, obstacles, params)
%
% params: maxRange(120), fov(140 deg), rangeNoiseStd(0.2 m),
%   bearingNoiseStd(1.5 deg), rangeRateNoiseStd(0.1 m/s), dropoutProb(0.03)
%
% Output struct array fields: range, bearing, rangeRate, x, y, trueId
%
% rangeRate is closing speed along the line of sight (negative =
% approaching), from ground-truth relative velocity + noise. This is
% what lets a track's velocity be estimated from a SINGLE frame via
% radar, unlike LiDAR/camera which need >=2 frames of finite-differencing.
% See sensors/trackObstacles.m and NOTES.md.

if nargin < 3, params = struct(); end
maxRange          = getOrDefault(params,'maxRange',120);
fov               = getOrDefault(params,'fov',deg2rad(140));
rangeNoiseStd     = getOrDefault(params,'rangeNoiseStd',0.2);
bearingNoiseStd   = getOrDefault(params,'bearingNoiseStd',deg2rad(1.5));
rangeRateNoiseStd = getOrDefault(params,'rangeRateNoiseStd',0.1);
dropoutProb       = getOrDefault(params,'dropoutProb',0.03);

detections = struct('range',{},'bearing',{},'rangeRate',{},'x',{},'y',{},'trueId',{});

egoVx = veh.v*cos(veh.theta);
egoVy = veh.v*sin(veh.theta);

for k = 1:numel(obstacles)
    obs = obstacles(k);
    if ~obs.active, continue; end
    dx = obs.x - veh.x; dy = obs.y - veh.y;
    r = hypot(dx,dy);
    brg = atan2(dy,dx) - veh.theta;
    brg = atan2(sin(brg), cos(brg));
    if r > maxRange || abs(brg) > fov/2, continue; end
    if rand() < dropoutProb, continue; end

    losX = dx / max(r,1e-6); losY = dy / max(r,1e-6);
    relVx = obs.vx - egoVx; relVy = obs.vy - egoVy;
    trueRangeRate = relVx*losX + relVy*losY;

    d.range = r + rangeNoiseStd*randn();
    d.bearing = brg + bearingNoiseStd*randn();
    d.rangeRate = trueRangeRate + rangeRateNoiseStd*randn();
    d.x = veh.x + d.range*cos(d.bearing+veh.theta);
    d.y = veh.y + d.range*sin(d.bearing+veh.theta);
    d.trueId = obs.id;
    detections(end+1) = d;
end
end

function v = getOrDefault(s,f,default)
if isfield(s,f) && ~isempty(s.(f)), v = s.(f); else, v = default; end
end
