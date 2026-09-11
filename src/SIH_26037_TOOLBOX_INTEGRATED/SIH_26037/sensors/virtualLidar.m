function detections = virtualLidar(veh, obstacles, params)
% virtualLidar  Simulated LiDAR: range/bearing point returns.
% Implements Phase B item 5: LiDAR detecting multiple objects.
%
%   detections = virtualLidar(veh, obstacles, params)
%
% params fields (optional, defaults shown):
%   maxRange (60 m), fov (2*pi rad), rangeNoiseStd (0.05 m),
%   bearingNoiseStd (0.3 deg), dropoutProb (0.02)
%
% Output: struct array with fields range, bearing (relative to vehicle
% heading), x, y (global, noisy), trueId (ground-truth id -- for
% debugging/tests only; a real stack would not have this).
%
% LIMITATION (see NOTES.md): no occlusion / line-of-sight modeling --
% every in-range, in-FOV obstacle is "visible" every frame.

if nargin < 3, params = struct(); end
maxRange        = getOrDefault(params, 'maxRange', 60);
fov             = getOrDefault(params, 'fov', 2*pi);
rangeNoiseStd   = getOrDefault(params, 'rangeNoiseStd', 0.05);
bearingNoiseStd = getOrDefault(params, 'bearingNoiseStd', deg2rad(0.3));
dropoutProb     = getOrDefault(params, 'dropoutProb', 0.02);

detections = struct('range', {}, 'bearing', {}, 'x', {}, 'y', {}, 'trueId', {});

for k = 1:numel(obstacles)
    obs = obstacles(k);
    if ~obs.active, continue; end
    dx = obs.x - veh.x;
    dy = obs.y - veh.y;
    r  = hypot(dx, dy);
    brg = atan2(dy, dx) - veh.theta;
    brg = atan2(sin(brg), cos(brg));
    if r > maxRange || abs(brg) > fov/2
        continue;
    end
    if rand() < dropoutProb
        continue;
    end
    rNoisy   = r + rangeNoiseStd*randn();
    brgNoisy = brg + bearingNoiseStd*randn();
    d.range   = rNoisy;
    d.bearing = brgNoisy;
    d.x = veh.x + rNoisy*cos(brgNoisy + veh.theta);
    d.y = veh.y + rNoisy*sin(brgNoisy + veh.theta);
    d.trueId = obs.id;
    detections(end+1) = d;
end
end

function v = getOrDefault(s, f, default)
if isfield(s, f) && ~isempty(s.(f))
    v = s.(f);
else
    v = default;
end
end
