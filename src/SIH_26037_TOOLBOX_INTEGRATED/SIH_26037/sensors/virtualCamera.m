function detections = virtualCamera(veh, obstacles, params)
% virtualCamera  Simulated forward camera: bearing + type classification.
% Implements Phase B item 6, adding a modality with a distinct failure
% mode: no reliable metric range, but semantic type info.
%
%   detections = virtualCamera(veh, obstacles, params)
%
% params: maxRange(40), fov(100 deg), bearingNoiseStd(0.5 deg),
%   misclassProb(0.08), dropoutProb(0.05)
%
% Output struct array fields: bearing, type (string), confidence, trueId
%
% NOTE (see NOTES.md): camera alone gives no metric range in this
% simplified model. sensors/sensorFusion.m uses camera only to attach a
% `type` label to the nearest LiDAR/radar detection by bearing.

if nargin < 3, params = struct(); end
maxRange        = getOrDefault(params,'maxRange',40);
fov             = getOrDefault(params,'fov',deg2rad(100));
bearingNoiseStd = getOrDefault(params,'bearingNoiseStd',deg2rad(0.5));
misclassProb    = getOrDefault(params,'misclassProb',0.08);
dropoutProb     = getOrDefault(params,'dropoutProb',0.05);

types = {'pedestrian','bike','auto','pushcart','cattle'};
detections = struct('bearing',{},'type',{},'confidence',{},'trueId',{});

for k = 1:numel(obstacles)
    obs = obstacles(k);
    if ~obs.active, continue; end
    dx = obs.x - veh.x; dy = obs.y - veh.y;
    r = hypot(dx,dy);
    brg = atan2(dy,dx) - veh.theta;
    brg = atan2(sin(brg), cos(brg));
    if r > maxRange || abs(brg) > fov/2, continue; end
    if rand() < dropoutProb, continue; end

    thisType = obs.type;
    conf = 0.9;
    if rand() < misclassProb
        others = setdiff(types, {obs.type});
        thisType = others{randi(numel(others))};
        conf = 0.5;
    end

    d.bearing = brg + bearingNoiseStd*randn();
    d.type = thisType;
    d.confidence = conf;
    d.trueId = obs.id;
    detections(end+1) = d;
end
end

function v = getOrDefault(s,f,default)
if isfield(s,f) && ~isempty(s.(f)), v = s.(f); else, v = default; end
end
