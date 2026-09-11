function [detections, usedFallback] = virtualLidarToolbox(veh, obstacles, params)
% virtualLidarToolbox  Lidar Toolbox alternate to sensors/virtualLidar.m.
%
% *** TOOLBOX ADD-ON -- see TOOLBOX_INTEGRATION.md ***
% sensors/virtualLidar.m returns exactly one noisy point per obstacle,
% which sidesteps the actual job a real LiDAR perception stack does:
% turn a raw multi-point return into per-object detections. This
% version instead scatters SEVERAL raw return points across each
% obstacle's physical footprint (from vehicle/obstacleFootprints.m),
% pools every obstacle's points into one Lidar Toolbox `pointCloud`,
% and clusters them back into per-object detections with `pcsegdist`
% (Euclidean/DBSCAN-style clustering) -- this is the genuinely new
% capability the toolbox adds here, not just a different function name
% for the same computation.
%
%   detections = virtualLidarToolbox(veh, obstacles, params)
%
% params: same fields as virtualLidar.m (maxRange, fov, rangeNoiseStd,
%   bearingNoiseStd, dropoutProb), PLUS:
%   .pointsPerObstacle (default 8)   - raw returns scattered per visible obstacle
%   .clusterDistance   (default 2.0 m) - pcsegdist/fallback grouping distance
%
% Output: SAME struct shape as virtualLidar.m (range, bearing, x, y,
% trueId), one entry per detected CLUSTER (normally one per visible
% obstacle, unless two obstacles are close enough to merge into one
% cluster, or one obstacle's own points fail to all chain together --
% both realistic LiDAR failure modes the single-point version cannot
% represent at all).
% Second output `usedFallback` (optional, backward compatible -- old
% callers capturing only `detections` are unaffected): true if
% `pcsegdist` was unavailable/errored THIS CALL and the plain-MATLAB
% fallback clustering ran instead. Callers that display which backend
% is active (e.g. visualization/animateScenario.m) use this to avoid
% claiming "Lidar Toolbox" is running when it silently wasn't.
%
% TUNING TRADEOFF, QUANTIFIED (Octave-tested via the fallback
% clustering path, 200 random trials per case -- see
% TOOLBOX_INTEGRATION.md): with the defaults above, a single obstacle's
% own points fail to all cluster together (false fragmentation) in
% roughly 6% of trials (auto/pedestrian/cattle at 20/40/55 m); two
% DIFFERENT obstacles 5 m apart (as in the `cattle` environment) are
% incorrectly merged into one detection in roughly 3% of trials.
% Smaller clusterDistance reduces false merging but increases false
% fragmentation of large obstacles (e.g. `auto`, 3.2 m long) and vice
% versa -- there is no setting that eliminates both failure modes
% simultaneously with only 8 scattered points; more points per
% obstacle would improve both but cost more per-cycle compute.
%
% CONFIDENCE: the point-scatter generation and the detection-building
% math (centroid -> range/bearing, nearest-ground-truth-id for
% debugging) are plain MATLAB and ARE Octave-tested below -- see
% TOOLBOX_INTEGRATION.md. `pcsegdist`/`pointCloud` themselves are real,
% stable, long-standing Lidar/Computer Vision Toolbox functions I'm
% fairly confident in the call shape of, but Lidar Toolbox is not
% available in this sandbox's Octave, so that ONE call could not be
% executed. A plain-MATLAB fallback clustering function (below) is used
% automatically if pcsegdist is unavailable/errors, which is what makes
% the rest of this file testable at all, and is also a safety net for a
% MATLAB seat that has Lidar Toolbox installed but somehow still hits
% an API surprise -- this file degrades to "still works, just with
% simpler clustering" rather than crashing.

if nargin < 3, params = struct(); end
maxRange        = getOrDefault(params, 'maxRange', 60);
fov             = getOrDefault(params, 'fov', 2*pi);
rangeNoiseStd   = getOrDefault(params, 'rangeNoiseStd', 0.05);
bearingNoiseStd = getOrDefault(params, 'bearingNoiseStd', deg2rad(0.3));
dropoutProb     = getOrDefault(params, 'dropoutProb', 0.02);
pointsPerObs    = getOrDefault(params, 'pointsPerObstacle', 8);
clusterDistance = getOrDefault(params, 'clusterDistance', 2.0);

allPts = zeros(0,3);
allTrueId = zeros(0,1);

for k = 1:numel(obstacles)
    obs = obstacles(k);
    if ~obs.active, continue; end
    dx = obs.x - veh.x; dy = obs.y - veh.y;
    r = hypot(dx,dy);
    brg = atan2(dy,dx) - veh.theta;
    brg = atan2(sin(brg), cos(brg));
    if r > maxRange || abs(brg) > fov/2, continue; end
    if rand() < dropoutProb, continue; end

    dims = obstacleFootprints(obs.type);
    hl = dims.Length/2; hw = dims.Width/2;

    for p = 1:pointsPerObs
        % Scatter a raw return uniformly over the obstacle's footprint
        % (in its own body frame), then rotate into the global frame.
        localX = (2*rand()-1)*hl;
        localY = (2*rand()-1)*hw;
        gx = obs.x + localX*cos(obs.theta) - localY*sin(obs.theta);
        gy = obs.y + localX*sin(obs.theta) + localY*cos(obs.theta);

        rr = hypot(gx-veh.x, gy-veh.y) + rangeNoiseStd*randn();
        bb = atan2(gy-veh.y, gx-veh.x) - veh.theta + bearingNoiseStd*randn();
        px = veh.x + rr*cos(bb+veh.theta);
        py = veh.y + rr*sin(bb+veh.theta);

        allPts(end+1,:) = [px py 0]; %#ok<AGROW>
        allTrueId(end+1,1) = obs.id; %#ok<AGROW>
    end
end

detections = struct('range', {}, 'bearing', {}, 'x', {}, 'y', {}, 'trueId', {});
usedFallback = false;
if isempty(allPts)
    return;
end

[labels, usedFallback] = clusterPoints(allPts, clusterDistance);

numClusters = max(labels);
for c = 1:numClusters
    idx = (labels == c);
    if ~any(idx), continue; end
    cx = mean(allPts(idx,1));
    cy = mean(allPts(idx,2));

    r  = hypot(cx-veh.x, cy-veh.y);
    brg = atan2(cy-veh.y, cx-veh.x) - veh.theta;
    brg = atan2(sin(brg), cos(brg));

    % Debug-only trueId: majority vote among this cluster's raw points.
    idsInCluster = allTrueId(idx);
    trueId = mode(idsInCluster);

    d.range = r; d.bearing = brg; d.x = cx; d.y = cy; d.trueId = trueId;
    detections(end+1) = d; %#ok<AGROW>
end
end

function [labels, usedFallback] = clusterPoints(pts, minDistance)
% Cluster 3xN-style [x y z] points into groups no farther than
% minDistance apart (chained), preferring Lidar Toolbox's real
% `pcsegdist` and falling back to a plain-MATLAB connected-components
% clustering (identical grouping rule: Euclidean chain-linkage) if the
% toolbox isn't available -- see file header for why this matters for
% testability.
try
    ptCloud = pointCloud(pts);
    labels = pcsegdist(ptCloud, minDistance);
    usedFallback = false;
catch
    labels = simpleChainCluster(pts(:,1:2), minDistance);
    usedFallback = true;
end
end

function labels = simpleChainCluster(xy, minDistance)
% Plain-MATLAB Euclidean chain-linkage clustering (equivalent grouping
% rule to pcsegdist's default): two points in the same cluster iff
% connected by a chain of points each within minDistance of the next.
% O(n^2) -- fine for this project's small per-cycle point counts.
n = size(xy,1);
labels = zeros(n,1);
nextLabel = 0;
for i = 1:n
    if labels(i) ~= 0, continue; end
    nextLabel = nextLabel + 1;
    queue = i;
    labels(i) = nextLabel;
    while ~isempty(queue)
        cur = queue(1); queue(1) = [];
        d = hypot(xy(:,1)-xy(cur,1), xy(:,2)-xy(cur,2));
        neighbors = find(d <= minDistance & labels==0);
        labels(neighbors) = nextLabel;
        queue = [queue; neighbors]; %#ok<AGROW>
    end
end
end

function v = getOrDefault(s, f, default)
if isfield(s, f) && ~isempty(s.(f))
    v = s.(f);
else
    v = default;
end
end
