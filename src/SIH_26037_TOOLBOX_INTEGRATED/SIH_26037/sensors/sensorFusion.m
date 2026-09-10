function fused = sensorFusion(veh, lidarDet, radarDet, cameraDet, params)
% sensorFusion  Associate & merge single-frame LiDAR/radar/camera detections.
% Implements Phase B item 6: combining multiple sensors' detections into
% one fused per-object list, before temporal tracking.
%
%   fused = sensorFusion(veh, lidarDet, radarDet, cameraDet, params)
%
% Strategy (deliberately simple & explainable, not a Kalman fusion):
%   1. LiDAR detections are the primary position source (best
%      range/bearing accuracy in this simulated sensor suite).
%   2. For each LiDAR detection, find the nearest radar detection within
%      a bearing gate and copy its rangeRate across.
%   3. For each LiDAR detection, find the nearest camera detection
%      within a bearing gate and copy its type/confidence.
%   4. A radar detection with no matching LiDAR return is still kept
%      (radar-only) -- a sensor "seeing" something the others miss is
%      not silently dropped (relevant to Phase B item 8).
%
% Output: struct array `fused` with fields x, y, bearing, rangeRate
%   (NaN if unknown), type ('unknown' if unclassified), confidence,
%   trueId (debug only).
%
% LIMITATION (see NOTES.md): camera-only bearings with no LiDAR/radar
% match are dropped here, since this simplified camera model has no
% metric range to build an (x,y) fix from bearing alone.

if nargin < 5, params = struct(); end
bearingGate = getOrDefault(params, 'bearingGate', deg2rad(3));

fused = struct('x',{},'y',{},'bearing',{},'rangeRate',{},'type',{},'confidence',{},'trueId',{});
usedRadar = false(1, numel(radarDet));
usedCamera = false(1, numel(cameraDet));

for i = 1:numel(lidarDet)
    L = lidarDet(i);
    f.x = L.x; f.y = L.y; f.bearing = L.bearing;
    f.rangeRate = NaN; f.type = 'unknown'; f.confidence = 0; f.trueId = L.trueId;

    bestJ = 0; bestErr = bearingGate;
    for j = 1:numel(radarDet)
        if usedRadar(j), continue; end
        err = abs(angdiff(L.bearing, radarDet(j).bearing));
        if err < bestErr
            bestErr = err; bestJ = j;
        end
    end
    if bestJ > 0
        f.rangeRate = radarDet(bestJ).rangeRate;
        usedRadar(bestJ) = true;
    end

    bestK = 0; bestErr = bearingGate;
    for k = 1:numel(cameraDet)
        if usedCamera(k), continue; end
        err = abs(angdiff(L.bearing, cameraDet(k).bearing));
        if err < bestErr
            bestErr = err; bestK = k;
        end
    end
    if bestK > 0
        f.type = cameraDet(bestK).type;
        f.confidence = cameraDet(bestK).confidence;
        usedCamera(bestK) = true;
    end

    fused(end+1) = f;
end

% Radar-only detections (no LiDAR match): keep as position+velocity,
% type unknown.
for j = 1:numel(radarDet)
    if usedRadar(j), continue; end
    R = radarDet(j);
    f.x = R.x; f.y = R.y; f.bearing = R.bearing;
    f.rangeRate = R.rangeRate; f.type = 'unknown'; f.confidence = 0; f.trueId = R.trueId;
    fused(end+1) = f;
end
end

function d = angdiff(a,b)
d = atan2(sin(a-b), cos(a-b));
end

function v = getOrDefault(s,f,default)
if isfield(s,f) && ~isempty(s.(f)), v = s.(f); else, v = default; end
end
