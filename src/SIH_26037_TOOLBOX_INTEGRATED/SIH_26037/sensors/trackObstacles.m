function tracks = trackObstacles(tracks, fused, veh, dt, params)
% trackObstacles  Simple nearest-neighbor multi-object tracker.
% Implements Phase B item 7 (object tracking) and item 8 (unknown /
% surprise obstacle appearance).
%
% *** Implements Hard-requirement bug fix #2 ("zero-velocity assumption
% on first detection") ***
% A brand-new track NEVER gets [vx,vy] = [0,0]. It is assigned a
% conservative, type-informed worst-case closing velocity (see
% defaultWorstCaseVelocity below), or the radar-measured closing speed
% if a radar return was fused into this detection on this very frame.
% The track is flagged .confirmed = false until it has been associated
% across >=2 frames and a real finite-difference velocity has been
% computed. Callers (planning/riskScore.m, planning/predictObstaclePosition.m)
% MUST treat unconfirmed tracks as elevated risk, never as "known safe".
%
%   tracks = trackObstacles(tracks, fused, veh, dt, params)
%
% tracks: struct array (id,x,y,vx,vy,type,confirmed,age,missedFrames) --
%         use sensors/initTrackList.m to create the first, empty one.
% fused : this frame's detections, see sensors/sensorFusion.m
% veh   : current ego vehicle state (needed to anchor the worst-case
%         velocity direction for brand-new tracks -- see NOTES.md)
% dt    : time since the previous tracking update [s]
% params: .gateRadius (default 3 m), .maxMissedFrames (default 5)

if nargin < 5, params = struct(); end
gateRadius = getOrDefault(params, 'gateRadius', 3.0);
maxMissed  = getOrDefault(params, 'maxMissedFrames', 5);

persistent nextId
if isempty(nextId), nextId = 1; end

usedDet = false(1, numel(fused));
newTracks = tracks;

% --- Associate existing tracks to detections (nearest neighbor, gated) ---
for i = 1:numel(tracks)
    predX = tracks(i).x + tracks(i).vx*dt;
    predY = tracks(i).y + tracks(i).vy*dt;

    bestJ = 0; bestD = gateRadius;
    for j = 1:numel(fused)
        if usedDet(j), continue; end
        dd = hypot(fused(j).x - predX, fused(j).y - predY);
        if dd < bestD
            bestD = dd; bestJ = j;
        end
    end

    if bestJ > 0
        det = fused(bestJ);
        usedDet(bestJ) = true;

        % Any track reaching this branch already has >=1 real prior
        % detected position (from spawn or an earlier update), so THIS
        % successful re-association is always the 2nd-or-later real
        % detection -- finite-difference velocity is valid starting now.
        % (Earlier draft gated this on tracks(i).age>=1 *before*
        % incrementing, which off-by-one delayed confirmation to the
        % 3rd sighting instead of the 2nd -- caught by
        % /tmp/test_sensors.m during development, fixed here.)
        newVx = (det.x - tracks(i).x) / max(dt, 1e-3);
        newVy = (det.y - tracks(i).y) / max(dt, 1e-3);
        % KNOWN LIMITATION (see NOTES.md): a single-frame finite
        % difference amplifies sensor noise into velocity error by
        % roughly (position noise / dt). At long range this is severe
        % for radar in particular, because its bearing noise translates
        % into several METRES of cross-range position error, which
        % divided by a 0.1 s tracking period can produce double-digit
        % m/s "ghost" velocities for an object that is barely moving.
        % A production tracker would run a Kalman/EKF filter per track
        % to average this down over many frames; this one does not.
        % Two partial mitigations are applied here instead:
        %   1. A slow exponential blend (newWeight=0.3, not 0.5/0.5) so
        %      noise averages down over several frames rather than
        %      being substituted in wholesale every frame.
        %   2. A hard sanity clamp on the resulting speed, generous
        %      enough to cover every configured obstacle speed in this
        %      project (envConfigs.m tops out at 14 m/s for a highway
        %      auto) while still rejecting clearly-implausible spikes.
        newWeight = 0.3;
        blendedVx = (1-newWeight)*tracks(i).vx + newWeight*newVx;
        blendedVy = (1-newWeight)*tracks(i).vy + newWeight*newVy;
        maxTrackSpeed = 18; % m/s, sanity clamp -- see note above
        spd = hypot(blendedVx, blendedVy);
        if spd > maxTrackSpeed
            scale = maxTrackSpeed / spd;
            blendedVx = blendedVx * scale;
            blendedVy = blendedVy * scale;
        end
        newTracks(i).vx = blendedVx;
        newTracks(i).vy = blendedVy;
        newTracks(i).confirmed = true;

        newTracks(i).x = det.x;
        newTracks(i).y = det.y;
        if isfield(det,'type') && ~strcmp(det.type, 'unknown')
            newTracks(i).type = det.type;
        end
        newTracks(i).age = tracks(i).age + 1;
        newTracks(i).missedFrames = 0;
    else
        newTracks(i).missedFrames = tracks(i).missedFrames + 1;
        % Coast on last known velocity while missed -- never zero it.
        newTracks(i).x = tracks(i).x + tracks(i).vx*dt;
        newTracks(i).y = tracks(i).y + tracks(i).vy*dt;
    end
end

% --- Drop stale tracks ---
if ~isempty(newTracks)
    keep = [newTracks.missedFrames] <= maxMissed;
    newTracks = newTracks(keep);
end

% --- Spawn new tracks for unmatched detections (Phase B item 8) ---
for j = 1:numel(fused)
    if usedDet(j), continue; end
    det = fused(j);
    t.id = nextId; nextId = nextId + 1;
    t.x = det.x; t.y = det.y;
    [t.vx, t.vy] = defaultWorstCaseVelocity(det, veh);
    if isfield(det,'type'), t.type = det.type; else, t.type = 'unknown'; end
    t.confirmed = false;
    t.age = 0;
    t.missedFrames = 0;
    newTracks(end+1) = t;
end

tracks = newTracks;
end

function [vx, vy] = defaultWorstCaseVelocity(det, veh)
% Conservative assumed velocity for a brand-new, single-frame detection.
% Bug-fix #2: NEVER default to [0,0]. If radar supplied a rangeRate on
% this very first frame and it indicates closing, trust that (radar
% genuinely measures closing speed in one frame). Otherwise assume the
% object is moving toward the ego vehicle's current position at a
% type-typical worst-case speed -- deliberately pessimistic rather than
% assuming stationary (see NOTES.md).
typeSpeeds = struct('pedestrian',2.0,'bike',6.0,'auto',10.0, ...
                     'pushcart',1.5,'cattle',1.5,'unknown',4.0);
if isfield(det,'type') && isfield(typeSpeeds, det.type)
    worstSpeed = typeSpeeds.(det.type);
else
    worstSpeed = typeSpeeds.unknown;
end

% Direction from the detection toward the ego vehicle (global frame).
losAngle = atan2(veh.y - det.y, veh.x - det.x);

if isfield(det,'rangeRate') && ~isnan(det.rangeRate) && det.rangeRate < 0
    speed = max(-det.rangeRate, worstSpeed*0.5);
else
    speed = worstSpeed;
end

vx = speed*cos(losAngle);
vy = speed*sin(losAngle);
end

function v = getOrDefault(s,f,default)
if isfield(s,f) && ~isempty(s.(f)), v = s.(f); else, v = default; end
end
