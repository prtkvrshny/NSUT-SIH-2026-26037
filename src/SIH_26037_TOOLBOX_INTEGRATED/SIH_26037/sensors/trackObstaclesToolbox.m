function tracks = trackObstaclesToolbox(tracks, fused, veh, dt, params)
% trackObstaclesToolbox  Sensor Fusion and Tracking Toolbox alternate to
% sensors/trackObstacles.m, using trackerGNN instead of the hand-rolled
% nearest-neighbor tracker.
%
% *** TOOLBOX ADD-ON -- see TOOLBOX_INTEGRATION.md before trusting this ***
% *** REVISED after a real-MATLAB run showed absurd swerving -- read
% "WHY THE VELOCITY IS NOT JUST tt.State(2)/(4) ANYMORE" below before
% touching this again. ***
%
% This is still the single HIGHEST-RISK file in this delivery, and this
% revision is STILL not executable in this sandbox (no Sensor Fusion
% and Tracking Toolbox in this Octave install) -- I cannot run
% trackerGNN even now. What changed is based on reasoning through a
% real failure you reported (a village-run screenshot showing the
% vehicle swerving hard, near the road edge, well before any obstacle
% was close), not on new test evidence from this sandbox. Re-test this
% specific fix before trusting it.
%
%   tracks = trackObstaclesToolbox(tracks, fused, veh, dt, params)
%
% Same call signature, same input/output track struct shape
% (id,x,y,vx,vy,type,confirmed,age,missedFrames) as
% sensors/trackObstacles.m, so every downstream file (planning/*,
% simulation/*) works unchanged regardless of which tracker produced
% the track list -- this is the ONLY reason swapping trackers is safe
% without touching any tested planning code.
%
% WHY THE VELOCITY IS NOT JUST tt.State(2)/(4) ANYMORE
% ------------------------------------------------------
% The original version of this file read vx/vy straight off the
% Kalman filter's state vector every cycle. sensors/initSIHTrackFilter.m
% deliberately biases a BRAND NEW track's velocity toward a
% conservative "closing on the ego" worst case (reimplementing
% hard-requirement bug-fix #2 -- see that file). That part is fine for
% frame 1. The problem is what happens on frames 2, 3, 4...: a Kalman
% filter's velocity states are only ever corrected INDIRECTLY (through
% the position-velocity correlation the constant-velocity motion model
% builds up over successive predict/correct cycles), not immediately
% from a single new position measurement the way a plain finite
% difference is. That means a wrong initial velocity GUESS can keep
% biasing the filter's OWN velocity estimate for several cycles after
% the guess should have been superseded by real data -- exactly long
% enough to explain a sustained, multi-cycle wrong evasive swerve
% rather than a single-frame blip. sensors/trackObstacles.m (the plain
% tracker) doesn't have this problem because it was never a Kalman
% filter to begin with -- it uses the worst-case guess for exactly one
% frame, then a direct finite-difference between the two most recent
% real positions from the second sighting onward, which corrects
% essentially instantly.
%
% The fix: keep trackerGNN for what it's actually good at here --
% detection-to-track ASSOCIATION and ID/lifecycle management (spawn/
% confirm/delete), which is real, useful toolbox capability -- but stop
% trusting its internal Kalman VELOCITY estimate for anything beyond a
% brand-new track's very first cycle. From the second sighting of a
% given TrackID onward, velocity here is recomputed as a direct
% finite-difference between this cycle's and last cycle's POSITION
% (which the Kalman filter does estimate quickly and well, since
% position is directly measured), tracked in a small position-history
% map keyed by TrackID -- exactly mirroring trackObstacles.m's own
% approach, on purpose, since that approach is the one you confirmed
% works.
%
% This finite-difference logic itself IS unit-testable without a real
% tracker (feed it synthetic position histories) and was tested that
% way -- see TOOLBOX_INTEGRATION.md. What remains untested is
% everything upstream of it (whether trackerGNN really associates
% detections the way I assume, whether TrackID stays stable across
% calls the way I assume).
%
% A velocity sanity clamp was also added as a second line of defense:
% regardless of root cause, no track's reported speed can exceed a
% generous physical ceiling (default 20 m/s, see maxSpeedClamp below),
% so a future bug in either the finite-difference path or the
% first-cycle worst-case path can no longer send the planner a
% velocity absurd enough to explain a swerve to the road edge.
%
% Differences from trackObstacles.m you should know about:
%   - `.age` and `.missedFrames` are NOT meaningfully computed here
%     (grep confirms nothing outside trackObstacles.m/initTrackList.m
%     reads these two fields, so they are set to harmless placeholder
%     values: `.age` mirrors trackerGNN's own reported Age when
%     available, `.missedFrames` is always 0, since trackerGNN's own
%     DeletionThreshold -- not an external missedFrames count -- is
%     what removes stale tracks here).
%   - `.type`/`.confidence` are not tracked by the geometric filter at
%     all; they ride along via objectDetection's ObjectAttributes and
%     are read back off each track's most recent associated detection.
%     CONFIDENCE NOTE: I'm less sure ObjectAttributes round-trips onto
%     objectTrack exactly the way this code assumes than I am about
%     the core State/TrackID/IsConfirmed mechanics -- if it doesn't,
%     the practical effect is every track reports type='unknown' (see
%     the try/catch below), which degrades type-based risk weighting
%     back toward the 'unknown' default but does NOT crash the tracker.
%
% *** IMPORTANT -- statefulness across scenario runs ***: trackerGNN
% AND the position-history map below are both persistent, stateful
% state (unlike trackObstacles.m, which only persists a simple ID
% counter). Calling this function again for a NEW scenario run without
% resetting would incorrectly carry over the previous run's tracks and
% position history. Pass params.resetTracker = true on the FIRST call
% of every new runScenario()/animateScenario() invocation
% (simulation/runScenario.m and visualization/animateScenario.m both
% do this for you when trackerBackend='toolbox').

if nargin < 5, params = struct(); end
resetTracker = isfield(params,'resetTracker') && params.resetTracker;
maxSpeedClamp = getFieldOrDefault(params, 'maxSpeedClamp', 20); % m/s, generous ceiling -- see header

persistent tracker
if isempty(tracker) || resetTracker
    tracker = trackerGNN( ...
        'FilterInitializationFcn', @initSIHTrackFilter, ...
        'AssignmentThreshold', 30, ...           % CONFIDENCE: revised from an initial guess of 9 (too tight -- risked failed association/spurious re-spawning) to 30, the value most commonly seen in MathWorks trackerGNN examples for this kind of 2-D position gating. Still a guess, not verified in this sandbox.
        'ConfirmationThreshold', [2 2], ...       % confirmed by the 2nd sighting -- approximates bug-fix #2's ">=2 tracked frames" rule
        'DeletionThreshold', [5 5]);              % replaces trackObstacles.m's maxMissedFrames=5
end

persistent tNow
if isempty(tNow) || resetTracker
    tNow = 0;
end
tNow = tNow + dt;

% Position history for the finite-difference velocity fix (see header).
% containers.Map value: struct('x',...,'y',...,'t',...) from the LAST
% cycle this TrackID was seen.
persistent posHistory
if isempty(posHistory) || resetTracker
    posHistory = containers.Map('KeyType','double','ValueType','any');
end

detections = cell(1, numel(fused));
for i = 1:numel(fused)
    f = fused(i);
    attrs = struct('type', getFieldOrDefault(f,'type','unknown'), ...
                    'confidence', getFieldOrDefault(f,'confidence',0), ...
                    'trueId', getFieldOrDefault(f,'trueId',NaN), ...
                    'egoX', veh.x, 'egoY', veh.y, ...
                    'rangeRate', getFieldOrDefault(f,'rangeRate',NaN));
    measNoise = diag([0.3 0.3]); % approx. sensorFusion.m's LiDAR-primary accuracy
    detections{i} = objectDetection(tNow, [f.x; f.y], ...
        'MeasurementNoise', measNoise, ...
        'ObjectAttributes', attrs);
end

toolboxTracks = tracker(detections, tNow);

n = numel(toolboxTracks);
newTracks = struct('id',{},'x',{},'y',{},'vx',{},'vy',{},'type',{}, ...
                    'confirmed',{},'age',{},'missedFrames',{});
liveIds = zeros(1,n);
for i = 1:n
    tt = toolboxTracks(i);
    t.id = tt.TrackID;
    liveIds(i) = t.id;
    t.x  = tt.State(1);
    t.y  = tt.State(3);

    [t.vx, t.vy] = velocityForTrack(t.id, t.x, t.y, tNow, tt.State(2), tt.State(4), posHistory, maxSpeedClamp);
    posHistory(t.id) = struct('x', t.x, 'y', t.y, 't', tNow);

    try
        t.confirmed = logical(tt.IsConfirmed);
    catch
        t.confirmed = true; % if IsConfirmed isn't exposed, don't silently under-warn
    end
    try
        t.age = double(tt.Age);
    catch
        t.age = NaN; % unused downstream -- see header
    end
    t.missedFrames = 0; % unused downstream -- see header

    t.type = 'unknown';
    try
        a = tt.ObjectAttributes;
        if iscell(a) && ~isempty(a), a = a{1}; end
        if isstruct(a) && isfield(a,'type')
            t.type = a.type;
        end
    catch
        % leave as 'unknown' -- see CONFIDENCE NOTE above
    end

    newTracks(end+1) = t; %#ok<AGROW>
end

% Drop history for ids no longer live, so the map doesn't grow
% unbounded over a long run (same pattern as sensors/smoothTrackTypes.m).
if posHistory.Count > 0
    allKeys = cell2mat(keys(posHistory));
    for k = 1:numel(allKeys)
        if ~ismember(allKeys(k), liveIds)
            remove(posHistory, allKeys(k));
        end
    end
end

tracks = newTracks;
end

function [vx, vy] = velocityForTrack(id, x, y, t, kfVx, kfVy, posHistory, maxSpeedClamp)
% Fast, direct finite-difference velocity from the last two associated
% positions of this TrackID, falling back to the Kalman filter's own
% (worst-case-biased, on a brand new track) velocity estimate ONLY when
% there is no prior position for this id yet. See file header for why.
% Pure plain-MATLAB logic -- unit-tested with synthetic inputs (see
% TOOLBOX_INTEGRATION.md), does not require a real tracker to verify.
if isKey(posHistory, id)
    prev = posHistory(id);
    dt = t - prev.t;
    if dt > 1e-6
        vx = (x - prev.x) / dt;
        vy = (y - prev.y) / dt;
    else
        % Same-timestamp re-entry (shouldn't normally happen -- tNow is
        % strictly increasing every call) -- keep the filter's estimate
        % rather than divide by ~0.
        vx = kfVx; vy = kfVy;
    end
else
    % Brand new track this cycle: no position history yet to
    % difference against, so use the Kalman filter's own estimate,
    % which sensors/initSIHTrackFilter.m deliberately biased toward a
    % conservative worst case for exactly this situation.
    vx = kfVx; vy = kfVy;
end

speed = hypot(vx, vy);
if speed > maxSpeedClamp
    scale = maxSpeedClamp / speed;
    vx = vx * scale;
    vy = vy * scale;
end
end

function v = getFieldOrDefault(s, f, default)
if isfield(s,f) && ~isempty(s.(f)) && ~(isnumeric(s.(f)) && isscalar(s.(f)) && isnan(s.(f)))
    v = s.(f);
else
    v = default;
end
end
