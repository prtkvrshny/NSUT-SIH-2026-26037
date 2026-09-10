function [log, metrics] = runScenario(envName, opts)
% runScenario  Run one full closed-loop simulation of a given
% environment using the single shared adaptive planner.
% Orchestrates Phase A-E for one scenario, dispatched by the Phase F
% config in envConfigs.m.
%
%   [log, metrics] = runScenario(envName, opts)
%
% opts (all optional):
%   .dtSim        - fast integration step [s] (default 0.02, 50 Hz)
%   .planPeriod   - planning cycle period [s] (default 0.1, 10 Hz --
%                   Phase E item 17)
%   .sensorMode   - 'groundtruth'|'lidar'|'radar'|'camera'|'fusion'
%                   (default 'fusion'; used by runStage5_sensors.m to
%                   add sensors one at a time)
%   .seed         - RNG seed (default: unseeded / current stream)
%   .showPlot     - true/false, plot at the end (default false)
%   .reflexRadius - independent low-level AEB trigger distance [m]
%                   (default 2.0) -- see NOTES.md: this is a
%                   supplementary safety net beyond the two required
%                   hard-requirement bug fixes, analogous to a real
%                   vehicle's independent last-resort AEB sensor,
%                   checking ground-truth distance every fast tick
%                   rather than waiting for the next 10 Hz planning cycle.

if nargin < 2, opts = struct(); end
dtSim         = getOrDefault(opts,'dtSim',0.02);
planPeriod    = getOrDefault(opts,'planPeriod',0.1);
sensorMode    = getOrDefault(opts,'sensorMode','fusion');
reflexRadius  = getOrDefault(opts,'reflexRadius',2.0);

% *** TOOLBOX ADD-ON opt-ins (see TOOLBOX_INTEGRATION.md) *** -- all
% three default to the exact plain-MATLAB behavior this project
% already had; passing 'toolbox'/true is required to touch anything
% new here.
trackerBackend = getOrDefault(opts,'trackerBackend','plain'); % 'plain'|'toolbox'
lidarBackend   = getOrDefault(opts,'lidarBackend','plain');   % 'plain'|'toolbox'
smoothTypes    = getOrDefault(opts,'smoothTypes',false);      % true to fix camera-type flicker (see sensors/smoothTrackTypes.m)

if isfield(opts,'seed') && ~isempty(opts.seed)
    rand('seed', opts.seed); randn('seed', opts.seed); %#ok<RAND>
end

cfg = envConfigs(envName);
% Straight reference axis; the vehicle and obstacles are free to use the
% full corridor width -- no lane assumption (Stage 4 requirement).
centerline = [0 0; cfg.roadLength 0];

veh = initVehicleState(cfg.egoStart(1), cfg.egoStart(2), cfg.egoStart(3), cfg.egoStart(4));
obstacles = initObstacles(cfg.obstacles);
tracks = initTrackList();
plannerState = [];

planParams = struct('corridorHalfWidth', cfg.corridorHalfWidth, ...
                     'targetSpeed', cfg.targetSpeed);

nSteps = round(cfg.simDuration / dtSim);
stepsPerPlan = max(1, round(planPeriod / dtSim));

log.t = zeros(nSteps,1);
log.vehX = zeros(nSteps,1); log.vehY = zeros(nSteps,1);
log.vehTheta = zeros(nSteps,1); log.vehV = zeros(nSteps,1);
log.mode = cell(nSteps,1);
log.risk = zeros(nSteps,1);
log.planLatency = nan(nSteps,1);
log.ttc = inf(nSteps,1);
log.minObsDist = inf(nSteps,1);
log.delta = zeros(nSteps,1);
log.emergencyEvents = 0;
log.collision = false;
log.reachedGoal = false;

lastMode = '';
ctrl.a = 0; ctrl.delta = 0;
roadInfo.centerline = centerline; roadInfo.halfWidth = cfg.corridorHalfWidth;

lastStep = nSteps;

for step = 1:nSteps
    t = (step-1)*dtSim;

    % --- obstacles move ---
    obstacles = obstacleUpdate(obstacles, dtSim, t, roadInfo);

    % --- perception + planning at the slower planning rate ---
    if mod(step-1, stepsPerPlan) == 0
        isFirstPlan = (step == 1); % used to reset any stateful toolbox backend at the start of THIS run
        tracks = perceive(veh, obstacles, tracks, sensorMode, planPeriod, ...
                           trackerBackend, lidarBackend, smoothTypes, isFirstPlan);
        [ctrl, plannerState, diag] = adaptivePlanner(veh, tracks, centerline, plannerState, planParams);
        log.risk(step) = diag.maxRisk;
        log.planLatency(step) = diag.planLatency;
        log.ttc(step) = diag.worstTTC;
        if strcmp(diag.mode,'emergency_brake') && ~strcmp(lastMode,'emergency_brake')
            log.emergencyEvents = log.emergencyEvents + 1;
        end
        lastMode = diag.mode;
    end

    % --- independent low-level reflex AEB using CURRENT (pre-step)
    %     ground-truth distances -- supplementary safety net, see
    %     NOTES.md ---
    minDistNow = Inf;
    for k = 1:numel(obstacles)
        if ~obstacles(k).active, continue; end
        dd = hypot(obstacles(k).x - veh.x, obstacles(k).y - veh.y);
        minDistNow = min(minDistNow, dd);
    end
    log.minObsDist(step) = minDistNow;
    if minDistNow < reflexRadius
        [ctrl.a, ctrl.delta] = emergencyBraking(veh, 8.0);
    end

    % --- integrate vehicle ---
    veh = bicycleModel(veh, ctrl.a, ctrl.delta, dtSim);

    % --- collision bookkeeping using POST-step ground-truth distances ---
    for k = 1:numel(obstacles)
        if ~obstacles(k).active, continue; end
        ddPost = hypot(obstacles(k).x - veh.x, obstacles(k).y - veh.y);
        if ddPost < 1.2
            log.collision = true;
        end
    end

    log.t(step) = t;
    log.vehX(step) = veh.x; log.vehY(step) = veh.y;
    log.vehTheta(step) = veh.theta; log.vehV(step) = veh.v;
    log.mode{step} = lastMode;
    log.delta(step) = ctrl.delta;

    stopNow = false;
    if log.collision
        stopNow = true;
    elseif veh.x >= cfg.egoGoalX
        log.reachedGoal = true;
        stopNow = true;
    end
    if stopNow
        lastStep = step;
        break;
    end
end

fn = {'t','vehX','vehY','vehTheta','vehV','mode','risk','planLatency','ttc','minObsDist','delta'};
for i = 1:numel(fn)
    log.(fn{i}) = log.(fn{i})(1:lastStep);
end

metrics = computeMetrics(log, dtSim);

if getOrDefault(opts,'showPlot',false)
    plotResults(log, obstacles, cfg, envName);
end
end

function tracks = perceive(veh, obstacles, tracks, sensorMode, dt, ...
                            trackerBackend, lidarBackend, smoothTypes, isFirstPlan)
% *** TOOLBOX ADD-ON dispatch (see TOOLBOX_INTEGRATION.md) *** -- the
% four extra arguments all default-behave exactly as the pre-add-on
% version of this function did when trackerBackend='plain',
% lidarBackend='plain', smoothTypes=false (runScenario.m's opts default
% to exactly this).
if nargin < 6 || isempty(trackerBackend), trackerBackend = 'plain'; end
if nargin < 7 || isempty(lidarBackend),   lidarBackend   = 'plain'; end
if nargin < 8 || isempty(smoothTypes),    smoothTypes    = false;   end
if nargin < 9,                             isFirstPlan    = false;  end

useLidarToolbox = strcmp(lidarBackend, 'toolbox');

switch sensorMode
    case 'groundtruth'
        fused = groundTruthAsFused(veh, obstacles);
    case 'lidar'
        if useLidarToolbox
            L = virtualLidarToolbox(veh, obstacles, struct());
        else
            L = virtualLidar(veh, obstacles, struct());
        end
        fused = lidarOnlyAsFused(L);
    case 'radar'
        R = virtualRadar(veh, obstacles, struct());
        fused = radarOnlyAsFused(R);
    case 'camera'
        % Camera alone gives no range in this model; for this
        % incremental demo mode only, ground-truth position is used
        % with camera-only classification/noise -- see runStage5 notes
        % and NOTES.md for why this is a deliberately limited stand-in.
        C = virtualCamera(veh, obstacles, struct());
        fused = cameraOnlyAsFused(obstacles, C);
    case 'fusion'
        if useLidarToolbox
            L = virtualLidarToolbox(veh, obstacles, struct());
        else
            L = virtualLidar(veh, obstacles, struct());
        end
        R = virtualRadar(veh, obstacles, struct());
        C = virtualCamera(veh, obstacles, struct());
        fused = sensorFusion(veh, L, R, C, struct());
    otherwise
        error('runScenario:badSensorMode','Unknown sensorMode "%s"', sensorMode);
end

if strcmp(trackerBackend, 'toolbox')
    % Graceful fallback, consistent with every other toolbox add-on in
    % this delivery: if Sensor Fusion and Tracking Toolbox isn't
    % available/licensed (as in this sandbox's Octave) or an API
    % mismatch surfaces, fall back to the plain tracker rather than
    % crashing the whole scenario run. Warned once per MATLAB/Octave
    % session, not once per planning cycle.
    try
        tracks = trackObstaclesToolbox(tracks, fused, veh, dt, struct('resetTracker', isFirstPlan));
    catch ME
        persistent warnedTrackerFallback
        if isempty(warnedTrackerFallback)
            warning('runScenario:trackerToolboxUnavailable', ...
                ['trackObstaclesToolbox failed (%s: %s) -- falling back to the ' ...
                 'plain-MATLAB tracker for the rest of this session. See ' ...
                 'TOOLBOX_INTEGRATION.md.'], ME.identifier, ME.message);
            warnedTrackerFallback = true;
        end
        tracks = trackObstacles(tracks, fused, veh, dt, struct());
    end
else
    tracks = trackObstacles(tracks, fused, veh, dt, struct());
end

if smoothTypes
    tracks = smoothTrackTypes(tracks, struct('resetHistory', isFirstPlan));
end
end

function fused = groundTruthAsFused(veh, obstacles)
fused = struct('x',{},'y',{},'bearing',{},'rangeRate',{},'type',{},'confidence',{},'trueId',{});
for k = 1:numel(obstacles)
    o = obstacles(k);
    if ~o.active, continue; end
    f.x = o.x; f.y = o.y;
    f.bearing = atan2(o.y-veh.y, o.x-veh.x) - veh.theta;
    f.rangeRate = NaN; f.type = o.type; f.confidence = 1; f.trueId = o.id;
    fused(end+1) = f;
end
end

function fused = lidarOnlyAsFused(L)
fused = struct('x',{},'y',{},'bearing',{},'rangeRate',{},'type',{},'confidence',{},'trueId',{});
for i = 1:numel(L)
    f.x = L(i).x; f.y = L(i).y; f.bearing = L(i).bearing;
    f.rangeRate = NaN; f.type = 'unknown'; f.confidence = 0; f.trueId = L(i).trueId;
    fused(end+1) = f;
end
end

function fused = radarOnlyAsFused(R)
fused = struct('x',{},'y',{},'bearing',{},'rangeRate',{},'type',{},'confidence',{},'trueId',{});
for i = 1:numel(R)
    f.x = R(i).x; f.y = R(i).y; f.bearing = R(i).bearing;
    f.rangeRate = R(i).rangeRate; f.type = 'unknown'; f.confidence = 0; f.trueId = R(i).trueId;
    fused(end+1) = f;
end
end

function fused = cameraOnlyAsFused(obstacles, C)
fused = struct('x',{},'y',{},'bearing',{},'rangeRate',{},'type',{},'confidence',{},'trueId',{});
for i = 1:numel(C)
    idx = find([obstacles.id] == C(i).trueId, 1);
    if isempty(idx), continue; end
    f.x = obstacles(idx).x; f.y = obstacles(idx).y; f.bearing = C(i).bearing;
    f.rangeRate = NaN; f.type = C(i).type; f.confidence = C(i).confidence; f.trueId = C(i).trueId;
    fused(end+1) = f;
end
end

function v = getOrDefault(s,f,default)
if isfield(s,f) && ~isempty(s.(f)), v = s.(f); else, v = default; end
end
