function animateScenario(envName, opts)
% animateScenario  Live, dashboard-style animated visualization of one
% scenario run -- a RoadRunner-substitute for demos/pitches.
%
% *** ADDITIVE ONLY (core loop) ***: this file does not modify, and was
% not required to modify, any file in vehicle/, sensors/, planning/, or
% simulation/. It calls the exact same public functions those tested
% modules already expose (envConfigs, initObstacles, obstacleUpdate,
% virtualLidar/Radar/Camera, sensorFusion, trackObstacles,
% adaptivePlanner, bicycleModel) through its own lightweight loop, so
% what you see animated is the real planner behavior, not a separate
% mock. Nothing about the tested simulation/planning logic changes by
% using this.
%
% *** UPDATED post-delivery to also optionally exercise the toolbox
% add-ons from TOOLBOX_INTEGRATION.md (trackerBackend/lidarBackend/
% smoothTypes below) ***, so this animation can actually be used to
% DEMONSTRATE them running, not just describe them. All three still
% default to the exact plain-MATLAB behavior above -- nothing changes
% unless you pass them explicitly. When any is active, the on-screen
% title shows exactly which backend(s) are running, so a judge watching
% the screen (or a screenshot/recording of it) can see it's really
% happening, not just hear it described.
%
% This is offered as a fallback if RoadRunner integration doesn't come
% together in time -- see RoadRunner_Integration_Guide.md for the real
% 3D option. This is a 2D top-down MATLAB figure, not a 3D scene; it
% will not look like RoadRunner, but it does show the planner actually
% making decisions over time, which a static plot cannot.
%
%   animateScenario('village')
%   animateScenario('village', struct('saveVideo', true))
%
% opts (all optional):
%   .seed           - RNG seed (default 1)
%   .sensorMode     - 'groundtruth'|'lidar'|'radar'|'camera'|'fusion'
%                     (default 'fusion' -- the real pipeline used
%                     elsewhere in this project, INCLUDING its known
%                     long-range sensor-noise limitations documented in
%                     NOTES.md. If a glitch-free demo reel matters more
%                     to you than showing the raw sensor pipeline, pass
%                     'groundtruth' instead -- it skips simulated
%                     sensor noise entirely and only shows true
%                     obstacle positions.)
%   .dtSim          - fast integration step [s] (default 0.02)
%   .planPeriod     - planning cycle period [s] (default 0.1)
%   .playbackSpeed  - real-time multiplier for the LIVE on-screen replay
%                     only (default 1). Does not affect saved video timing.
%   .saveVideo      - true/false (default false). If true, writes an
%                     .mp4 to results/animations/<envName>.mp4 via
%                     MATLAB's VideoWriter (standard MATLAB; NOT
%                     available in the GNU Octave environment this
%                     project was smoke-tested in -- see NOTES.md).
%   .showLive       - true/false (default true). Set false with
%                     saveVideo true to render straight to a video file
%                     without a screen window (faster for batch export).
%   .predictHorizon - how far ahead [s] to draw the predicted-obstacle
%                     marker (default 2.0)
%   .goalRadius     - cosmetic "goal radius" circle drawn around the
%                     goal point [m] (default 3)
%   .renderDecimation - render every Nth fast simulation step (default
%                     1 = every step, smoothest but slowest to render)
%   .trackerBackend   - 'plain' (default) | 'toolbox' -- 'toolbox' uses
%                     Sensor Fusion and Tracking Toolbox's trackerGNN
%                     (sensors/trackObstaclesToolbox.m) instead of the
%                     plain tracker. Falls back to 'plain' automatically
%                     (with a one-time warning) if the toolbox isn't
%                     available -- see TOOLBOX_INTEGRATION.md for how
%                     much this specific add-on has been tested (short
%                     answer: less than everything else -- try it once
%                     before a live demo).
%   .lidarBackend     - 'plain' (default) | 'toolbox' -- 'toolbox' uses
%                     Lidar Toolbox point-cloud clustering
%                     (sensors/virtualLidarToolbox.m) instead of one
%                     hand-placed point per obstacle. Only affects
%                     sensorMode='lidar' or 'fusion'.
%   .smoothTypes      - true|false (default false) -- true fixes the
%                     documented camera-classification "flicker" via
%                     sensors/smoothTrackTypes.m (majority vote over
%                     recent frames). Only visibly matters for
%                     sensorMode='camera' or 'fusion'.
% All three above default to this project's original plain-MATLAB
% behavior; see TOOLBOX_INTEGRATION.md for what each actually changes
% and exactly how confident to be in each one.

if nargin < 2, opts = struct(); end
seed            = getOr(opts,'seed',1);
sensorMode      = getOr(opts,'sensorMode','fusion');
dtSim           = getOr(opts,'dtSim',0.02);
planPeriod      = getOr(opts,'planPeriod',0.1);
playbackSpeed   = getOr(opts,'playbackSpeed',1);
saveVideo       = getOr(opts,'saveVideo',false);
showLive        = getOr(opts,'showLive',true);
predictHorizon  = getOr(opts,'predictHorizon',2.0);
goalRadius      = getOr(opts,'goalRadius',3);
renderDecimation = getOr(opts,'renderDecimation',1);
trackerBackend  = getOr(opts,'trackerBackend','plain');   % 'plain'|'toolbox' -- see TOOLBOX_INTEGRATION.md
lidarBackend    = getOr(opts,'lidarBackend','plain');     % 'plain'|'toolbox'
smoothTypes     = getOr(opts,'smoothTypes',false);        % true fixes camera-type flicker

% On-screen label so a judge watching the animation (or a recording of
% it) can see exactly which backend(s) are active, rather than taking
% your word for it.
backendBits = {};
if strcmp(trackerBackend,'toolbox'), backendBits{end+1} = 'Tracker=SensorFusionTbx'; end
if strcmp(lidarBackend,'toolbox'),   backendBits{end+1} = 'LiDAR=LidarTbx'; end
if smoothTypes,                      backendBits{end+1} = 'TypeSmoothing=ON'; end
if isempty(backendBits)
    backendLabel = 'Backends: plain MATLAB (baseline)';
else
    backendLabel = ['Backends: ' strjoin(backendBits, ', ')];
end
% These two turn true the moment a toolbox call actually fails and this
% animation silently falls back -- backendLabel is recomputed from
% these on every planning cycle below, so the ON-SCREEN label always
% reflects what is REALLY running this instant, not just what was
% requested. This matters specifically because a live demo is exactly
% the situation where quietly showing a label that stopped being true
% would be the worst time to find out.
trackerActuallyFellBack = false;
lidarActuallyFellBack = false;

rand('seed', seed); randn('seed', seed); %#ok<RAND>

cfg = envConfigs(envName);
centerline = [0 0; cfg.roadLength 0];
halfWidth = cfg.corridorHalfWidth;

veh = initVehicleState(cfg.egoStart(1), cfg.egoStart(2), cfg.egoStart(3), cfg.egoStart(4));
obstacles = initObstacles(cfg.obstacles);
tracks = initTrackList();
plannerState = [];
planParams = struct('corridorHalfWidth', halfWidth, 'targetSpeed', cfg.targetSpeed);
roadInfo.centerline = centerline; roadInfo.halfWidth = halfWidth;

stepsPerPlan = max(1, round(planPeriod/dtSim));
nSteps = round(cfg.simDuration/dtSim);

% ---- figure + persistent graphics handles (updated via set(), not
% re-plotted every frame, so this stays reasonably fast over thousands
% of frames) ----
fig = figure('Color','k','Position',[100 100 1000 560], 'InvertHardcopy','off');
if ~showLive, set(fig,'Visible','off'); end
ax = axes('Parent',fig,'Color','k','XColor','w','YColor','w'); hold(ax,'on'); grid(ax,'on');
set(ax, 'GridColor', [0.3 0.3 0.3]);
xlim(ax, [0 cfg.roadLength]); ylim(ax, [-(halfWidth+2), halfWidth+2]);
xlabel(ax,'X position (m)','Color','w'); ylabel(ax,'Y position (m)','Color','w');

plot(ax, [0 cfg.roadLength], [0 0], '--', 'Color',[0.6 0.6 0.6], 'DisplayName','Road center');
plot(ax, [0 cfg.roadLength], [halfWidth halfWidth], '-', 'Color',[0.4 0.4 0.4], 'LineWidth',1.2, 'DisplayName','Road boundary');
plot(ax, [0 cfg.roadLength], [-halfWidth -halfWidth], '-', 'Color',[0.4 0.4 0.4], 'LineWidth',1.2, 'HandleVisibility','off');

gTheta = linspace(0,2*pi,50);
plot(ax, cfg.egoGoalX + goalRadius*cos(gTheta), 0 + goalRadius*sin(gTheta), ':', 'Color',[0.2 0.9 0.4], 'DisplayName','Goal radius');
plot(ax, cfg.egoGoalX, 0, 'p', 'MarkerSize',16, 'MarkerFaceColor',[0.2 0.9 0.4], 'MarkerEdgeColor',[0.2 0.9 0.4], 'DisplayName','Goal');

hTraj    = plot(ax, nan, nan, 'b-', 'LineWidth',1.6, 'DisplayName','Vehicle trajectory');
hPath    = plot(ax, nan, nan, 'g--', 'LineWidth',1.3, 'DisplayName','Selected path');
hPredLn  = plot(ax, nan, nan, ':', 'Color',[0.2 0.9 0.9], 'DisplayName','Prediction');
hTrueObs = plot(ax, nan, nan, 'o', 'Color','r', 'MarkerSize',10, 'LineWidth',1.6, 'DisplayName','True obstacle');
hDetObs  = plot(ax, nan, nan, 'o', 'Color','m', 'MarkerSize',7,  'LineWidth',1.4, 'DisplayName','Detected obstacle');
hPredObs = plot(ax, nan, nan, '*', 'Color',[0.2 0.9 0.9], 'MarkerSize',8, 'DisplayName','Predicted obstacle');
hVeh     = plot(ax, nan, nan, 's', 'MarkerFaceColor','b', 'MarkerEdgeColor','w', 'MarkerSize',9, 'DisplayName','Vehicle');

% Legend: deliberately left in its DEFAULT (light box, dark text)
% styling rather than forced fully dark to match the rest of the
% figure. During development, explicitly setting the legend's
% background 'Color' property (not just 'TextColor', which worked fine
% alone) silently broke text rendering in the GNU Octave environment
% used for testing -- no error was thrown, the labels simply vanished.
% That is a rendering quirk of Octave's 'gnuplot' toolkit specifically
% (a real MATLAB renderer is very likely fine with it), but since it
% failed silently rather than erroring, I could not build a reliable
% try/catch around it, and chose not to ship a legend I could not
% confirm actually displays its labels. A plain legend box next to a
% dark plot is a normal, working look -- if you want a fully dark
% legend and confirm it renders correctly in your MATLAB, add:
%   set(lg, 'TextColor','w', 'Color',[0.12 0.12 0.12], 'EdgeColor',[0.5 0.5 0.5]);
lg = legend(ax,'Location','eastoutside'); %#ok<NASGU>
hTitle = title(ax,'', 'Color','w', 'Interpreter','none');

if saveVideo
    projRoot = fileparts(fileparts(mfilename('fullpath')));
    animDir = fullfile(projRoot,'results','animations');
    if ~exist(animDir,'dir'), mkdir(animDir); end
    vw = VideoWriter(fullfile(animDir, [envName '.mp4']), 'MPEG-4'); %#ok<*TNMLP>
    vw.FrameRate = max(1, round(1/(dtSim*renderDecimation)));
    open(vw);
end

trajX = []; trajY = [];
lastMode = 'replanned'; lastRisk = 0; lastTTC = Inf;
lastPlanWallClock = tic; measuredHz = 1/planPeriod;
status = 'RUNNING';
lastStep = nSteps;

for step = 1:nSteps
    t = (step-1)*dtSim;
    obstacles = obstacleUpdate(obstacles, dtSim, t, roadInfo);

    if mod(step-1, stepsPerPlan) == 0
        measuredHz = 1/max(toc(lastPlanWallClock), 1e-6);
        lastPlanWallClock = tic;

        switch sensorMode
            case 'groundtruth'
                fused = groundTruthAsFused(veh, obstacles);
            case 'lidar'
                if strcmp(lidarBackend,'toolbox')
                    [L, lidarActuallyFellBack] = virtualLidarToolbox(veh, obstacles, struct());
                    fused = lidarOnlyAsFused(L);
                else
                    fused = lidarOnlyAsFused(virtualLidar(veh, obstacles, struct()));
                end
            case 'radar'
                fused = radarOnlyAsFused(virtualRadar(veh, obstacles, struct()));
            case 'camera'
                fused = cameraOnlyAsFused(obstacles, virtualCamera(veh, obstacles, struct()));
            case 'fusion'
                if strcmp(lidarBackend,'toolbox')
                    [L, lidarActuallyFellBack] = virtualLidarToolbox(veh, obstacles, struct());
                else
                    L = virtualLidar(veh, obstacles, struct());
                end
                R = virtualRadar(veh, obstacles, struct());
                C = virtualCamera(veh, obstacles, struct());
                fused = sensorFusion(veh, L, R, C, struct());
            otherwise
                error('animateScenario:badSensorMode','Unknown sensorMode "%s"', sensorMode);
        end

        if strcmp(trackerBackend, 'toolbox')
            % Graceful fallback, same pattern as simulation/runScenario.m's
            % perceive(): if Sensor Fusion and Tracking Toolbox isn't
            % available/licensed or an API mismatch surfaces, fall back
            % to the plain tracker rather than crashing the animation.
            try
                tracks = trackObstaclesToolbox(tracks, fused, veh, planPeriod, struct('resetTracker', step==1));
            catch ME
                persistent warnedAnimTrackerFallback
                if isempty(warnedAnimTrackerFallback)
                    warning('animateScenario:trackerToolboxUnavailable', ...
                        ['trackObstaclesToolbox failed (%s: %s) -- falling back to the ' ...
                         'plain-MATLAB tracker for the rest of this session. See ' ...
                         'TOOLBOX_INTEGRATION.md.'], ME.identifier, ME.message);
                    warnedAnimTrackerFallback = true;
                end
                tracks = trackObstacles(tracks, fused, veh, planPeriod, struct());
                trackerActuallyFellBack = true;
            end
        else
            tracks = trackObstacles(tracks, fused, veh, planPeriod, struct());
        end

        if smoothTypes
            tracks = smoothTrackTypes(tracks, struct('resetHistory', step==1));
        end

        % Recompute the on-screen label every cycle from what ACTUALLY
        % ran, not just what was requested -- see the comment where
        % trackerActuallyFellBack/lidarActuallyFellBack are declared.
        backendBits = {};
        if strcmp(trackerBackend,'toolbox')
            if trackerActuallyFellBack
                backendBits{end+1} = 'Tracker=SensorFusionTbx (FELL BACK to plain)';
            else
                backendBits{end+1} = 'Tracker=SensorFusionTbx';
            end
        end
        if strcmp(lidarBackend,'toolbox')
            if lidarActuallyFellBack
                backendBits{end+1} = 'LiDAR=LidarTbx (FELL BACK to plain clustering)';
            else
                backendBits{end+1} = 'LiDAR=LidarTbx';
            end
        end
        if smoothTypes, backendBits{end+1} = 'TypeSmoothing=ON'; end
        if isempty(backendBits)
            backendLabel = 'Backends: plain MATLAB (baseline)';
        else
            backendLabel = ['Backends: ' strjoin(backendBits, ', ')];
        end

        [ctrl, plannerState, diag] = adaptivePlanner(veh, tracks, centerline, plannerState, planParams);
        lastMode = diag.mode; lastRisk = diag.maxRisk; lastTTC = diag.worstTTC;

        if strcmp(diag.mode,'emergency_brake')
            set(hPath,'XData',nan,'YData',nan);
        else
            set(hPath,'XData',diag.chosenPath.xy(:,1),'YData',diag.chosenPath.xy(:,2));
        end

        detX = nan(1,numel(tracks)); detY = nan(1,numel(tracks));
        predSegX = []; predSegY = [];
        for i = 1:numel(tracks)
            detX(i) = tracks(i).x; detY(i) = tracks(i).y;
            p = predictObstaclePosition(tracks(i), predictHorizon);
            predSegX = [predSegX, tracks(i).x, p.x, nan]; %#ok<AGROW>
            predSegY = [predSegY, tracks(i).y, p.y, nan]; %#ok<AGROW>
        end
        set(hDetObs,'XData',detX,'YData',detY);
        if numel(tracks) > 0
            predStarX = arrayfun(@(k) predSegX((k-1)*3+2), 1:numel(tracks));
            predStarY = arrayfun(@(k) predSegY((k-1)*3+2), 1:numel(tracks));
        else
            predStarX = nan; predStarY = nan;
        end
        set(hPredObs,'XData',predStarX,'YData',predStarY);
        set(hPredLn,'XData',predSegX,'YData',predSegY);
    end

    veh = bicycleModel(veh, ctrl.a, ctrl.delta, dtSim);

    collided = false;
    trueX = []; trueY = [];
    for k = 1:numel(obstacles)
        if ~obstacles(k).active, continue; end
        trueX(end+1) = obstacles(k).x; trueY(end+1) = obstacles(k).y; %#ok<AGROW>
        if hypot(obstacles(k).x-veh.x, obstacles(k).y-veh.y) < 1.2
            collided = true;
        end
    end

    if mod(step-1, renderDecimation) == 0 || collided || veh.x >= cfg.egoGoalX
        trajX(end+1) = veh.x; trajY(end+1) = veh.y; %#ok<AGROW>
        set(hTraj,'XData',trajX,'YData',trajY);
        set(hVeh,'XData',veh.x,'YData',veh.y);
        set(hTrueObs,'XData',trueX,'YData',trueY);

        ttcStr = 'Inf'; if isfinite(lastTTC), ttcStr = sprintf('%.2f s', lastTTC); end
        set(hTitle,'String', sprintf('SIH 26037 Adaptive Planner | %s | %s | t = %.2fs | Objects = %d | Planner = %.2f Hz | TTC = %s | Risk = %.2f\n%s', ...
            upper(envName), status, t, numel(trueX), measuredHz, ttcStr, lastRisk, backendLabel));

        drawnow;
        if saveVideo
            writeVideo(vw, getframe(fig));
        end
        if showLive && playbackSpeed > 0
            pause(max(0, dtSim*renderDecimation/playbackSpeed));
        end
    end

    if collided
        status = 'COLLISION'; lastStep = step; break;
    end
    if veh.x >= cfg.egoGoalX
        status = 'REACHED GOAL'; lastStep = step; break;
    end
end

if lastStep == nSteps && ~strcmp(status,'COLLISION') && ~strcmp(status,'REACHED GOAL')
    status = 'TIME LIMIT';
end
set(hTitle,'String', sprintf('SIH 26037 Adaptive Planner | %s | %s | final t = %.2fs\n%s', upper(envName), status, (lastStep-1)*dtSim, backendLabel));
drawnow;
if saveVideo
    writeVideo(vw, getframe(fig)); % hold the final status frame briefly
    close(vw);
    fprintf('Saved video: %s\n', fullfile(animDir, [envName '.mp4']));
end
end

% ---- perception dispatch, deliberately duplicated (not imported) from
% simulation/runScenario.m's local `perceive` helper: that helper is a
% MATLAB subfunction private to runScenario.m and cannot be called from
% another file without editing runScenario.m, which this module is
% intentionally not doing. ----
function fused = groundTruthAsFused(veh, obstacles)
fused = struct('x',{},'y',{},'bearing',{},'rangeRate',{},'type',{},'confidence',{},'trueId',{});
for k = 1:numel(obstacles)
    o = obstacles(k);
    if ~o.active, continue; end
    f.x=o.x; f.y=o.y; f.bearing=atan2(o.y-veh.y,o.x-veh.x)-veh.theta;
    f.rangeRate=NaN; f.type=o.type; f.confidence=1; f.trueId=o.id;
    fused(end+1) = f;
end
end

function fused = lidarOnlyAsFused(L)
fused = struct('x',{},'y',{},'bearing',{},'rangeRate',{},'type',{},'confidence',{},'trueId',{});
for i = 1:numel(L)
    f.x=L(i).x; f.y=L(i).y; f.bearing=L(i).bearing;
    f.rangeRate=NaN; f.type='unknown'; f.confidence=0; f.trueId=L(i).trueId;
    fused(end+1) = f;
end
end

function fused = radarOnlyAsFused(R)
fused = struct('x',{},'y',{},'bearing',{},'rangeRate',{},'type',{},'confidence',{},'trueId',{});
for i = 1:numel(R)
    f.x=R(i).x; f.y=R(i).y; f.bearing=R(i).bearing;
    f.rangeRate=R(i).rangeRate; f.type='unknown'; f.confidence=0; f.trueId=R(i).trueId;
    fused(end+1) = f;
end
end

function fused = cameraOnlyAsFused(obstacles, C)
fused = struct('x',{},'y',{},'bearing',{},'rangeRate',{},'type',{},'confidence',{},'trueId',{});
for i = 1:numel(C)
    idx = find([obstacles.id] == C(i).trueId, 1);
    if isempty(idx), continue; end
    f.x=obstacles(idx).x; f.y=obstacles(idx).y; f.bearing=C(i).bearing;
    f.rangeRate=NaN; f.type=C(i).type; f.confidence=C(i).confidence; f.trueId=C(i).trueId;
    fused(end+1) = f;
end
end

function v = getOr(s,f,default)
if isfield(s,f) && ~isempty(s.(f)), v = s.(f); else, v = default; end
end
