function filter = initSIHTrackFilter(detection)
% initSIHTrackFilter  Custom FilterInitializationFcn for trackerGNN.
%
% *** TOOLBOX ADD-ON (see TOOLBOX_INTEGRATION.md) ***
% *** Reimplements Hard-requirement bug fix #2 at the toolbox layer ***
% Sensor Fusion and Tracking Toolbox's own built-in init helpers (e.g.
% initcvekf) default a brand-new track's velocity states to ZERO --
% exactly the "zero-velocity assumption on first detection" failure
% mode this project's hard requirement says to never repeat. This
% custom init function instead assigns the SAME conservative,
% type-informed worst-case closing velocity used in the plain-MATLAB
% tracker (sensors/trackObstacles.m's defaultWorstCaseVelocity -- kept
% in sync by hand; if you tune one, tune both), AND gives the velocity
% states a correspondingly large initial covariance (this is a guess,
% not a measurement) rather than the toolbox default's small initial
% velocity covariance, which would make the filter overconfident in a
% guess.
%
% trackerGNN's FilterInitializationFcn only receives the detection that
% spawned the track, not the ego vehicle pose -- so, to reproduce the
% plain-MATLAB version's "assume it's closing on the ego" direction
% (not just an isotropic worst-case speed), sensors/trackObstaclesToolbox.m
% attaches the ego x/y (and radar rangeRate, if any) into each
% objectDetection's ObjectAttributes before calling the tracker, and
% this function reads them back out. If ObjectAttributes is somehow
% missing (should not happen via trackObstaclesToolbox.m, but defended
% here anyway), this falls back to an isotropic worst-case velocity
% guess (zero mean, high covariance) rather than erroring.
%
% CONFIDENCE (see TOOLBOX_INTEGRATION.md for the full breakdown): the
% [x;vx;y;vy] state order for trackingKF's '2D Constant Velocity'
% motion model is a stable, well-documented MathWorks convention I'm
% confident in. The exact constructor argument names/shapes below are
% my best-effort recollection and are COMPLETELY UNTESTED -- Sensor
% Fusion and Tracking Toolbox is not available in this sandbox's
% Octave, so this file could not be run even once, unlike
% planning/obbCollisionCheck.m. Treat this as the highest-uncertainty
% file in this delivery alongside sensors/trackObstaclesToolbox.m.

typeSpeeds = struct('pedestrian',2.0,'bike',6.0,'auto',10.0, ...
                     'pushcart',1.5,'cattle',1.5,'unknown',4.0);

attrs = struct();
if isprop(detection,'ObjectAttributes') || isfield(detection,'ObjectAttributes')
    a = detection.ObjectAttributes;
    if iscell(a) && ~isempty(a), a = a{1}; end
    if isstruct(a), attrs = a; end
end

if isfield(attrs,'type') && isfield(typeSpeeds, attrs.type)
    worstSpeed = typeSpeeds.(attrs.type);
else
    worstSpeed = typeSpeeds.unknown;
end

pos = detection.Measurement(1:2);
pos = pos(:);

haveEgo = isfield(attrs,'egoX') && isfield(attrs,'egoY');
if haveEgo
    losAngle = atan2(attrs.egoY - pos(2), attrs.egoX - pos(1));
    if isfield(attrs,'rangeRate') && ~isnan(attrs.rangeRate) && attrs.rangeRate < 0
        speed = max(-attrs.rangeRate, worstSpeed*0.5);
    else
        speed = worstSpeed;
    end
    vx0 = speed*cos(losAngle);
    vy0 = speed*sin(losAngle);
else
    % No ego pose available (should not normally happen -- see header):
    % fall back to a zero-mean, high-covariance isotropic guess.
    vx0 = 0; vy0 = 0;
end

velVar = worstSpeed^2; % "could plausibly be moving this fast, any direction"

state = [pos(1); vx0; pos(2); vy0];
stateCov = diag([1 velVar 1 velVar]);

try
    measNoise = detection.MeasurementNoise;
catch
    measNoise = diag([1 1]);
end

filter = trackingKF('MotionModel', '2D Constant Velocity', ...
    'State', state, 'StateCovariance', stateCov, ...
    'MeasurementNoise', measNoise, ...
    'ProcessNoise', diag([0.5 0.5]));
end
