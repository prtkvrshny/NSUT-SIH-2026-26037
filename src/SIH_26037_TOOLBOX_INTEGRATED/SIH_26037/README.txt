SIH 2026 — Problem Statement 26037
Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles
on Unstructured Indian Roads
=======================================================================

WHAT THIS IS
------------
A plain-MATLAB (no required toolboxes) simulation of a bicycle-model
vehicle navigating roads with unstructured, non-lane-following traffic
(pedestrians, bikes, autos, pushcarts, cattle), using a shared adaptive
planner across 5 environments (village, intersection, highway, market,
cattle).

IMPORTANT — READ THIS FIRST
----------------------------
This code has NOT been run in MATLAB (no MATLAB runtime was available
during development). It HAS been extensively run and debugged in GNU
Octave 8.4.0, a free, closely related but NOT identical environment —
see NOTES.md for exactly what was tested, what bugs were found and
fixed as a result, and what remains a known limitation. Read NOTES.md
before trusting any specific claim about this project's behavior.

SETUP
-----
1. Unzip this folder anywhere on your MATLAB path, or open MATLAB with
   this folder as the current directory.
2. Run:
       setupProject
   This adds vehicle/, sensors/, planning/, simulation/ to your path.

RECOMMENDED RUN ORDER (build confidence incrementally)
-------------------------------------------------------
Because this could not be executed in MATLAB before delivery, the
build was structured so you can verify it incrementally yourself,
exactly the way the original spec's stage sequence intended:

    runStage1_vehicleOnly        % bicycle model + pure pursuit, no AI
    runStage2_staticObstacle     % first real planner: one static obstacle
    runStage3_dynamicObstacle    % predict + replan against one moving obstacle
    runStage4_indianRoadBrain    % mixed obstacle types, traversability + risk
    runStage5_sensors            % sensors added ONE AT A TIME (see note below)
    runDemo                      % all 5 environments + aggregated metrics

Each script prints a one-line summary and produces a plot. If a stage
misbehaves, the bug is very likely in that stage's newly-introduced
piece — see MANIFEST.md for exactly which file/function each stage
exercises.

runStage5_sensors.m runs all 5 sensor modes
('groundtruth'->'lidar'->'radar'->'camera'->'fusion') back to back for
convenience, but the spec's intent is "each proven working before the
next" — consider running one mode at a time yourself
(runScenario('village', struct('sensorMode','lidar'))) rather than
trusting the whole script blind, especially since the long-range radar
noise issue documented in NOTES.md is exactly the kind of thing that
benefits from checking one sensor at a time.

RUNNING THE FULL DEMO
----------------------
    runDemo

Runs all 5 environments (default 5 trials each, from
simulation/envConfigs.m's numTrials field), prints per-trial results,
and writes:
    results/summary_metrics.csv       — aggregated Phase G metrics table
    results/<scenario>_trajectory.png — one plot per environment (trial 1)

This can take a while — each environment run advances a 50 Hz
simulation for 30-55 simulated seconds with planning at 10 Hz, and this
was interpreted (not compiled) MATLAB/Octave code during testing.

RUNNING A SINGLE SCENARIO MANUALLY
------------------------------------
    setupProject();
    [log, metrics] = runScenario('village', struct('seed', 1, 'showPlot', true));
    disp(metrics)

opts fields (all optional): dtSim, planPeriod, sensorMode, seed,
showPlot, reflexRadius — see simulation/runScenario.m's header comment.

WHAT'S IN EACH FOLDER
------------------------
    vehicle/     bicycle-model dynamics, pure-pursuit path following
    sensors/     virtual LiDAR/radar/camera, sensor fusion, multi-object
                 tracker (this is where hard-requirement bug fix #2 lives)
    planning/    prediction, TTC, collision checking, traversability map,
                 risk scoring, candidate path generation/selection,
                 emergency braking, and the top-level adaptive planner
                 (this is where hard-requirement bug fix #1 lives)
    simulation/  obstacle world, the 5 environment configs, the main
                 simulation loop, metrics, plotting
    results/     created at runtime — figures, CSV logs, metrics table

DOCUMENTATION FILES
----------------------
    MANIFEST.md                    every checklist item -> file/function
    NOTES.md                       confidence levels, bugs found & fixed,
                                    known limitations (READ THIS)
    RoadRunner_Integration_Guide.md what to build in RoadRunner yourself,
                                    and how to export trajectories to it
    Simulink_Stateflow_Integration_Guide.md  same idea, for Simulink/Stateflow
    TOOLBOX_INTEGRATION.md         optional toolbox add-ons (see below) --
                                    what was added, how confident I am in
                                    each piece, and how to opt into them

OPTIONAL: TOOLBOX ADD-ONS (Automated Driving / Navigation / Lidar /
Sensor Fusion and Tracking / Computer Vision / Control System Toolbox)
--------------------------------------------------------------------------
Added after the original plain-MATLAB delivery, once you confirmed
which toolboxes you actually have. ALL OFF BY DEFAULT -- every stage
script and runDemo above behaves exactly as already described unless
you explicitly opt in via new opts/params fields. Confidence varies a
lot by piece, from "Octave-tested" to "never executed even once, test
this yourself first" -- see TOOLBOX_INTEGRATION.md for the honest,
file-by-file breakdown before using any of it. Quick start:

    setupProject();
    [log, metrics] = runScenario('village', struct( ...
        'trackerBackend', 'toolbox', ...   % Sensor Fusion and Tracking Toolbox
        'lidarBackend',   'toolbox', ...   % Lidar Toolbox
        'smoothTypes',    true));          % fixes camera-type flicker (plain MATLAB)

OPTIONAL: LIVE ANIMATED VISUALIZATION (RoadRunner fallback)
--------------------------------------------------------------
If RoadRunner integration doesn't come together in time, there is a
2D animated "dashboard" visualization you can show instead -- NOT a
3D scene, but it does show the planner actually swerving/braking over
time rather than a single static plot. It is purely additive: it does
not change the behavior of anything in vehicle/, sensors/, planning/,
or simulation/, and was added after the rest of this project was
already delivered and verified.

    setupProject();
    animateScenario('village');                              % live view
    animateScenario('village', struct('saveVideo', true));    % + save .mp4
    runAllAnimations(struct('saveVideo', true));               % all 5 envs

Videos are saved to results/animations/<scenario>.mp4 using MATLAB's
VideoWriter -- a standard function that could not be executed in the
GNU Octave sandbox used to test the rest of this project (Octave does
not implement VideoWriter), so this specific save path is untested by
me. The live plotting itself WAS tested and visually verified (frame
captures reviewed by hand) in Octave. See NOTES.md's "Visualization
module" section for exactly what was and wasn't checked, including a
specific note about the legend's default (not fully dark-themed)
styling.

By default the animation uses sensorMode='fusion', the same real
sensor pipeline used everywhere else in this project -- including its
documented long-range noise limitations (see NOTES.md). If you want a
guaranteed glitch-free demo reel for a pitch, pass
struct('sensorMode','groundtruth') instead, which skips simulated
sensor noise and only shows true obstacle positions.

A NOTE ON THE TWO HARD-REQUIREMENT BUGS
------------------------------------------
Both described failure modes (single-threat tracking; zero-velocity
assumption on first detection) have explicit, commented fixes — search
for "Hard-requirement bug fix #1" and "Hard-requirement bug fix #2" in
the source (they're in planning/selectSafePath.m /
planning/adaptivePlanner.m, and sensors/trackObstacles.m,
respectively). Bug fix #1 was specifically tested by reconstructing
your reported scenario end to end (see NOTES.md for the exact test).

A NOTE ON A THIRD BUG FOUND AFTER DELIVERY (cattle trajectory loop)
------------------------------------------------------------------
A saved cattle_trajectory.png showed the ego vehicle looping back on
itself in a closed circle partway down the road, well outside the
corridor. Root cause: the planner never had a "my current path is
about to run out" replan trigger, only "no path / path unsafe / risk
changed" -- so once cattle's obstacles cleared and risk stayed flat for
its long uneventful final stretch, the SAME ~30 m path kept being
"followed" long after the vehicle had actually driven past its end, and
the pure-pursuit controller ended up orbiting the path's fixed final
waypoint. Fixed in planning/frenetUtils.m, planning/
predictVehiclePosition.m, planning/replanDecision.m, and
planning/adaptivePlanner.m -- search for "Bug fix (loop/orbit failure"
in the source, and see NOTES.md's "Post-delivery fix" section for the
full root-cause writeup and re-verification (reproduced pre-fix,
confirmed fixed post-fix, in Octave, across seeds 1-4 for cattle and
seed 1 for the other 4 environments).
