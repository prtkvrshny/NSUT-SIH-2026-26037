# Toolbox Integration — what was added, and how much to trust it

You reported the following toolboxes installed: Automated Driving
Toolbox, Computer Vision Toolbox, Control System Toolbox, Deep
Learning Toolbox, Image Processing Toolbox, Lidar Toolbox, Navigation
Toolbox, Sensor Fusion and Tracking Toolbox, Simulink, Stateflow.

The original delivery deliberately used NONE of these ("prefer plain
MATLAB for portability" — see the original project brief). This
document covers what was added on top of that plain-MATLAB core, why,
and — this is the important part — exactly how much each piece was
able to be verified, since none of these toolboxes exist in this
sandbox's GNU Octave install (confirmed: `pkg install -forge control`
also failed here, no internet access to Octave Forge either, so not
even Octave's control package could be added to help verify the
Control System Toolbox piece).

**Nothing below changes the default behavior of anything you already
had.** Every toolbox add-on is either (a) a new, separate file that
nothing else calls unless you explicitly opt in, or (b) gated behind a
new params/opts field that defaults to the exact pre-existing
behavior. `runDemo.m`, all five `runStage*.m` scripts, and every
environment in `simulation/envConfigs.m` run exactly as before,
unchanged, unless you pass the new opt-in fields described below.
This was a deliberate choice, not an oversight: the plain-MATLAB core
was Octave-tested across 20 scenario/seed combinations before any of
this was added, and I did not want new, unverifiable toolbox code to
be able to silently change a result you already had reason to trust.

## Toolboxes NOT integrated, and why

- **Deep Learning Toolbox**: skipped entirely. `virtualCamera.m` never
  produces real images — only a ground-truth label plus a flat
  misclassification probability. There is no real training data
  anywhere in this simulation to train a network on; a "trained"
  classifier here would mean hand-fabricated weights standing in for
  training that never happened. Instead, the actual documented problem
  this would have addressed (camera classification "flicker" — see
  NOTES.md) is fixed directly with a real, Octave-tested technique:
  `sensors/smoothTrackTypes.m` (majority-vote hysteresis over recent
  frames). Deep Learning Toolbox is the right tool once real camera
  images exist to train on.
- **Simulink / Stateflow**: no `.slx` files were authored, for the
  same reason RoadRunner scene files weren't: they're binary/XML
  formats built through a GUI app, impossible to hand-author reliably
  as text, and impossible to validate from this sandbox. See
  `Simulink_Stateflow_Integration_Guide.md` for what to build yourself
  and how to wire it to the existing tested `.m` code with the least
  risk (short version: MATLAB Function blocks wrapping the existing
  functions, not a native reimplementation).
- **Image Processing Toolbox**: no standalone use found that would add
  real capability beyond what Computer Vision Toolbox's
  `insertObjectAnnotation` already covers for this project's one
  visualization add-on (see below). Not used.

## What WAS added, ranked by confidence

### Highest confidence — actually Octave-tested

- **`vehicle/obstacleFootprints.m` + `planning/obbCollisionCheck.m`**
  (Automated Driving Toolbox `vehicleDimensions` + hand-written SAT
  collision geometry). Fixes the flagged "flat 1.5 m collision radius
  for every obstacle type" limitation with real per-type footprints.
  The geometry math is plain MATLAB and was run through 8 hand-checked
  test cases in Octave (concentric boxes, exact-boundary touch,
  1 cm gaps, perpendicular separation, 45°-rotated boxes, and the
  ego-vs-pedestrian footprint sizes) — all passed. `vehicleDimensions`
  itself falls back to a plain struct with the same field names when
  unavailable (which is the path Octave actually exercised).
  **NOT wired into the live planning loop** — `planning/checkFutureCollision.m`
  and `planning/selectSafePath.m` (the tested hot path used by every
  environment) were deliberately left untouched to protect that
  already-verified code; this is an available, tested, opt-in building
  block, not an automatic replacement. See "How to actually use this"
  below for why and how you'd wire it in yourself.

- **`sensors/smoothTrackTypes.m`** (plain MATLAB, no toolbox). Fixes
  the documented camera-classification-flicker limitation. Fully
  Octave-tested: majority-vote convergence, per-scenario history
  reset, and multi-track isolation all verified. Opt in via
  `runScenario(..., struct('smoothTypes', true))`.

- **`sensors/virtualLidarToolbox.m`** (Lidar Toolbox `pointCloud` +
  `pcsegdist`, with a plain-MATLAB fallback clustering algorithm used
  automatically when the toolbox call fails — which is what let this
  be Octave-tested at all). Scatters realistic multi-point returns
  over each obstacle's footprint and clusters them back into
  detections, instead of one hand-placed point per obstacle. Verified
  in Octave (via the fallback path): correct output shape, empty-input
  edge case, and 200-trial sweeps to choose defaults. Quantified,
  honest result: with the shipped defaults (8 points/obstacle,
  2.0 m cluster distance), a single obstacle's points fail to all
  cluster together in ~6% of trials, and two obstacles 5 m apart merge
  into one detection in ~3% of trials — there is no parameter setting
  that eliminates both failure modes at once with this few points; see
  the file header for the tradeoff. The `pcsegdist`/`pointCloud` calls
  themselves are UNTESTED (no Lidar Toolbox in this sandbox); only the
  fallback clustering path actually ran. Opt in via `lidarBackend='toolbox'`.

### Medium confidence — written carefully, but genuinely untested

- **`planning/traversabilityMapToolbox.m`** (Navigation Toolbox
  `occupancyMap`). By design, the actual SAFE/UNCERTAIN/BLOCKED
  classification is NOT reimplemented — this function calls the
  existing, tested `traversabilityMap.m` and wraps its result in an
  `occupancyMap` object for toolbox interoperability/visualization
  only, inside a try/catch. Verified in Octave that (a) the
  classification output is byte-identical to calling
  `traversabilityMap.m` directly, and (b) the graceful fallback
  (`occMap=[]` + a warning message) triggers correctly when
  `occupancyMap` isn't available. The `occupancyMap` construction
  itself is untested. Opt in via `params.traversabilityBackend='toolbox'`
  passed to `adaptivePlanner`/`runScenario`.

- **`tools/tunePurePursuitSpeedGain.m`** (Control System Toolbox
  `tf`/`feedback`/`step`/`stepinfo`). An offline design script, not
  part of the runtime loop, that formally derives a recommended speed-
  controller gain. The underlying control theory (first-order plant,
  P controller, `kP = 4/settlingTime`) is exact and was hand-checked,
  independent of the toolbox. The toolbox calls meant to numerically
  confirm that answer are completely untested (no Control System
  Toolbox AND no internet access to install Octave's `control` package
  in this sandbox — genuinely zero ability to execute any of this).
  Does not touch `vehicle/purePursuitControl.m`'s existing, tested
  `kP=1.0`.

- **`visualization/annotateCameraFrame.m`** (Computer Vision Toolbox
  `insertObjectAnnotation`). Purely cosmetic demo visual — draws boxes
  + type labels for `virtualCamera.m` detections on a synthetic frame.
  Cannot affect planning/tracking/metrics even if wrong, since nothing
  consumes its output. Untested (no Computer Vision Toolbox here), but
  low-risk by construction.

### Highest risk — written, but I would not trust this without testing it yourself first

- **`sensors/trackObstaclesToolbox.m` + `sensors/initSIHTrackFilter.m`**
  (Sensor Fusion and Tracking Toolbox `trackerGNN`). This is the
  riskiest file in the whole delivery, confirmed the hard way: a real
  MATLAB run produced an absurd vehicle path (a hard swerve to the
  road edge starting well before any obstacle was actually close).
  **Root cause (reasoned through, not directly observed — I still
  cannot execute `trackerGNN`):** `initSIHTrackFilter.m` deliberately
  biases a brand-new track's velocity toward a conservative "closing on
  the ego" worst case (reimplementing hard-requirement bug fix #2,
  since the toolbox's own default init helpers zero it instead). That's
  correct for frame 1. The problem is that a Kalman filter's velocity
  states only get corrected INDIRECTLY, through position/velocity
  correlation built up over several predict/correct cycles — not
  instantly from one new position measurement — so the wrong worst-case
  velocity guess can keep biasing the filter's own estimate for several
  cycles after real data should have overridden it. That's long enough
  to produce a sustained wrong evasive swerve, not a one-frame blip.
  `sensors/trackObstacles.m` (the plain tracker you confirmed works)
  never had this problem because it isn't a Kalman filter at all — it
  uses the worst-case guess for exactly one frame, then a direct
  finite-difference from the second sighting onward, which corrects
  essentially instantly.
  **Fix applied:** `trackObstaclesToolbox.m` now keeps `trackerGNN` for
  what it's actually good at (detection-to-track association and
  ID/lifecycle management) but no longer trusts its internal Kalman
  velocity estimate past a track's first cycle. From the second
  sighting of a given TrackID onward, velocity is recomputed as a
  direct finite-difference between this cycle's and last cycle's
  filtered position (tracked in a small position-history map) —
  deliberately mirroring the plain tracker's approach, since that's the
  one you confirmed works. A velocity sanity clamp (default 20 m/s) was
  also added as a second line of defense regardless of root cause. Also
  raised `AssignmentThreshold` from an initial low-confidence guess of
  9 to 30 (the value most commonly seen in MathWorks `trackerGNN`
  examples) since too-tight gating could itself cause spurious
  re-association/re-spawning.
  **What was and wasn't verified this time:** the finite-difference
  velocity fix is pure plain-MATLAB logic and WAS unit-tested in
  isolation with synthetic position histories (confirmed: it overrides
  a stale/wrong Kalman velocity guess by the very next real sighting
  regardless of how slowly the filter itself converges, correctly
  tracks a genuinely moving obstacle, and the clamp caps an absurd
  jump). What remains UNVERIFIED, same as before, is everything
  upstream — the actual `trackerGNN`/`objectDetection`/`trackingKF`
  API calls, since this sandbox still has no Sensor Fusion and Tracking
  Toolbox to run them against. This fix should stop the specific
  failure mode you saw (sustained wrong swerve from a slow-converging
  velocity guess); it cannot rule out a *different* failure mode I
  haven't seen evidence of yet. Re-test on your machine before a live
  demo, the same as before this fix — this file has NOT graduated out
  of "highest risk," it has had one specific, evidenced bug addressed.

## How to actually use any of this

**In the live animation (recommended for demos):** `visualization/animateScenario.m`
(and therefore `runAllAnimations.m`, which just loops over it) now
accepts the same opt-in fields and shows an on-screen title line
naming exactly which backend(s) are active:

```matlab
setupProject();
animateScenario('village', struct( ...
    'trackerBackend', 'toolbox', ...   % sensors/trackObstaclesToolbox.m
    'lidarBackend',   'toolbox', ...   % sensors/virtualLidarToolbox.m
    'smoothTypes',    true));          % sensors/smoothTrackTypes.m

runAllAnimations(struct('trackerBackend','toolbox','lidarBackend','toolbox'));
```

Importantly, that on-screen label is **live-checked, not just
echoed back**: if a toolbox call actually fails partway through (e.g.
the tracker hits an API mismatch) the title updates to say so —
`Tracker=SensorFusionTbx (FELL BACK to plain)` — instead of continuing
to claim a backend is active after it silently stopped being true.
This existed as a real risk in this delivery specifically because the
Sensor Fusion and Tracking Toolbox tracker is genuinely untested (see
above); the label is there so a live demo shows the truth in real time
rather than you finding out afterward. In THIS sandbox (no toolboxes
installed at all in the Octave used for testing), every toolbox
backend falls back and the label correctly shows the fallback —
that's expected here and is not a sign anything is broken; on your
actual MATLAB with the real toolboxes licensed, you should see the
non-fallback label (except possibly the tracker, if my API guesses
there turn out to be wrong — see above).

**Without the animation:** the same opt-ins work directly through
`simulation/runScenario.m`:

```matlab
[log, metrics] = runScenario('village', struct( ...
    'trackerBackend', 'toolbox', 'lidarBackend', 'toolbox', 'smoothTypes', true));
```

For the OBB collision geometry (highest-confidence add-on, but not
wired into the hot path — see above), the intended usage is standalone
diagnostics/analysis, e.g. checking a specific close-call moment from a
logged run with a REAL geometric check instead of the flat-radius one:

```matlab
egoDims = obstacleFootprints('ego');
obsDims = obstacleFootprints(track.type);
[collides, gap] = obbCollisionCheck([veh.x veh.y veh.theta], egoDims, ...
                                     [track.x track.y atan2(track.vy,track.vx)], obsDims);
```

If you decide you DO want this wired into the live planning loop after
reviewing it yourself, the lowest-risk path is a NEW alternate function
(e.g. `checkFutureCollisionOBB.m`, mirroring `checkFutureCollision.m`'s
signature) selected by its own opt-in flag — not editing
`checkFutureCollision.m`/`selectSafePath.m` in place — for the same
reason every other add-on here is a parallel opt-in path rather than
an in-place rewrite: those two files are used by every environment in
every trial that was actually verified, and are exactly what "don't
sacrifice Phase A-E correctness to add coverage" (the original brief's
stated priority order) means to protect.

## Regression check performed on this delivery

Before and after all of the above, `runStage1_vehicleOnly.m` through
`runStage5_sensors.m` and a partial `runDemo.m` run were re-executed in
Octave with **default options only** (i.e., no toolbox opt-ins). Every
result (final position, collision/completion status, mode sequence,
peak risk, latency figures) matched the pre-add-on baseline exactly.
This confirms the add-ons above are additive, as designed — but it is
a regression check on the DEFAULT path, not a validation of the
toolbox-backed paths themselves, which remain as described above.

`visualization/animateScenario.m` was likewise re-run headless
(`showLive=false`) in Octave both with default opts (title correctly
read "Backends: plain MATLAB (baseline)", scenario completed
identically to before this file was touched) and with all three
toolbox opts on (title correctly showed both fallbacks triggering,
tracked in real time, and the scenario still completed without
crashing).
