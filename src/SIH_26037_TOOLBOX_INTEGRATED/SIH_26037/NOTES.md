# NOTES — honesty, confidence, and known limitations

## Post-delivery fix: path-exhaustion loop/orbit (found via the `cattle` trajectory plot)

After initial delivery, a run's saved `cattle_trajectory.png` showed the ego
vehicle's path looping back on itself in a closed circle around x≈130–150,
well outside the ±4 m corridor, before continuing to the goal. Root-caused
and fixed using the same Octave setup as everything else in this file
(reproduced, instrumented, confirmed fixed — see below).

**What was actually happening**: `planning/adaptivePlanner.m` only
regenerates candidate paths (`generateCandidatePaths.m`, ~30 m horizon) when
`planning/replanDecision.m` says to, and that function only checked three
things: no path yet, current path no longer collision-feasible, or risk
changed materially since the last cycle. Nothing checked whether the vehicle
had simply driven far enough along its current ~30 m path that it was about
to run out. In the `cattle` scenario, once the last obstacle (the bike at
x=120) clears the 60 m relevance range, risk stays flat at 0 for a long,
uneventful final stretch to the goal at x=190 — so `riskChanged` never fires
again. Meanwhile the "is my current path still safe" re-check
(`selectSafePathSingle` → `selectSafePath` → `predictVehiclePosition.m`) had
its own bug: `predictVehiclePosition` assumed the path's first sample
(`candidateS(1)`) was always the vehicle's current position — true only the
instant a path is freshly generated, not on later cycles once the vehicle
has actually driven partway (or, here, all the way past) it. So the
"still safe?" check kept re-evaluating the same long-passed stretch of road
as risk-free, forever. With neither trigger ever firing, the SAME ~30 m
path — generated once, around x=142–172 — kept being "followed" long after
the vehicle had driven past its end (confirmed via instrumentation: at
t=36 s the vehicle's actual road position was s≈179 while its followed
path only extended to s=172.2). `vehicle/purePursuitControl.m`'s lookahead
search clamps to the path's LAST point once the vehicle is past every
sample, so the controller ended up steering the vehicle in a repeating
orbit around that single fixed, stale waypoint — exactly the closed loop
seen in the plot. Not a physics bug, not a cattle-specific obstacle bug: a
missing "the path I'm following is about to run out" replan trigger,
exposed by `cattle`'s long uneventful final stretch (55 s duration, and its
last obstacle is well short of its goal, unlike the other 4 environments)
but latent in every environment.

**Fix** (three small, targeted changes, no behavior changed for any
already-correct case):
1. `planning/frenetUtils.m` — added a `'projectOnPath'` mode: nearest-point
   projection of an (x,y) onto an arbitrary already-sampled path polyline,
   returning the interpolated arc-length from that path's own `.s` array
   (generalizes the existing `cart2frenet` centerline-projection logic to
   an arbitrary candidate path).
2. `planning/predictVehiclePosition.m` — now calls
   `frenetUtils('projectOnPath', ...)` to anchor every prediction to the
   vehicle's ACTUAL current position along the path being checked, instead
   of assuming the path's first sample. This alone makes the feasibility
   re-check honest (a stale, already-exhausted path now correctly predicts
   the vehicle stuck at that path's end, not still at its start).
3. `planning/replanDecision.m` / `planning/adaptivePlanner.m` — added an
   explicit 4th replan trigger: the vehicle's real projected position along
   its current path (via the same `'projectOnPath'` projection) is compared
   to that path's end; if the remaining distance drops below
   `params.pathReplanMargin` (default 10 m), a replan is forced regardless
   of risk/feasibility, exactly like the other three triggers.

**Verification**: reproduced with the exact original bug present (Octave,
`runScenario('cattle', struct('seed',1))` — instrumented log showed the
vehicle's road position running ~7–37 m ahead of its followed path's end
for the whole 33–45 s window, `max(abs(vehY))=11.1` m, over 2.5x the 4 m
corridor half-width). After the fix, same seed: `max(abs(vehY))=3.94` m
(inside the corridor), zero collisions, goal reached in ~34 s (down from
~48 s previously wasted looping). Re-ran seeds 1–4 for `cattle`
(all: zero collisions, goal reached, max lateral excursion 2.8–4.1 m) and
seed 1 for the other 4 environments (`village`, `intersection`, `highway`,
`market` — all: zero collisions, goal reached, no behavior change observed
outside the fixed condition, since none of them previously hit the
"path exhausted, risk flat" state within their shorter/busier road extents).
Did not re-run every seed for every environment after this fix (time
budget); the change is narrowly scoped enough (one new, clearly-gated
trigger; one projection anchor point corrected) that I'm confident it can
only ever cause an EARLIER, more accurate replan than before, never a worse
one — but flagging that full 20-run re-verification (matching the original
Phase F/G testing depth) was not repeated.

## How this was actually verified

I have no MATLAB runtime. Every `.m` file here is **untested in MATLAB,
desk-checked only** for MATLAB specifically. However, this project
deliberately avoids toolbox-specific calls (per the project brief's
"prefer plain MATLAB" instruction), which meant I could install **GNU
Octave 8.4.0** in this sandbox and actually **execute** the large
majority of the codebase — not just read it. Octave is not MATLAB:
there are real differences (notably `struct2table`/`writetable`, used
only in the final summary step of `runDemo.m`, are not implemented in
Octave — confirmed via `exist('struct2table')` returning `0`). Anything
below marked "Octave-tested" means I ran it and it produced correct
output; it is NOT the same claim as "MATLAB-verified," and you should
still expect small MATLAB-specific syntax issues are possible even in
Octave-tested code.

Bugs actually found and fixed via this testing (not just theoretical):

1. **Off-by-one in track confirmation** (`sensors/trackObstacles.m`):
   original code delayed a track's `.confirmed = true` transition to
   the 3rd sighting instead of the 2nd, because the age check ran
   *before* the increment. Caught by a written test, fixed.
2. **Uncertainty-radius blowup** (`planning/predictObstaclePosition.m`):
   an unconfirmed, high-unpredictability track's predicted uncertainty
   circle could grow past 12 m over a 6 s horizon — wider than the
   entire road corridor — causing the vehicle to emergency-brake with
   the nearest real obstacle 40 m away and nothing actually threatening
   it. Fixed by lowering the growth rate and adding a hard cap.
3. **Fixed-horizon lateral transition** (`planning/generateCandidatePaths.m`):
   candidate paths always spread their lateral swerve over the full
   30 m sampling horizon. Against an obstacle only ~9 m away, no
   candidate had swerved far enough by the time it reached the
   obstacle, so *every* candidate looked infeasible and the vehicle
   froze in place while the obstacle closed in. Fixed by making
   `planning/adaptivePlanner.m` shrink the transition distance toward
   the nearest relevant obstacle (floor 5 m, ceiling 15 m, 3 m margin).
4. **Long-range radar "ghost" velocities**: found, partially mitigated,
   NOT fully solved — see the dedicated section below.

Root-cause bugs #1 and #2 in this list were unrelated to the two
hard-requirement bugs you described but would have produced the exact
same class of failure (planner concluding "safe" when it wasn't) had
they shipped. I'm flagging them with the same seriousness.

## The two hard-requirement bug fixes: confidence HIGH

- **Bug fix #1 (single-threat tracking)**: `planning/selectSafePath.m`
  loops over every currently-tracked obstacle for every candidate, with
  no early-exit that would let checking obstacle A "clear" the need to
  check obstacle B. `planning/adaptivePlanner.m` uses this exact same
  function, with the exact same track list, both for full replanning
  and for re-validating the currently-followed path — there is no
  separate, weaker check for "is my current path still fine."
  I explicitly reconstructed your reported scenario in a test: chose a
  path that safely avoids a static obstacle, then introduced a second
  obstacle sitting on that same chosen path, and confirmed the planner
  abandons/replans rather than continuing to "follow." I'm confident in
  this fix specifically for the scenario as you described it. I have
  **not** tested more exotic variants (e.g., three or more obstacles
  with overlapping, moving threat envelopes).

- **Bug fix #2 (zero-velocity assumption)**: `sensors/trackObstacles.m`
  never assigns `[0,0]` to a new track; see `defaultWorstCaseVelocity`.
  A track is `.confirmed = false` until its second real sighting, and
  `planning/riskScore.m` / `planning/predictObstaclePosition.m` both
  treat unconfirmed tracks as *more* dangerous, never less. Verified via
  a direct test asserting new-track speed is never near zero and
  `.confirmed` flips correctly on the 2nd sighting.

## Weakest / least-confident items — read this before trusting anything else

- **Long-range radar velocity noise (Phase B, item 7)**: at 100+ m
  range, radar's ~1.5° bearing noise translates into several METRES of
  cross-range position error. A single-frame finite-difference (0.1 s
  apart) turns that into double-digit-m/s "ghost" velocities for
  objects barely moving — I observed this directly (an "auto" 140 m
  away logged with an 11.8 m/s velocity spike that didn't correspond to
  its real motion). A real tracker would run a Kalman/EKF filter per
  track to average this down over many frames; `trackObstacles.m` does
  a slow exponential blend plus a hard speed clamp instead — a
  mitigation, not a fix. Combined with a 60 m relevance filter in
  `adaptivePlanner.m` (obstacles farther away don't affect local path
  choice), this keeps it from causing visible failures in the 20
  scenario/seed combinations I ran, but I would NOT trust this tracker
  as-is for anything beyond ~60 m, and I would not be surprised if a
  different seed or a denser long-range obstacle set exposed it again.
- **Flat collision radius (1.5 m for every type)**: no per-type
  footprint. A cattle-sized obstacle and a pedestrian get identical
  clearance requirements. Deliberately not fixed, to avoid adding
  untested geometry code — flagging instead.
- **Camera classification flicker**: `virtualCamera.m`'s 8%
  misclassification rate combined with no temporal smoothing means a
  single track's inferred `.type` can visibly flip between calls (I
  observed a track labeled `cattle` then `bike` a tenth of a second
  later). This mildly affects risk-weighting and uncertainty growth but
  didn't cause a failure in testing. A real system would use majority-
  vote/hysteresis over several frames.
- **Replanning latency vs. the 10 Hz budget (Phase G item 29)**: mean
  latency in Octave ran ~50–120 ms per cycle across the 5 environments
  — i.e., some scenarios (market, highway) occasionally exceed the
  100 ms budget implied by 10 Hz in this *interpreted* environment.
  MATLAB's JIT is typically faster than Octave's interpreter for this
  kind of code, and compiled/codegen deployment would be faster still,
  but I cannot verify actual MATLAB timing from here. Treat the
  reported latency numbers as directionally informative, not as a
  guarantee real-time MATLAB execution clears the budget.
- **`traversabilityMap` (Phase D items 13–14)** treats (s,d) distance
  as locally Euclidean — an approximation that degrades on sharply
  curved roads. All 5 environments use straight reference centerlines,
  so this was never stress-tested on a curve.
- **`runDemo.m`'s final table/CSV step**: the per-trial loop and metric
  accumulation feeding it were verified via a manual replication of the
  exact same logic (Octave lacks `struct2table`/`writetable`). The
  `struct2table`/`writetable` calls themselves are standard MATLAB and
  were not executed anywhere in this session.
- **Obstacle behaviors are unbounded**: `simulation/obstacleBehavior.m`
  never clamps an obstacle to stay within any road/corridor boundary
  (`roadInfo` is passed in but unused for this purpose). Over the
  ~40–55 s scenario durations used here this didn't visibly matter, but
  a much longer run could see an obstacle wander arbitrarily far away.

## Visualization module (added after initial delivery)

`visualization/animateScenario.m` and `runAllAnimations.m` are a
**purely additive** live-animation layer, built after everything above
was already delivered and verified, as a RoadRunner fallback for
demos. They call the same tested public functions
(`envConfigs`/`initObstacles`/`obstacleUpdate`/`virtualLidar`/
`virtualRadar`/`virtualCamera`/`sensorFusion`/`trackObstacles`/
`adaptivePlanner`/`bicycleModel`) through their own loop — nothing in
`vehicle/`, `sensors/`, `planning/`, or `simulation/` was modified to
build this.

What was actually verified, specifically:
- The live plotting loop runs to completion, without crashing, across
  all 5 environments (Octave-tested).
- The plot CONTENT was visually verified by capturing PNG frames at
  several points in a run and reviewing them by hand: vehicle
  trajectory, true/detected/predicted obstacle markers, the live
  "selected path" overlay, goal marker, road boundaries, and the
  title's live t/Objects/Planner-Hz/TTC/Risk readout all render
  correctly and update as expected.
- Two real rendering bugs were found and fixed this way: (1) a
  MATLAB-only `ax.GridColor = ...` dot-notation assignment doesn't work
  in this Octave version — switched to the portable `set(ax,
  'GridColor', ...)`, which works in both; (2) figures default to a
  white background when exported/printed unless `'InvertHardcopy','off'`
  is set on the figure — a genuine, general MATLAB gotcha, not
  Octave-specific, now set explicitly.
- One cosmetic issue was found and NOT fully fixed: explicitly setting
  the legend's background `Color` (not `TextColor`, which worked fine
  alone) silently made the legend's text disappear in Octave's
  `gnuplot` rendering toolkit — no error was thrown, so this could not
  be caught defensively. Rather than ship a legend I couldn't confirm
  actually shows its labels, the legend was left in its default (light
  box, dark text) styling, which IS confirmed to render correctly with
  real labels. The rest of the figure (plot area, markers, title) stays
  dark-themed; only the legend box itself is a plain default box. If
  you're on real MATLAB and want a fully dark legend, the exact call to
  add back is in a comment right above the `legend(...)` line in
  `animateScenario.m`.
- NOT verified at all: the actual `VideoWriter`/`writeVideo`/`close(vw)`
  call sequence used when `saveVideo` is true. `VideoWriter` is a
  standard MATLAB function but is not implemented in the GNU Octave
  environment used for all other testing in this project (confirmed via
  `exist('VideoWriter')` returning 0), so `saveVideo=true` has never
  actually been executed, only written carefully. This is the single
  highest-risk piece of this add-on — test it yourself early (a short
  scenario, `showLive=false` for a quick batch check) before relying on
  it for a live demo.
- The "measured Planner Hz" in the title is real wall-clock timing
  (`tic`/`toc` between planning cycles, same technique as the Phase G
  latency metric elsewhere in this project), not a hardcoded value — it
  will vary run to run and machine to machine, same as the reference
  style you liked.
- The "Objects" count in the title is the number of currently-active
  GROUND TRUTH obstacles, not the number of tracks. The tracker's known
  noise/ghost-track issue (see the long-range radar section above)
  means raw track count can be visually misleading in a demo; ground
  truth count is what a viewer intuitively expects "Objects" to mean.

## Medium confidence

- Phase F (5 environments) and Phase G (metrics): the underlying
  mechanics (Phase A–E) are shared and well-tested; the environment
  configs and duration/road-length tuning (`envConfigs.m`) were
  iterated against actual Octave runs (originally `market` and `cattle`
  didn't finish in the allotted time purely because of a config
  mismatch between target speed and road length — fixed and
  re-verified). Tested across seeds 1–4 for all 5 environments: 19/20
  runs zero-collision and completed; one run (`market`, seed 3) didn't
  finish within its time budget in the densest, slowest scenario — no
  collision, just ran out of time. I have not run more than 4 seeds per
  environment, so I can't rule out rarer failure modes at scale.

## High confidence

- Stage 1 (bicycle model + pure pursuit): Octave-tested, straightforward.
- Stages 2–4 (static obstacle, dynamic obstacle, mixed Indian-road
  obstacle types): Octave-tested end to end, including the risk/
  traversability machinery.
- `planning/frenetUtils.m`: tested including round-trips on a kinked
  (non-straight) centerline, not just a straight one.
- Core prediction/collision/risk primitives (`predictObstaclePosition`,
  `predictVehiclePosition`, `timeToCollision`, `checkFutureCollision`,
  `riskScore`): each unit-tested individually with hand-checked
  expected values, not just run-without-crashing.

## Things I did not attempt

- Any RoadRunner file authoring (per your explicit constraint) — see
  `RoadRunner_Integration_Guide.md`.
- Any Simulink/Stateflow file authoring, for the same reason — see
  `Simulink_Stateflow_Integration_Guide.md`.
- Curved-road geometry, intersections with actual crossing *roads*
  (rather than crossing *obstacles* on one road), and multi-lane-width
  variation across the corridor.

## Post-delivery: optional toolbox add-ons

The original delivery intentionally stayed in plain MATLAB throughout
(per the brief's "prefer plain MATLAB for portability"). After
delivery, you provided a list of MATLAB toolboxes actually available
on your machine (Automated Driving, Computer Vision, Control System,
Deep Learning, Image Processing, Lidar, Navigation, Sensor Fusion and
Tracking, Simulink, Stateflow) and asked for them to be integrated
where useful. That work is documented separately and in full in
**`TOOLBOX_INTEGRATION.md`** rather than repeated here — short version:
every add-on is opt-in (nothing above changes by default), confidence
varies a lot by file (from "Octave-tested" to "genuinely never
executed, test this yourself first"), and `TOOLBOX_INTEGRATION.md`
ranks every add-on file honestly by that confidence level rather than
presenting them as uniformly trustworthy.

## Post-delivery fix #2: toolbox tracker velocity convergence bug (found via a live MATLAB run, village scenario)

After the toolbox add-ons above were delivered, you ran the animation on
real MATLAB with `trackerBackend='toolbox'` and reported (with a
screenshot) the vehicle following an absurd path — a hard swerve up to
the road edge starting well before any obstacle was actually close,
while `trackerBackend='plain'` stayed correct. Root-caused by reasoning
through the code (NOT by direct reproduction — this sandbox still has
no Sensor Fusion and Tracking Toolbox to run `trackerGNN` against).

**What was actually happening (best-effort diagnosis)**:
`sensors/initSIHTrackFilter.m` deliberately biases a brand-new track's
velocity toward a conservative "closing on the ego" worst case, to
reimplement hard-requirement bug fix #2 (the toolbox's own default init
helpers zero a new track's velocity instead, which is exactly the bug
that fix exists to prevent). That's correct for the track's very first
cycle. The problem: a Kalman filter's velocity states are only
corrected INDIRECTLY, through position/velocity correlation built up
over several predict/correct cycles — not instantly from a single new
position measurement, the way a plain finite difference is. That means
the wrong worst-case velocity guess can keep biasing the filter's own
velocity estimate for several cycles after real position data should
have overridden it — long enough to explain a sustained wrong evasive
swerve, not a single-frame blip. `sensors/trackObstacles.m` (the plain
tracker, confirmed correct) never had this problem because it was never
a Kalman filter — it uses the worst-case guess for exactly one frame,
then a direct finite-difference from the second sighting onward, which
corrects essentially instantly.

**The fix**: `sensors/trackObstaclesToolbox.m` now keeps `trackerGNN`
for what it's actually good at (association + ID/lifecycle management)
but stops trusting its internal Kalman velocity estimate past a track's
first cycle. From the second sighting of a given TrackID onward,
velocity is recomputed as a direct finite-difference between this
cycle's and last cycle's filtered position — deliberately mirroring the
plain tracker's approach, since that's the one already confirmed to
work. A velocity sanity clamp (default 20 m/s) was added as a second
line of defense regardless of root cause, and `AssignmentThreshold` was
raised from an initial low-confidence guess of 9 to the more standard
30, since overly tight gating could itself cause spurious
re-association.

**How this was actually verified**: the finite-difference fix is pure
plain-MATLAB logic and was extracted and unit-tested in isolation
(Octave) with synthetic position histories: confirmed it overrides a
stale/wrong Kalman velocity guess by the very next real sighting
regardless of how slowly the underlying filter converges, correctly
tracks a genuinely moving obstacle, and the clamp caps an absurd
velocity jump. What remains unverified is everything upstream of that
fix — the actual `trackerGNN`/`objectDetection`/`trackingKF` calls
themselves, same as before. This should resolve the specific failure
mode you observed; it cannot rule out a different one showing up next.
`sensors/trackObstaclesToolbox.m` has NOT graduated out of "highest
risk" in TOOLBOX_INTEGRATION.md — it has had one specific, evidenced
bug addressed. Please re-test on your machine before relying on it
live.
