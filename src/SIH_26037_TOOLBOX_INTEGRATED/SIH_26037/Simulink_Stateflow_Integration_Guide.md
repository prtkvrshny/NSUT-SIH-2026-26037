# Simulink / Stateflow Integration Guide

This project does not attempt to author Simulink models (`.slx`) or
Stateflow charts. Both are binary/XML container formats built and
edited through their own GUI applications; like RoadRunner
(`.rrscene`/`.rrhd`, see `RoadRunner_Integration_Guide.md`), they are
not something that can be hand-authored reliably as text from outside
the app, and there is no way to open or validate an `.slx` file from
this sandbox. Rather than hand-write an `.slx` file I cannot check —
which would be far more likely to silently corrupt or simply fail to
open than to work — this is documentation of what to build yourself in
Simulink/Stateflow, and how to wire it to the existing, tested MATLAB
code with the least risk.

Both toolboxes were listed as available in your screenshot; this guide
exists so choosing not to produce `.slx`/Stateflow files is a
documented decision, not a silent omission.

## 1. What would actually map to Stateflow

`planning/adaptivePlanner.m` already contains a clean, explicit mode
machine: a per-cycle decision lands on one of `'follow'`,
`'replanned'`, or `'emergency_brake'`, driven by `replanDecision.m`'s
three (soon four, with the path-exhaustion fix) trigger conditions. If
you want a literal Stateflow chart, the shape is:

- States: `Follow`, `Replanning`, `EmergencyBrake` (mirrors `diag.mode`).
- Transition `Follow -> Replanning`: `~currentPathFeasible ||
  riskChanged || pathRunningOut` (see `planning/adaptivePlanner.m`'s
  `doReplan` computation and `planning/replanDecision.m`).
- Transition `Replanning -> Follow`: a feasible candidate was found
  (`~isempty(chosenNew)`).
- Transition `Replanning -> EmergencyBrake` / `Follow ->
  EmergencyBrake`: no feasible candidate (`isempty(chosenNew)`) or the
  independent reflex AEB trigger in `simulation/runScenario.m`
  (`minDistNow < reflexRadius`).

The safest way to use this in Stateflow is as a **thin supervisory
layer**, not a reimplementation: have each state's entry/during action
call the *existing, tested* MATLAB functions
(`selectSafePath`/`generateCandidatePaths`/`emergencyBraking`, etc.)
via a MATLAB Function block or a Stateflow MATLAB-action, rather than
re-deriving the mode logic natively in Stateflow syntax. Re-deriving
the same logic twice, in two languages, neither of which I can run
here, is exactly the kind of duplication that produces silent drift
between "what the chart does" and "what `adaptivePlanner.m` does" —
worse than not having a chart at all.

## 2. What would actually map to Simulink

The natural Simulink structure mirrors `simulation/runScenario.m`'s
existing two-rate loop (50 Hz vehicle integration, 10 Hz
perception+planning, see that file's header):

- A **fast-rate subsystem** (e.g. 0.02 s) wrapping
  `vehicle/bicycleModel.m` as a MATLAB Function block — it's already a
  single discrete-step function of `(veh, a, delta, dt)`, which is
  close to a drop-in fit for a MATLAB Function block with `veh`'s
  fields broken into separate scalar inports/outports (Simulink
  MATLAB Function blocks don't accept arbitrary structs as
  ports as cleanly as plain MATLAB does).
- A **slow-rate subsystem** (e.g. 0.1 s, triggered or rate-transitioned
  from the fast rate) wrapping the perception + `adaptivePlanner.m`
  call chain, similarly as a MATLAB Function block or an "Interpreted
  MATLAB Function" style wrapper.
- The obstacle/world simulation (`simulation/obstacleUpdate.m`,
  `simulation/obstacleBehavior.m`) as its own subsystem at the fast
  rate, since obstacles move every fast tick in the current MATLAB
  version.

## 3. Why a MATLAB Function block wrapper, not a native rebuild

For both of the above, wrapping the existing `.m` functions from inside
Simulink/Stateflow (via MATLAB Function blocks) is the recommended
path, not rewriting the logic natively in Simulink blocks or Stateflow
actions. Reasons:

1. Everything in `vehicle/`, `sensors/`, `planning/`, and `simulation/`
   has already been Octave-tested (see `NOTES.md`) — a native
   Simulink/Stateflow rebuild would throw that verification away and
   start over, untested, in a tool I cannot run at all.
2. MATLAB Function blocks accept plain MATLAB code (with some
   restrictions — no dynamic field access on some struct patterns,
   limited `containers.Map`/cell array support depending on your
   MATLAB Coder settings) largely as-is, so most of this project's
   functions should port with minor signature changes (structs ->
   flat scalar arguments) rather than a rewrite.
3. It keeps a single source of truth for the planning/collision logic
   in `.m` files that remain independently testable (via Octave, as
   done throughout this delivery) even after a Simulink model is
   built around them.

## 4. Known gap

None of this has been opened in real Simulink or Stateflow. Signal
dimensions, bus/struct handling specifics, and MATLAB Function block
restrictions on the exact code in this project (e.g. variable-size
struct arrays like `tracks`/`obstacles`, which MATLAB Function blocks
generally need declared as fixed-size or handled via `coder.varsize`)
are the most likely friction points — budget time for adapting struct
arrays to Simulink-friendly signal types before wiring the whole loop.
