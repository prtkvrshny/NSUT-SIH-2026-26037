# RoadRunner Integration Guide

This project does not attempt to author RoadRunner scene files
(`.rrscene`/`.rrhd`). RoadRunner is a GUI application for building 3D
road scenes; it is not a text format that can be hand-authored reliably
outside the app, and there is no way to verify a hand-written scene
file against the real application from this environment. Building the
actual RoadRunner scene is work for you, inside RoadRunner itself.

What follows is (1) what to build in RoadRunner to match each of the 5
environments in `simulation/envConfigs.m`, and (2) how to get this
project's simulated paths and obstacle trajectories out as CSV/`.mat`
so you can bring them into RoadRunner (or a RoadRunner Scenario) for
visual replay.

## 1. What to build per environment

All 5 environments in this project use a single straight reference
road, `cfg.roadLength` metres long, with a drivable corridor
`2*cfg.corridorHalfWidth` wide and NO enforced lane markings (obstacles
and the ego vehicle are free to use the full corridor width — see
`planning/frenetUtils.m`'s docstring). In RoadRunner, this maps to:

- **Village** (`roadLength=200`, corridor half-width 4 m): a single
  carriageway with informal/no lane markings, unpaved shoulder texture,
  low-density roadside clutter (structures, trees) close to the edge of
  the drivable width to visually motivate the tight corridor.
- **Intersection** (`roadLength=200`, corridor half-width 4 m): add a
  crossing road at roughly the x-positions where `envConfigs.m`'s
  `'intersection'` obstacles are placed (x=60–80), since the obstacles
  there use `'crossing'` behavior perpendicular to the main road.
- **Highway** (`roadLength=200`, corridor half-width 4 m, but obstacles
  configured for high speed): a wider, paved carriageway, painted lane
  lines (cosmetic only — this project's planner does not follow them),
  no roadside clutter.
- **Market** (`roadLength=120`, corridor half-width 3 m — narrower and
  shorter than the others, see `envConfigs.m`): dense roadside stalls,
  narrow effective drivable width, high pedestrian-density set dressing.
- **Cattle** (`roadLength=200`, corridor half-width 4 m): rural road,
  open shoulders (cattle can plausibly enter from either side).

Obstacle START positions/types for each environment are listed
explicitly in `simulation/envConfigs.m` (search for the `case` matching
each environment name) — use those x/y coordinates and types
(pedestrian/bike/auto/pushcart/cattle) to place RoadRunner actors at
matching start points. Obstacle MOTION is procedural
(`simulation/obstacleBehavior.m`) rather than a fixed path, so for a
literal visual match you'd want to either (a) drive RoadRunner Scenario
actors from the exported trajectory CSVs below rather than
hand-authoring their paths, or (b) approximate the behavior tag
(`'straightLine'`, `'crossing'`, `'weaving'`, `'randomWalk'`, `'erratic'`
— see the same file for what each does) with a hand-built RoadRunner
Scenario path.

## 2. Exporting trajectories for replay

`simulation/runScenario.m`'s output `log` struct already contains
everything needed to replay the ego vehicle's motion:

- `log.t` — timestamps [s]
- `log.vehX`, `log.vehY`, `log.vehTheta`, `log.vehV` — ego pose/speed

and obstacle ground truth is available from the `obstacles` struct
array (`simulation/initObstacles.m` / `simulation/obstacleUpdate.m`)
at the end of a run, or by logging it at every step yourself if you
need obstacle trajectories over time (the current `runScenario.m` only
logs the ego vehicle's full trajectory and obstacles' FINAL positions,
to keep the log lightweight — see below for how to extend this).

### Quick export (ego trajectory only, using what's already logged)

```matlab
setupProject();
[log, metrics] = runScenario('village', struct('seed', 1));
T = table(log.t, log.vehX, log.vehY, log.vehTheta, log.vehV, ...
          'VariableNames', {'t','x','y','theta','v'});
writetable(T, 'results/village_ego_trajectory.csv');
save('results/village_run.mat', 'log', 'metrics');
```

### Full export including obstacle trajectories over time

`runScenario.m` does not currently log per-step obstacle positions (only
their final positions, for the end-of-run plot). If you need full
obstacle trajectories for RoadRunner Scenario playback, the
lowest-risk way to get them is to copy `simulation/runScenario.m`'s main
loop into your own script and add one line inside the loop:

```matlab
obsLog(step,:) = arrayfun(@(o) o.x, obstacles); % repeat per obstacle/field you need
```

This is left as an exercise rather than built in, since it changes the
per-step data volume and I did not want to speculatively modify the
tested simulation loop for a use case (RoadRunner playback) that is
explicitly out of scope for this delivery.

## 3. Known gap

None of this has been checked against an actual RoadRunner import —
column layout, units, or coordinate-frame conventions RoadRunner
expects for Scenario actor playback may need adjusting on your end.
