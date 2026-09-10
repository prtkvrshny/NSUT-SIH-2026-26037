# SIH 26037 — System Architecture

## 1. Architecture Goal

The project is built around a **single shared adaptive planner** for unstructured Indian-road conditions. The environment changes through configuration; the planning engine is not duplicated for each scenario.

The current implementation is a MATLAB/Octave-oriented simulation prototype. The architecture intentionally keeps the core algorithm separate from visualization and optional toolbox integrations so that the planner remains reusable.

---

## 2. Top-Level Architecture

```text
┌─────────────────────────────────────────────────────────────────────┐
│                         SCENARIO / WORLD                            │
│  village | intersection | highway | market | cattle               │
│                                                                     │
│  Static + dynamic actors:                                          │
│  pedestrian | bike | auto | pushcart | cattle                     │
└──────────────────────────────┬──────────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│                       SENSOR / PERCEPTION                           │
│                                                                     │
│  virtualCamera   virtualLidar   virtualRadar   sensorFusion        │
└──────────────────────────────┬──────────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│                     TRACKING / ESTIMATION                          │
│                                                                     │
│  trackObstacles                                                    │
│  smoothTrackTypes                                                  │
│  optional toolbox tracker backend                                  │
└──────────────────────────────┬──────────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│                  PREDICTION / SAFETY ASSESSMENT                     │
│                                                                     │
│  predictObstaclePosition                                           │
│  timeToCollision                                                   │
│  riskScore                                                         │
│  traversabilityMap                                                 │
└──────────────────────────────┬──────────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│                       ADAPTIVE PLANNER                              │
│                                                                     │
│  adaptivePlanner                                                   │
│    ├── path feasibility                                             │
│    ├── replanning decision                                          │
│    ├── candidate path generation                                   │
│    ├── safe path selection                                          │
│    └── emergency braking fallback                                  │
└──────────────────────────────┬──────────────────────────────────────┘
                               │
                         control {a, delta}
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        VEHICLE MODEL                                │
│                                                                     │
│  purePursuitControl → bicycleModel → updated [x,y,theta,v]        │
└──────────────────────────────┬──────────────────────────────────────┘
                               │
                               └─────────────── feedback loop ────────┐
                                                                      │
                                                                      └──► next sensor/planning cycle
```

---

## 3. Scenario Layer

### Source of truth

`simulation/envConfigs.m` defines the configured environments.

The current environments are:

- `village`
- `intersection`
- `highway`
- `market`
- `cattle`

Each configuration provides items such as:

- corridor half-width
- road length
- ego start state
- goal location
- target speed
- obstacle list
- obstacle type
- obstacle behavior
- obstacle spawn time
- simulation duration
- number of trials

### Design rule

**Do not create a separate planner per environment.**

The same `planning/adaptivePlanner.m` is reused. Scenario differences are injected through configuration and simulation state.

---

## 4. Simulation Loop

`simulation/runScenario.m` is the main scenario executor.

The loop is intentionally two-rate:

```text
Fast simulation loop
≈ 50 Hz (dtSim = 0.02 s)
        │
        ├── obstacle update
        ├── vehicle integration
        └── low-level reflex safety check

Planning loop
≈ 10 Hz (planPeriod = 0.10 s)
        │
        ├── perception
        ├── tracking
        ├── adaptive planning
        └── updated control target
```

This separation reflects the design intent that vehicle dynamics can run faster than the high-level planner.

---

## 5. Sensor / Perception Layer

The repository exposes multiple sensor modes through `simulation/runScenario.m`.

### Supported modes

```text
groundtruth
lidar
radar
camera
fusion
```

### Core files

```text
sensors/virtualCamera.m
sensors/virtualLidar.m
sensors/virtualRadar.m
sensors/sensorFusion.m
```

### Optional toolbox paths

```text
sensors/virtualLidarToolbox.m
sensors/trackObstaclesToolbox.m
sensors/initSIHTrackFilter.m
sensors/smoothTrackTypes.m
```

Toolbox-backed components are optional and must be enabled explicitly through `runScenario` options or planner parameters. This keeps the plain-MATLAB path available.

---

## 6. Tracking and Motion Estimation

`trackObstacles.m` converts sensor detections into persistent object tracks.

The conceptual data flow is:

```text
raw detections
      ↓
object association
      ↓
track state
      ↓
estimated x / y / type / velocity
      ↓
prediction + risk
```

The tracker also retains confidence / confirmation information so that a single noisy first detection does not automatically become an overconfident zero-velocity object.

The repository contains an optional toolbox tracker, but the plain tracker remains the default.

---

## 7. Prediction Layer

`planning/predictObstaclePosition.m` estimates future obstacle locations from current track information.

The planner then reasons about:

- current obstacle state
- estimated obstacle velocity
- predicted future position
- object uncertainty / unpredictability
- collision likelihood over a planning horizon

This is what changes the system from **reactive obstacle avoidance** to **predictive obstacle-aware planning**.

---

## 8. TTC and Risk Layer

`planning/timeToCollision.m` computes a predicted time-to-collision for the vehicle and a tracked obstacle.

`planning/riskScore.m` converts the collision timing, distance, and obstacle state into a normalized risk measure.

The planner also uses the risk result to modify decision-making, including speed adaptation and the urgency of replanning.

The project intentionally keeps the global TTC/risk values as **diagnostic metrics** as well as planner inputs. A global low TTC should not automatically force a path change if the obstacle is outside the currently relevant path corridor.

---

## 9. Traversability Layer

`planning/traversabilityMap.m` classifies the local road corridor according to obstacle occupancy.

The target semantic states are:

```text
SAFE
UNCERTAIN
BLOCKED
```

The planner uses these states together with dynamic prediction and path-feasibility checking to decide whether it should continue, slow down, replan, or brake.

`planning/traversabilityMapToolbox.m` provides an optional Navigation Toolbox representation for toolbox interoperability/visualization.

---

## 10. Candidate Path Generation

`planning/generateCandidatePaths.m` creates multiple lateral alternatives around the current reference path.

The project does **not** require conventional lane-following. Instead, the candidate set represents local lateral maneuver options inside the configured road corridor.

Conceptually:

```text
                 goal
                  ↑
       path A ────┘
       path B ────┘
 ego → path C ────┘
       path D ────┘
       path E ────┘
```

The exact number and geometry of candidate paths are controlled by the current implementation of `generateCandidatePaths.m`.

---

## 11. Safe Path Selection and Replanning

The central decision engine is:

```text
planning/adaptivePlanner.m
```

The planner:

1. Filters to locally relevant tracks.
2. Computes prediction, TTC, risk, and traversability.
3. Checks whether the current path is still feasible.
4. Forces a replan if the risk changes significantly.
5. Forces a replan when the current path is close to being exhausted.
6. Generates fresh candidate paths.
7. Evaluates candidates against all relevant tracked obstacles.
8. Selects the best feasible candidate.
9. Produces speed and steering commands.
10. Falls back to emergency braking when a safe trajectory cannot be found.

`planning/replanDecision.m` isolates the replanning trigger logic.

`planning/selectSafePath.m` performs the multi-obstacle safe-path evaluation.

---

## 12. Path-Exhaustion Protection

A local candidate path is only useful over its own spatial extent. If the vehicle drives beyond the useful end of that path and the planner never triggers a new plan, the pure-pursuit controller can continue steering toward an obsolete endpoint.

The current architecture addresses this through:

```text
planning/frenetUtils.m
planning/predictVehiclePosition.m
planning/replanDecision.m
planning/adaptivePlanner.m
```

The planner projects the current vehicle state onto the active path and monitors the **remaining path distance**. When the path is running out, a new local path is generated.

This is especially important in long, low-risk parts of the scenario where risk might otherwise stay flat for many cycles.

---

## 13. Vehicle Control Layer

The vehicle model is intentionally simple and transparent:

```text
vehicle/purePursuitControl.m
            ↓
steering command

adaptive planner / target speed
            ↓
acceleration command

            ↓
vehicle/bicycleModel.m
            ↓
[x, y, heading, velocity]
```

This allows the planner to be evaluated independently of a full vehicle dynamics simulator while still preserving closed-loop vehicle motion.

---

## 14. Safety Layers

The architecture contains more than one safety mechanism, but they have different responsibilities.

### High-level planning safety

`planning/selectSafePath.m` and `planning/checkFutureCollision.m` prevent the planner from intentionally selecting a predicted-collision trajectory.

### Emergency braking

`planning/emergencyBraking.m` supplies a last-resort deceleration command when the planner has no safe trajectory.

### Low-level reflex check

`simulation/runScenario.m` also contains a supplementary reflex-radius safety check that can force emergency braking on an immediate near-contact condition. This is a final safety net, not the main planning authority.

---

## 15. Metrics Layer

`simulation/computeMetrics.m` converts scenario logs into evaluation metrics.

`runDemo.m` runs multiple seeded trials for every environment and aggregates results into a summary table.

The intended evaluation layer includes:

```text
Collision rate
Completion percentage
Mean replanning latency
Path smoothness
Minimum TTC
Emergency braking events
Average speed
```

This turns the simulation from a visual demo into an experimentally measurable prototype.

---

## 16. Visualization Boundary

The repository deliberately keeps visualization separate from core behavior.

### Current visualization

```text
visualization/animateScenario.m
visualization/annotateCameraFrame.m
simulation/plotResults.m
```

These consume scenario/planner outputs but do not define the planner's behavior.

### Future judge-facing 3D layer

A 3D layer should follow the same separation principle:

```text
                 CORE ALGORITHM
                       │
                       │ state / detections / commands
                       ▼
              MATLAB / Simulink
                       │
                       ▼
              3D visualization API
                       │
                       ▼
                Unreal Engine
```

The 3D simulator should **render and simulate the environment and sensors**, while the existing adaptive-planning logic remains the decision-making authority.

---

## 17. Proposed Unreal / Simulink Extension

For a judge-facing 3D demonstration, the recommended extension is:

```text
┌──────────────────────────────────────────────────────────────┐
│                       UNREAL ENGINE                           │
│                                                              │
│  road / terrain / buildings / traffic actors                 │
│  ego vehicle / pedestrian / bike / auto / cattle / cart      │
│  chase camera / top-down camera                               │
│  virtual LiDAR / camera / radar                              │
└───────────────────────────────┬──────────────────────────────┘
                                │
                                │ sensor data / pose data
                                ▼
┌──────────────────────────────────────────────────────────────┐
│                         SIMULINK                              │
│                                                              │
│  sensor interface                                            │
│      ↓                                                       │
│  tracking + prediction                                       │
│      ↓                                                       │
│  TTC + risk + traversability                                 │
│      ↓                                                       │
│  adaptivePlanner                                              │
│      ↓                                                       │
│  steering + speed / braking                                  │
└───────────────────────────────┬──────────────────────────────┘
                                │
                                │ vehicle pose / control
                                ▼
                       ┌─────────────────┐
                       │ Unreal vehicle  │
                       └─────────────────┘
```

### Integration rule

Do **not** implement a second AI planner inside Unreal.

Unreal should be the visualization/simulation environment.

The SIH adaptive planner should remain the source of truth for the driving decision.

---

## 18. Judge-Facing 3D Presentation Concept

The recommended final visual layout is a two-view demonstration:

### View A — Chase / cinematic view

A third-person view behind the vehicle showing:

- road and terrain
- ego vehicle
- nearby dynamic actors
- vehicle movement and path

### View B — Top-down intelligence view

A debug-oriented view showing:

- ego vehicle
- obstacle positions
- predicted obstacle trajectories
- candidate / selected path
- TTC / risk / action state

This makes the project visually engaging while still exposing the actual adaptive-planning logic to judges.

---

## 19. Data Ownership Rules

To keep the architecture maintainable:

| Data | Source of truth |
|---|---|
| Scenario parameters | `simulation/envConfigs.m` |
| Dynamic obstacle state | simulation obstacle state |
| Sensor measurements | selected sensor backend |
| Persistent tracks | `sensors/trackObstacles.m` |
| Predicted obstacle state | planning prediction functions |
| Risk / traversability | planning risk + traversability functions |
| Selected path | `planning/adaptivePlanner.m` |
| Vehicle commands | planner/controller |
| Vehicle kinematics | `vehicle/bicycleModel.m` |
| Metrics | `simulation/computeMetrics.m` |
| Visualization | visualization layer only |

The 3D layer should consume these outputs rather than silently creating competing versions of the same state.

---

## 20. Extension Points

The architecture is intentionally open to the following extensions:

- real camera input
- real LiDAR / point-cloud processing
- radar-based tracking
- stronger multi-object filtering (for example Kalman/EKF variants)
- class-specific obstacle footprints
- richer uncertainty modelling
- Simulink / Stateflow implementation
- HIL testing
- Unreal Engine visualization
- real vehicle interface

These should be added behind stable interfaces so the planner remains reusable across scenarios.
