# SIH 26037 — Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads

## 1. Project Information

**Project Title:** Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads  
**Problem Statement (PS):** 26037  
**Category:** Software  
**Domain focus:** Autonomous driving / intelligent transportation for unstructured Indian-road conditions

> This repository implements a shared adaptive-planning pipeline for vehicle navigation in mixed, non-lane-following traffic. The project is designed around five configurable environments: **village, intersection, highway, market, and cattle**.

---

## 2. Problem Statement

Conventional lane-following approaches assume structured roads, predictable traffic, and well-defined lanes. Indian roads can instead contain mixed traffic and highly variable roadside behavior, including pedestrians, two-wheelers, auto-rickshaws, pushcarts, and cattle.

The vehicle therefore needs to continuously answer three questions:

1. **What is around the vehicle?**
2. **Where are those obstacles likely to move next?**
3. **What path and speed should the vehicle use now?**

The challenge is to make these decisions continuously without relying on a single fixed lane or a single obstacle.

---

## 3. Proposed Solution

This project implements a **single shared adaptive planner** that is reused across multiple Indian-road scenarios.

The core pipeline is:

```text
World / Scenario
      ↓
Sensor / Perception Layer
      ↓
Obstacle Tracking
      ↓
Motion Estimation
      ↓
Future Obstacle Prediction
      ↓
TTC + Risk Assessment
      ↓
Traversability Assessment
      ↓
Candidate Path Generation
      ↓
Safe Path Selection / Replanning
      ↓
Speed Adaptation + Emergency Braking
      ↓
Vehicle Dynamics
      ↓
Updated Vehicle State
      ↺
```

The planner is deliberately **scenario-independent**. Environment-specific information is supplied through `simulation/envConfigs.m`; the same `planning/adaptivePlanner.m` is used for all configured environments.

---

## 4. Key Features

- **Multi-obstacle handling** for pedestrians, bikes, autos, pushcarts, and cattle.
- **Static and dynamic obstacle modelling** with configurable behaviors such as crossing, weaving, random walk, straight-line motion, and erratic motion.
- **Obstacle motion estimation and tracking** across planning cycles.
- **Future position prediction** to reason about where an obstacle may move next.
- **Time-to-Collision (TTC)** and risk scoring for safety-aware planning.
- **Traversability classification** using SAFE / UNCERTAIN / BLOCKED states.
- **Adaptive candidate-path generation** without assuming lane-following.
- **Adaptive speed selection** and emergency-braking behavior.
- **Path exhaustion / path-running-out protection** to avoid following an old local path beyond its useful endpoint.
- **Five reusable environments**: village, intersection, highway, market, cattle.
- **Configurable sensor backends**: ground-truth, LiDAR, radar, camera, and fused sensing.
- **Optional toolbox-backed implementations** for LiDAR processing, tracking, traversability, and camera annotation.
- **Scenario metrics** including collision, completion, replanning latency, path smoothness, TTC, emergency braking events, and average speed.
- **Optional live 2D animation** for demonstration when a 3D simulator is not available.

---

## 5. Technology Stack

### Core

- **MATLAB** — primary implementation language and runtime target.
- **GNU Octave 8.4.0** — used during development for regression testing where compatible; Octave is not a substitute for MATLAB verification.

### MATLAB / Simulink Products Used or Supported

The project contains a plain-MATLAB implementation and optional toolbox-backed components for:

- Automated Driving Toolbox
- Computer Vision Toolbox
- Control System Toolbox
- Deep Learning Toolbox
- Image Processing Toolbox
- Lidar Toolbox
- Navigation Toolbox
- Sensor Fusion and Tracking Toolbox
- Simulink
- Stateflow

All toolbox add-ons are **opt-in** unless explicitly configured. See `TOOLBOX_INTEGRATION.md` for the current confidence level and activation method of each toolbox-backed component.

### Optional 3D Demonstration Layer

A future judge-facing 3D demonstration can use the **Unreal Engine based simulation workflow supported by Automated Driving Toolbox / Simulink 3D Animation**. The current repository keeps this separate from the core planner so the adaptive-planning logic remains runnable without a 3D simulator.

---

## 6. Architecture

The detailed architecture is documented in:

**`docs/architecture.md`**

At a high level:

```text
                     ┌──────────────────────────┐
                     │      Scenario World       │
                     │ village / market / ...   │
                     └────────────┬─────────────┘
                                  │
                                  ▼
                     ┌──────────────────────────┐
                     │ Sensors / Perception     │
                     │ LiDAR / Radar / Camera   │
                     └────────────┬─────────────┘
                                  │
                                  ▼
                     ┌──────────────────────────┐
                     │ Tracking + Motion        │
                     │ Estimation               │
                     └────────────┬─────────────┘
                                  │
                                  ▼
                     ┌──────────────────────────┐
                     │ Prediction + TTC + Risk  │
                     └────────────┬─────────────┘
                                  │
                                  ▼
                     ┌──────────────────────────┐
                     │ Adaptive Planner          │
                     │ Paths + Speed + Brake    │
                     └────────────┬─────────────┘
                                  │
                                  ▼
                     ┌──────────────────────────┐
                     │ Vehicle Dynamics          │
                     │ Bicycle Model             │
                     └────────────┬─────────────┘
                                  │
                                  └──────────↺
```

---

## 7. Repository Structure

```text
SIH_26037/
├── README.md
├── README.txt
├── requirements.txt
├── MANIFEST.md
├── NOTES.md
├── TOOLBOX_INTEGRATION.md
├── Simulink_Stateflow_Integration_Guide.md
├── RoadRunner_Integration_Guide.md
├── setupProject.m
├── runDemo.m
├── runStage1_vehicleOnly.m
├── runStage2_staticObstacle.m
├── runStage3_dynamicObstacle.m
├── runStage4_indianRoadBrain.m
├── runStage5_sensors.m
├── runAllAnimations.m
│
├── planning/
│   ├── adaptivePlanner.m
│   ├── generateCandidatePaths.m
│   ├── selectSafePath.m
│   ├── replanDecision.m
│   ├── traversabilityMap.m
│   ├── riskScore.m
│   ├── timeToCollision.m
│   ├── predictObstaclePosition.m
│   ├── predictVehiclePosition.m
│   ├── checkFutureCollision.m
│   ├── obbCollisionCheck.m
│   ├── emergencyBraking.m
│   └── ...
│
├── sensors/
│   ├── virtualCamera.m
│   ├── virtualLidar.m
│   ├── virtualRadar.m
│   ├── sensorFusion.m
│   ├── trackObstacles.m
│   ├── smoothTrackTypes.m
│   ├── virtualLidarToolbox.m
│   ├── trackObstaclesToolbox.m
│   └── ...
│
├── simulation/
│   ├── runScenario.m
│   ├── envConfigs.m
│   ├── initObstacles.m
│   ├── obstacleUpdate.m
│   ├── obstacleBehavior.m
│   ├── computeMetrics.m
│   └── plotResults.m
│
├── vehicle/
│   ├── initVehicleState.m
│   ├── bicycleModel.m
│   ├── purePursuitControl.m
│   └── obstacleFootprints.m
│
├── visualization/
│   ├── animateScenario.m
│   └── annotateCameraFrame.m
│
├── tools/
│   └── tunePurePursuitSpeedGain.m
│
├── docs/
│   └── architecture.md
│
└── results/
    └── generated at runtime
```

---

## 8. Installation / Setup

### MATLAB

1. Clone or download this repository.
2. Open MATLAB.
3. Set the repository folder as the current folder.
4. Run:

```matlab
setupProject
```

This adds the `vehicle`, `sensors`, `planning`, `simulation`, `visualization`, and `tools` folders to the MATLAB path.

### No Python environment is required

This is a **MATLAB project**, not a Python/FastAPI application. `requirements.txt` is therefore provided as a project-environment manifest listing the MATLAB runtime and optional products rather than as a `pip` dependency file.

---

## 9. Recommended Run Order

Because the core code was not executed in a MATLAB runtime during development, the repository is organized so the system can be verified incrementally.

### Stage 1 — Vehicle only

```matlab
runStage1_vehicleOnly
```

Verifies the bicycle-model vehicle and pure-pursuit control on an empty road.

### Stage 2 — Static obstacle

```matlab
runStage2_staticObstacle
```

Introduces a single static obstacle and the first adaptive path-selection loop.

### Stage 3 — Dynamic obstacle

```matlab
runStage3_dynamicObstacle
```

Introduces future obstacle prediction and dynamic collision checking.

### Stage 4 — Indian-road intelligence

```matlab
runStage4_indianRoadBrain
```

Uses mixed obstacle types and demonstrates traversability, risk, prediction, and adaptive planning.

### Stage 5 — Sensor pipeline

```matlab
runStage5_sensors
```

Runs the same village scenario through ground-truth, LiDAR, radar, camera, and fused sensor modes.

### Full demo

```matlab
runDemo
```

Runs the five configured environments and aggregates the scenario-level metrics into `results/summary_metrics.csv`.

### Single scenario

```matlab
setupProject
[log, metrics] = runScenario('village', struct( ...
    'seed', 1, ...
    'showPlot', true));

disp(metrics)
```

Valid `runScenario` options include `dtSim`, `planPeriod`, `sensorMode`, `seed`, `showPlot`, `reflexRadius`, `trackerBackend`, `lidarBackend`, and `smoothTypes`.

---

## 10. Configured Environments

The same `planning/adaptivePlanner.m` is reused across all five scenarios.

| Environment | Main characteristics |
|---|---|
| **Village** | Rural-road setting with cattle, pedestrian, pushcart, and bicycle activity |
| **Intersection** | Crossing auto-rickshaw, bicycle, and pedestrian traffic |
| **Highway** | Higher-speed traffic with autos and weaving bicycle behavior |
| **Market** | Narrower corridor with dense pedestrian and pushcart activity |
| **Cattle** | Dense close-range cattle interactions plus a bicycle |

Environment-specific dimensions, target speed, obstacle lists, behaviors, spawn times, and simulation duration are centralized in `simulation/envConfigs.m`.

---

## 11. Sensor Modes

`simulation/runScenario.m` supports the following sensor modes:

- `groundtruth`
- `lidar`
- `radar`
- `camera`
- `fusion`

The default is `fusion`.

Example:

```matlab
[log, metrics] = runScenario('village', struct( ...
    'sensorMode','lidar', ...
    'seed',1, ...
    'showPlot',true));
```

Optional toolbox backends can be enabled with fields such as:

```matlab
[log, metrics] = runScenario('village', struct( ...
    'trackerBackend','toolbox', ...
    'lidarBackend','toolbox', ...
    'smoothTypes',true));
```

See `TOOLBOX_INTEGRATION.md` before enabling toolbox-backed components because their current confidence levels differ.

---

## 12. Results and Evaluation Metrics

The project reports or stores metrics such as:

- Collision status / collision rate
- Goal completion / completion percentage
- Replanning latency
- Path smoothness
- Minimum TTC
- Emergency braking events
- Average speed
- Maximum detected objects
- Minimum obstacle distance
- Risk observations by SAFE / UNCERTAIN / BLOCKED category

The full multi-trial summary is written to:

```text
results/summary_metrics.csv
```

Scenario trajectory plots are also generated under `results/`.

---

## 13. Visualization

The repository includes a live MATLAB animation layer:

```matlab
setupProject
animateScenario('village')
```

To save a video:

```matlab
animateScenario('village', struct('saveVideo', true))
```

or run all environments:

```matlab
runAllAnimations(struct('saveVideo', true))
```

This visualization is a **2D MATLAB fallback**. It is additive and does not replace the planning, sensing, or vehicle functions.

For a future judge-facing 3D demonstration, the architecture can be extended with a supported **Unreal Engine + Simulink** layer while keeping the core adaptive-planning code unchanged. See `docs/architecture.md` for the proposed boundary between the algorithm layer and the 3D presentation layer.

---

## 14. Limitations / Current Confidence

This repository should be read with the following limitations in mind:

- The `.m` code was **not executed in MATLAB during development** because a MATLAB runtime was not available.
- Most core behavior was regression-tested in **GNU Octave 8.4.0**, which is similar but not identical to MATLAB.
- Long-range radar velocity estimation remains a known limitation because finite-difference estimates can amplify bearing noise.
- Collision geometry currently uses simplified footprint assumptions rather than fully type-specific physical meshes for every obstacle class.
- Camera-based obstacle type classification can flicker without temporal smoothing; an optional smoothing layer is provided.
- Some toolbox-backed files are intentionally optional and were not executed in the development sandbox.
- The exact real-time latency of the full planner in MATLAB must be measured on the target machine.

See `NOTES.md` for the detailed testing record, known bugs, fixes, and confidence levels.

---

## 15. Future Scope

Realistic next steps include:

1. Integrate the core planner with a real-time **Simulink model**.
2. Connect virtual camera / LiDAR / radar outputs to the same perception and tracking interfaces.
3. Add a judge-facing **Unreal Engine 3D environment** with a chase camera and top-down debug view.
4. Replace simplified obstacle footprints with class-specific 3D geometry.
5. Improve tracking with a Kalman/EKF-style multi-object tracker.
6. Add richer Indian-road scene assets and configurable scenario generation.
7. Validate planner timing and safety metrics on the final target MATLAB configuration.
8. Extend the planning layer toward real vehicle hardware / HIL experimentation.

---

## 16. Documentation Map

| Document | Purpose |
|---|---|
| `MANIFEST.md` | Maps project requirements/checklist items to files and functions |
| `NOTES.md` | Testing record, bug fixes, confidence levels, and known limitations |
| `TOOLBOX_INTEGRATION.md` | Optional MATLAB toolbox integrations and how to enable them |
| `Simulink_Stateflow_Integration_Guide.md` | Simulink / Stateflow integration guidance |
| `RoadRunner_Integration_Guide.md` | RoadRunner integration guidance / fallback planning |
| `docs/architecture.md` | Repository architecture and data flow |

---

## 17. Submission Safety

Before publishing the repository:

- Do not commit passwords, API keys, access tokens, or `.env` files containing secrets.
- Verify that the GitHub repository is accessible to reviewers.
- Keep generated or very large artifacts outside the repository when necessary, and provide a reviewer-accessible link from the submission documentation.

---

## 18. Project Status

The repository is structured as a **working research/demo prototype** with a shared adaptive-planning core, multi-scenario configuration, sensor modes, metrics, and optional toolbox integrations. MATLAB verification and final hardware/3D integration remain part of the project validation path.
