# MANIFEST — SIH 2026, Problem Statement 26037

Maps every checklist item to the file/function that implements it.
Comments in each function repeat the relevant item number(s).

## Stage 1 — Base vehicle simulation (prerequisite, not a numbered item)
- `vehicle/initVehicleState.m`, `vehicle/bicycleModel.m`,
  `vehicle/purePursuitControl.m`
- Demo: `runStage1_vehicleOnly.m`

## Phase A — Multi-object world
| # | Item | File / function |
|---|------|------------------|
| 1 | Multiple obstacles | `simulation/initObstacles.m` |
| 2 | Static + dynamic obstacles | `simulation/obstacleBehavior.m` (`'static'` vs. moving behaviors) |
| 3 | Object types | `simulation/initObstacles.m` (`.type`), `simulation/obstacleBehavior.m` |
| 4 | Obstacle motion/update | `simulation/obstacleUpdate.m` |

Demo: `runStage2_staticObstacle.m` (items 1,2 minimally), `runStage4_indianRoadBrain.m` (all four, mixed types)

## Phase B — Multi-object perception
| # | Item | File / function |
|---|------|------------------|
| 5 | LiDAR detecting multiple objects | `sensors/virtualLidar.m` |
| 6 | Multiple detections (radar, camera, fusion) | `sensors/virtualRadar.m`, `sensors/virtualCamera.m`, `sensors/sensorFusion.m` |
| 7 | Object tracking | `sensors/trackObstacles.m`, `sensors/initTrackList.m` |
| 8 | Unknown/surprise obstacle appearance | `sensors/trackObstacles.m` (new-track spawn branch), `simulation/obstacleUpdate.m` + `simulation/initObstacles.m` (`spawnTime`) |

Demo: `runStage5_sensors.m` (sensors added one at a time via `sensorMode`)

## Phase C — Prediction
| # | Item | File / function |
|---|------|------------------|
| 9 | Predict obstacle position | `planning/predictObstaclePosition.m` |
| 10 | Predict vehicle position | `planning/predictVehiclePosition.m` |
| 11 | Time-to-collision | `planning/timeToCollision.m` |
| 12 | Future trajectory collision checking | `planning/checkFutureCollision.m` |

Demo: `runStage3_dynamicObstacle.m`

## Phase D — Risk + Indian-road intelligence
| # | Item | File / function |
|---|------|------------------|
| 13 | Traversability map | `planning/traversabilityMap.m` |
| 14 | SAFE / UNCERTAIN / BLOCKED | `planning/traversabilityMap.m` (`.state`, `.stateNames`) |
| 15 | Pedestrian/bike/auto/pushcart/cattle behavior | `simulation/obstacleBehavior.m` (motion), `planning/predictObstaclePosition.m` and `planning/riskScore.m` (per-type unpredictability/danger weight) |
| 16 | Risk score | `planning/riskScore.m` |

Demo: `runStage4_indianRoadBrain.m`

## Phase E — Real adaptive replanning
| # | Item | File / function |
|---|------|------------------|
| 17 | 10 Hz planning cycle | `simulation/runScenario.m` (`planPeriod`/`stepsPerPlan` timing loop) |
| 18 | Replan when risk changes | `planning/replanDecision.m`, driven by `planning/adaptivePlanner.m`'s `riskChanged` (also forces a replan when the current path is about to run out — see Bug fix #3 below) |
| 19 | Multi-obstacle path evaluation | `planning/selectSafePath.m` (loops every candidate x every track) |
| 20 | Emergency braking | `planning/emergencyBraking.m`, plus the independent reflex layer in `simulation/runScenario.m` |
| 21 | Dynamic obstacle avoidance | `planning/adaptivePlanner.m` (orchestrates C+D+19+20 every cycle) |

**Hard-requirement bug fix #1** (single-threat tracking): `planning/selectSafePath.m` and `planning/adaptivePlanner.m` — every safety decision loops over the FULL current track list, never a single cached "the threat." See in-code comments marked `*** Hard-requirement bug fix #1 ***`. Directly tested in development via a reconstruction of the reported failure (safe path chosen around obstacle A, obstacle B then appears on that same path — planner is confirmed to replan rather than continue).

**Hard-requirement bug fix #2** (zero-velocity assumption): `sensors/trackObstacles.m` — new tracks are never given `[0,0]` velocity; see `defaultWorstCaseVelocity` and the `.confirmed` flag (true only from the 2nd real sighting onward). See in-code comments marked `*** Hard-requirement bug fix #2 ***`.

**Bug fix #3** (path-exhaustion loop/orbit — found post-delivery, reported via the `cattle` scenario trajectory plot): `planning/predictVehiclePosition.m`, `planning/adaptivePlanner.m`, `planning/replanDecision.m`, `planning/frenetUtils.m` (new `'projectOnPath'` mode). Every candidate path only extends ~30 m past wherever it was generated, and nothing tracked how much of that distance had actually been driven; once obstacles cleared and risk stayed flat, the planner kept "following" the same stale path indefinitely (the safety re-check always predicted from the path's original start, never the vehicle's real position), and pure pursuit ended up orbiting the path's fixed final waypoint — a closed loop, most visible in `cattle` because its obstacle set clears well before its road ends. Fixed by (1) anchoring `predictVehiclePosition` to the vehicle's actual projected position along whatever path it's checking, and (2) adding an explicit "current path is about to run out" trigger to `replanDecision`/`adaptivePlanner`, alongside the existing infeasibility/risk-change triggers. See in-code comments marked `*** Bug fix (loop/orbit failure...) ***`, and NOTES.md.

## Phase F — Five environments
| # | Item | File / function |
|---|------|------------------|
| 22 | Village | `simulation/envConfigs.m`, case `'village'` |
| 23 | Intersection | `simulation/envConfigs.m`, case `'intersection'` |
| 24 | Highway | `simulation/envConfigs.m`, case `'highway'` |
| 25 | Market | `simulation/envConfigs.m`, case `'market'` |
| 26 | Cattle | `simulation/envConfigs.m`, case `'cattle'` |

All five are dispatched by the single `simulation/runScenario.m`, which calls the same `planning/adaptivePlanner.m` regardless of environment — no per-scenario planner logic. Demo/aggregator: `runDemo.m`.

## Phase G — Metrics
| # | Item | File / function |
|---|------|------------------|
| 27 | Collision rate | `simulation/computeMetrics.m` (`.collision`, per run), aggregated to a % in `runDemo.m` |
| 28 | Scenario completion % | `simulation/computeMetrics.m` (`.completed`), aggregated in `runDemo.m` |
| 29 | Replanning latency (real wall-clock) | `planning/adaptivePlanner.m` (`tic`/`toc`), summarized in `simulation/computeMetrics.m` |
| 30 | Path smoothness | `simulation/computeMetrics.m` (mean \|steering-rate\|) |
| 31 | Time-to-collision | `simulation/computeMetrics.m` (`.minTTC`, from `planning/timeToCollision.m` via the per-cycle log) |
| 32 | Emergency braking event count | `simulation/runScenario.m` (counts mode transitions into `'emergency_brake'`) |
| 33 | Average speed | `simulation/computeMetrics.m` |

Aggregated across all 5 environments into one table by `runDemo.m`, saved to `results/summary_metrics.csv`.

## Phase H — RoadRunner (documentation only, per project constraints)
Items 34–38: see `RoadRunner_Integration_Guide.md`. No `.rrscene`/`.rrhd` files are produced. `simulation/runScenario.m`'s log (`log.vehX`, `log.vehY`, obstacle ground-truth positions) is plain numeric data suitable for export to CSV/`.mat` for later replay in RoadRunner — see the guide for how to do that export.

## Visualization add-on (not a numbered checklist item — added post-delivery)
`visualization/animateScenario.m` and `runAllAnimations.m`: a live, animated 2D "dashboard" view (optionally saved as `.mp4` to `results/animations/`) offered as a fallback if RoadRunner isn't ready in time. Purely additive — built entirely on top of the existing tested public functions with no changes to `vehicle/`, `sensors/`, `planning/`, or `simulation/`. See NOTES.md's "Visualization module" section for exactly what was and wasn't verified.

## Toolbox add-ons (post-delivery, optional, opt-in — see TOOLBOX_INTEGRATION.md)
Not part of the numbered 38-item checklist. Added after you reported which
MATLAB toolboxes you actually have installed. Every item below is off by
default; nothing above this section changes unless you explicitly opt in.

| Toolbox | File(s) | What it does | Confidence |
|---|---|---|---|
| Automated Driving Toolbox | `vehicle/obstacleFootprints.m`, `planning/obbCollisionCheck.m` | Real per-type collision footprints (fixes the flagged flat-1.5m-radius limitation) | Octave-tested |
| (plain MATLAB, fixes a DL-shaped gap) | `sensors/smoothTrackTypes.m` | Fixes documented camera-type "flicker" | Octave-tested |
| Lidar Toolbox | `sensors/virtualLidarToolbox.m` | Multi-point returns + `pcsegdist` clustering into detections | Octave-tested (fallback clustering path only) |
| Navigation Toolbox | `planning/traversabilityMapToolbox.m` | `occupancyMap` wrapper around the existing tested classification | Classification Octave-verified identical; occupancyMap wrapper untested |
| Control System Toolbox | `tools/tunePurePursuitSpeedGain.m` | Offline speed-controller gain design/analysis (not wired into runtime) | Closed-form math hand-checked; toolbox calls untested |
| Computer Vision Toolbox | `visualization/annotateCameraFrame.m` | Cosmetic annotated demo frame from camera detections | Untested, but cannot affect anything else |
| Sensor Fusion and Tracking Toolbox | `sensors/trackObstaclesToolbox.m`, `sensors/initSIHTrackFilter.m` | `trackerGNN`-based alternate tracker, reimplements bug-fix #2 at the filter layer | **Highest risk in this delivery — untested, test first** |
| Deep Learning Toolbox | *(skipped — see TOOLBOX_INTEGRATION.md)* | — | N/A |
| Simulink / Stateflow | `Simulink_Stateflow_Integration_Guide.md` | Documentation only, same rationale as the RoadRunner guide | N/A (no files authored) |
| Image Processing Toolbox | *(skipped — see TOOLBOX_INTEGRATION.md)* | — | N/A |

See `TOOLBOX_INTEGRATION.md` for the full write-up: what each does, exactly
what was and wasn't able to be tested, and how to opt into each one.
