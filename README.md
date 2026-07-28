# SlopeGuard: risk aware UGV navigation

A ground navigation stack for a post-landslide search robot, written in MATLAB. A drone surveys the debris field and produces a shared risk map; this is the ground vehicle layer that plans and drives through it.

The headline result is the ablation: **removing risk weighting makes paths 6 to 11 percent shorter and seven times more dangerous.**

![All five trial paths overlaid on the base cost map](figures/Trial%20paths.png)

## Why not just take the shortest path

A 25 m x 25 m debris field at 0.25 m resolution, containing a steep diagonal scarp, a loose debris zone, two impassable boulders and one clear access corridor. Cell values encode traversability from 0 (safe) to 1 (impassable).

The planner minimises a weighted cost rather than distance alone:

```
J(c) = w_d * d  +  w_r * r
```

where `d` is Euclidean step distance, `r` is per-cell terrain risk, `w_d = 1.0` and `w_r = 5.0`. The heuristic is Euclidean distance to goal, which stays admissible and consistent for `w_d >= 1`. Expansion is 8-connected; cells above 0.95 are treated as impassable.

A\* was hand-implemented rather than calling `plannerAStarGrid`, so the risk term sits directly inside the cost expression and every planner decision can be traced back to `w_d` and `w_r`.

## Results

| Trial | Length (m) | Mean cell risk | Nodes | Time (ms) |
|-------|-----------:|---------------:|------:|----------:|
| Baseline, no hazard | 28.13 | 0.090 | 4925 | 48.6 |
| Hazard A, corridor exit | 30.32 | 0.084 | 5997 | 44.8 |
| Hazard B, mid map | 31.06 | 0.083 | 6345 | 40.2 |
| Hazard C, near goal | 29.88 | 0.085 | 5753 | 35.1 |
| **Ablation, `w_r = 0`** | **26.52** | **0.649** | **75** | **2.4** |

The ablation row is the point. Turning off risk weighting collapses the search to 75 nodes and 2.4 ms and returns the geometrically optimal path, straight through the centre of the scarp where slope severity is highest. It is optimal under the wrong objective. For a robot whose failure mode is slipping and being lost, an order of magnitude more compute for an order of magnitude less path risk is the correct trade.

Across the four risk-weighted trials, mean cell risk stays in a narrow 0.083 to 0.090 band regardless of where the hazard appears. That predictability matters when a human supervisor has to approve moves remotely.

## Replanning

When a secondary slip is injected mid-mission, the planner re-searches against the updated cost map. Latency stays between **35 and 45 ms**, comfortably inside the 1.5 s operational budget, and no hazard caused a planning failure.

Full re-search was chosen over D\* Lite deliberately. On a 100 x 100 grid the absolute saving is tens of milliseconds, and a correct D\* Lite layered on top of an already custom A\* was a larger risk than the gain justified. On a 500 x 500 map, closer to a real disaster scene at this resolution, that calculus flips. It is the first thing I would change.

## Execution

Pure Pursuit drives a differential-drive kinematic model: 0.5 m track width, 0.5 m/s, 0.6 m lookahead, 10 Hz. The robot reaches the goal in 59.1 s, stopping 0.27 m short, with 2.6 percent tracking undershoot from corner cutting on tight bends.

## Repository layout

```
src/
  build_environment.m   builds the cost map, saves to .mat
  risk_aware_astar.m    global plan
  hazard_replan.m       injects a hazard mid-mission, replans
  run_simulation.m      Pure Pursuit execution
  experiments.m         the five trial study and plots
results/                saved .mat outputs from each stage
figures/                generated plots
```

Stages communicate through `.mat` files rather than shared workspace state. That mirrors the producer/consumer split between mapping, planning and control in a real navigation stack, and means any stage can be re-run without redoing the earlier ones.

## Running it

Requires MATLAB (developed on R2025b) with the Robotics System Toolbox and Navigation Toolbox.

```matlab
cd src
build_environment
risk_aware_astar
hazard_replan
run_simulation
experiments
```

## Known limitations

- Full re-search rather than incremental repair, see above
- The cost map is static between hazard updates; the wider concept assumes the aerial layer refines it continuously
- No observation noise is modelled. A real map carries thermal false positives, dust attenuation and imperfect slope estimates
- Only Pure Pursuit was tested. DWA would handle unmodelled local clutter better
- Hazards were injected one at a time; a sequence within a single mission would be a stronger test
