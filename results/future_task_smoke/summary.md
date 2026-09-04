# 50-entry future-task rollout summary

- Scope: deterministic offline Wipe Planner rollouts, including fixed-base collision-checked entry alignment, wall-normal approach, and full contact coverage.
- Samples: 3 collision-free entry states; Halton offset 2026, joint perturbation +/-0.350000 rad.
- Success: 3/3 (100.000000%).
- Wilson 95% interval for rollout success probability: [0.438503, 1.000000].
- With zero observed failures, the exact one-sided 95% upper bound on failure probability is 0.631597.

## Successful-rollout metrics (min / mean / max / std)

| Metric | Min | Mean | Max | Std |
|---|---:|---:|---:|---:|
| planning wall time [s] | 0.103785 | 2.550001 | 3.797097 | 1.729847 |
| nominal execution time [s] | 438.982545 | 442.973214 | 445.473356 | 2.851773 |
| entry alignment time [s] | 0.000000 | 3.990669 | 6.490811 | 2.851773 |
| min normalized joint margin | 0.011101 | 0.011101 | 0.011101 | 0.000000 |
| min manipulability | 0.019004 | 0.022888 | 0.027427 | 0.003470 |
| min self-collision clearance [m] | 0.000637 | 0.001803 | 0.002808 | 0.000894 |

## Interpretation boundary

This is a planning-level feasibility experiment, not 50 closed-loop MuJoCo/real-robot executions. `nominal_execution_time_s` is planned path time. The clearance column is URDF self-collision clearance; ESDF/environment clearance and measured tracking/force quality require a closed-loop logger.

All sampled entries planned successfully. For these samples, success/failure does not discriminate entries; the defensible next claim is future-task quality / entry optimization, subject to closed-loop validation.
