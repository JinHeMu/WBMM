# Future-task entry rollout summary

- Scope: deterministic offline Wipe Planner rollouts, including fixed-base collision-checked entry alignment, wall-normal approach, and full contact coverage.
- Samples: 50 collision-free entry states; Halton offset 2026, joint perturbation +/-0.350000 rad.
- Success: 50/50 (100.000000%).
- Wilson 95% interval for rollout success probability: [0.928652, 1.000000].
- With zero observed failures, the exact one-sided 95% upper bound on failure probability is 0.058155.

## Successful-rollout metrics (min / mean / max / std)

| Metric | Min | Mean | Max | Std |
|---|---:|---:|---:|---:|
| planning wall time [s] | 0.106649 | 3.643651 | 4.019433 | 0.522737 |
| nominal execution time [s] | 438.982545 | 444.344134 | 445.486775 | 1.285497 |
| entry alignment time [s] | 0.000000 | 5.361589 | 6.504230 | 1.285497 |
| min normalized joint margin | 0.000016 | 0.006349 | 0.011101 | 0.005300 |
| min manipulability | 0.014319 | 0.024661 | 0.027427 | 0.004203 |
| min self-collision clearance [m] | 0.000064 | 0.002389 | 0.002808 | 0.000768 |

## Interpretation boundary

This is a planning-level feasibility experiment, not 50 closed-loop MuJoCo/real-robot executions. `nominal_execution_time_s` is planned path time. The clearance column is URDF self-collision clearance; ESDF/environment clearance and measured tracking/force quality require a closed-loop logger.

All sampled entries planned successfully. For these samples, success/failure does not discriminate entries; the defensible next claim is future-task quality / entry optimization, subject to closed-loop validation.
