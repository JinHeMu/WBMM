# Pre-contact-conditioned future-task rollout summary

- Experiment: `q_pre(i) -> direct wall-normal approach -> full wipe rollout`.
- No measured-state alignment or return to a canonical entry is inserted.
- Candidate pool: 133 collision-free whole-body IK states satisfying the same pre-contact tool pose.
- Selected: 50 deterministic Halton entries (offset 2026).
- Success: 5/50 (10.000000%).
- Wilson 95% interval: [0.043476, 0.213602].

## Successful rollout metrics (min / mean / max / std)

| Metric | Min | Mean | Max | Std |
|---|---:|---:|---:|---:|
| nominal execution time [s] | 442.600470 | 444.614333 | 448.817298 | 2.331026 |
| direct normal approach time [s] | 39.500000 | 39.500000 | 39.500000 | 0.000000 |
| contact coverage time [s] | 403.100470 | 405.114333 | 409.317298 | 2.331026 |
| base travel [m] | 10.199524 | 10.267905 | 10.340000 | 0.045044 |
| accumulated arm travel [rad] | 4.090919 | 4.230515 | 4.538992 | 0.168294 |
| min normalized joint margin  | 0.014711 | 0.022840 | 0.039898 | 0.009334 |
| min manipulability  | 0.027422 | 0.027424 | 0.027427 | 0.000002 |
| min self-collision clearance [m] | 0.005593 | 0.006040 | 0.006332 | 0.000352 |

## Interpretation boundary

Every rollout starts exactly at its CSV 9D entry state. Metrics cover only the normal approach and future wipe task. This remains an offline planning-level test; environment ESDF clearance, force tracking and closed-loop execution quality are not claimed.
