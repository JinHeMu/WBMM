#!/usr/bin/env python3
"""Create measured-trace comparison figures, including matched-time arm poses."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
from ament_index_python.packages import get_package_share_directory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', required=True)
    args = parser.parse_args()
    folder = Path(args.directory)
    names = ['baseline', 'sigma', 'sigma_yoshikawa']
    labels = ['Costs off', 'Sigma margin', 'Sigma + Yoshikawa']
    colors = ['#d65a50', '#237cba', '#299773']
    traces, summaries = {}, {}
    for name in names:
        summaries[name] = json.loads((folder / f'{name}.json').read_text())
        traces[name] = [json.loads(line) for line in (folder / f'{name}.jsonl').read_text().splitlines()
                        if json.loads(line)['phase'] != 'startup']
    fig, axes = plt.subplots(2, 2, figsize=(12, 7.5), sharex=True)
    for name, label, color in zip(names, labels, colors):
        rows = traces[name]
        t = np.array([r['sim'] for r in rows]); t -= t[0]
        axes[0, 0].plot(t, [r['sigma'] for r in rows], label=label, color=color)
        axes[0, 1].plot(t, [r['yoshikawa'] for r in rows], color=color)
        axes[1, 0].plot(t, [r['tcp'][2] for r in rows], color=color)
        axes[1, 1].plot(t, [r['error_m'] * 1000 for r in rows], color=color)
    rows = traces['baseline']
    t = np.array([r['sim'] for r in rows]); t -= t[0]
    axes[1, 0].plot(t, [r['ref'][2] for r in rows], '--', color='#555555', label='Reference')
    axes[0, 0].axhline(.12, ls=':', color='#777777', label='Reserve target (soft)')
    for ax, title, ylabel in zip(axes.flat,
                                ['Weakest arm direction', 'Arm manipulability volume', 'Measured TCP height', 'TCP position tracking error'],
                                ['Minimum singular value', 'Yoshikawa product', 'World Z (m)', 'Position error (mm)']):
        ax.set_title(title, loc='left', fontsize=11)
        ax.set_ylabel(ylabel)
        ax.grid(alpha=.2)
        for start, end in [(3, 11), (15, 21)]:
            ax.axvspan(start, end, alpha=.045, color='black')
    axes[0, 0].legend(fontsize=8)
    axes[1, 0].legend(fontsize=8)
    for ax in axes[1]:
        ax.set_xlabel('Simulation time from initial quiet stage (s)')
    fig.suptitle('Force-MPC arm cost ablation: same 10 N keyboard history', fontsize=12)
    fig.tight_layout()
    fig.savefig(folder / 'comparison.png', dpi=180)
    fig.savefig(folder / 'comparison.svg')
    plt.close(fig)

    model = pin.buildModelFromUrdf(str(Path(get_package_share_directory('tracer_jaka_description')) / 'urdf/tracer_jaka_zu5.urdf'))
    data = model.createData()
    joints = [model.getJointId(f'joint_{i}') for i in range(1, 7)]
    qindices = [model.joints[j].idx_q for j in joints]
    snapshot_time = 14.8  # same force-history instant, before reverse force
    snapshots = {}
    fig = plt.figure(figsize=(12, 5.5))
    for index, (name, label, color) in enumerate(zip(names, labels, colors)):
        rows = traces[name]
        row = min(rows, key=lambda r: abs(r['sim'] - rows[0]['sim'] - snapshot_time))
        obs = np.array(row['obs'])
        q = pin.neutral(model); q[qindices] = obs[3:]
        pin.framesForwardKinematics(model, data, q)
        c, s = np.cos(obs[2]), np.sin(obs[2])
        rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        translation = np.array([obs[0], obs[1], 0])
        origins = np.array([data.oMi[j].translation.copy() for j in joints] +
                           [data.oMf[model.getFrameId('tool0')].translation.copy()])
        points = (rotation @ origins.T).T + translation
        base = np.array([[-.3, -.24, .05], [.3, -.24, .05], [.3, .24, .05], [-.3, .24, .05], [-.3, -.24, .05]])
        base = (rotation @ base.T).T + translation
        ax = fig.add_subplot(1, 3, index + 1, projection='3d')
        ax.plot(*base.T, color='#666666', lw=2)
        ax.plot(*points.T, '-o', color=color, lw=4, ms=5)
        ax.plot([translation[0], points[0, 0]], [translation[1], points[0, 1]], [0.05, points[0, 2]], color='#777777', lw=3)
        ax.scatter(*row['ref'], marker='x', color='black', s=40)
        ax.set_xlim(-.65, .35); ax.set_ylim(-.75, .25); ax.set_zlim(0, 1.1)
        ax.set_box_aspect((1, 1, 1.1)); ax.view_init(elev=22, azim=-55)
        ax.set_xlabel('World X (m)'); ax.set_ylabel('World Y (m)'); ax.set_zlabel('Z (m)')
        ax.set_title(f'{label}\nsigma={row["sigma"]:.4f}, elbow={np.degrees(obs[5]):.1f} deg', fontsize=10)
        snapshots[name] = {'elapsed_s': row['sim'] - rows[0]['sim'], 'sigma': row['sigma'],
                           'state': row['obs'], 'joint_origins_world': points.tolist()}
    fig.suptitle('Matched-time measured arm poses (14.8 s): URDF joint-origin geometry; cross = TCP reference', fontsize=11)
    fig.tight_layout()
    fig.savefig(folder / 'postures.png', dpi=180, bbox_inches='tight')
    fig.savefig(folder / 'postures.svg', bbox_inches='tight')
    plt.close(fig)
    history_checks = {}
    for name, rows in traces.items():
        checks = {}
        for phase, sign in [('0_f', -1), ('2_r', 1)]:
            samples = [r for r in rows if r['phase'] == phase]
            forces = [r['force'][2] for r in samples if r['force'] is not None]
            mean_force = float(np.mean(forces)) if forces else 0.0
            travel = float(np.linalg.norm(np.array(samples[-1]['tcp']) - samples[0]['tcp'])) if samples else 0.0
            checks[phase] = {'mean_force_tcp_z_n': mean_force, 'tcp_travel_m': travel,
                             'passed': sign * mean_force > 1.0 and travel > 0.01}
        checks['diagnostics_valid'] = all(r['metrics'] is not None and len(r['metrics']) == 6 and
                                          r['metrics'][5] == 1 for r in rows)
        history_checks[name] = checks
    diagnostics_valid = all(c['diagnostics_valid'] for c in history_checks.values())
    history_passed = all(c['0_f']['passed'] and c['2_r']['passed'] for c in history_checks.values())
    result = {'cases': summaries, 'matched_time_postures': snapshots,
              'force_history_checks': history_checks,
              'sigma_min_improvement_factor': summaries['sigma']['min_sigma'] / summaries['baseline']['min_sigma'],
              'sigma_yoshikawa_min_improvement_factor': summaries['sigma_yoshikawa']['min_sigma'] / summaries['baseline']['min_sigma'],
              'comparison_passed': diagnostics_valid and history_passed and
                  all(r['passed'] and not r['unexpected_collision'] for r in summaries.values()) and
                  summaries['sigma']['min_sigma'] > summaries['baseline']['min_sigma'] * 2 and
                  summaries['sigma']['tracking_rms_m'] <= summaries['baseline']['tracking_rms_m'] * 1.1,
              'metrics_are_diagnostic_only': all(r['metrics_are_diagnostic_only'] for r in summaries.values())}
    (folder / 'comparison.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ['cases', 'matched_time_postures']}, indent=2))


if __name__ == '__main__':
    main()
