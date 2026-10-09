"""Summarize and plot same-reference offline/ROS simulation comparisons."""
import gzip
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

folder = Path(__file__).parent
summary = {'offline': {}, 'mujoco': {}}
for name in ('previous', 'ideal', 'response', 'response_delay150'):
    z = np.genfromtxt(folder / (name + '_closed_loop.csv'), delimiter=',', names=True)
    item = {'samples': len(z), 'max_error_m': float(np.max(z['error_m'])),
            'rms_error_m': float(np.sqrt(np.mean(z['error_m'] ** 2)))}
    for label, start, stop in [('startup', 0, 3.0000001), ('release_settled', 18, 22.01)]:
        q = z[(z['time'] >= start) & (z['time'] < stop)]
        item[label] = {'base_displacement_m': float(np.hypot(q['base_x'][-1]-q['base_x'][0], q['base_y'][-1]-q['base_y'][0])),
                       'v_command_rms': float(np.sqrt(np.mean(q['v_cmd']**2))),
                       'w_command_rms': float(np.sqrt(np.mean(q['w_cmd']**2)))}
    summary['offline'][name] = item
fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
for name, label in [('ideal', 'Ideal-base MPC'), ('response', 'Response-aware MPC')]:
    rows = [json.loads(line) for line in gzip.open(folder/'headless'/(name+'.jsonl.gz'), 'rt').read().splitlines()]
    rows = [r for r in rows if r['phase'] != 'startup']
    times = np.array([r['sim'] for r in rows]); times -= times[0]
    cmd = np.array([r['base_command'] for r in rows])
    errors = np.array([r['error_m'] for r in rows])
    axes[0].plot(times, cmd[:,0], label=label, lw=1)
    axes[1].plot(times, cmd[:,1], label=label, lw=1)
    axes[2].plot(times, 1000*errors, label=label, lw=1)
    report = json.loads((folder/'headless'/(name+'.json')).read_text())
    item = {key:report[key] for key in ['passed','faults','unexpected_collision','tracking_rms_m','maximum_tracking_error_m','min_sigma']}
    for phase in ('1_zero', '3_zero'):
        q = [r for r in rows if r['phase'] == phase]
        q = [r for r in q if r['sim'] >= q[0]['sim'] + 1]
        u = np.array([r['base_command'] for r in q])
        item[phase] = {'v_command_rms':float(np.sqrt(np.mean(u[:,0]**2))),
                       'w_command_rms':float(np.sqrt(np.mean(u[:,1]**2))),
                       'tracking_rms_m':float(np.sqrt(np.mean([r['error_m']**2 for r in q])))}
    summary['mujoco'][name] = item
for ax, label in zip(axes, ['v command [m/s]', 'yaw command [rad/s]', 'TCP position error [mm]']):
    ax.set_ylabel(label);ax.grid(alpha=.3)
    for start, end in [(3,8),(11,16)]: ax.axvspan(start,end,color='gray',alpha=.12)
axes[0].legend();axes[2].set_xlabel('Simulation time after initial zero-force phase begins [s]')
fig.suptitle('Same delayed MuJoCo base, same +5 N / release / -5 N / release sequence')
fig.tight_layout();fig.savefig(folder/'base_response_comparison.png',dpi=160)
summary['driver'] = json.loads((folder/'driver_scheduling.json').read_text())
summary['interlock'] = json.loads((folder/'interlock/response.json').read_text())
(folder/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary['mujoco'],indent=2))
