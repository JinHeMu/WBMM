"""Plot already extracted diag04 arrays; does not connect to ROS."""
from pathlib import Path
import sys
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

source = Path(sys.argv[1]) if len(sys.argv) > 1 else Path('/tmp/wbmm_force_diag04_analysis')
out = Path(sys.argv[2]) if len(sys.argv) > 2 else source
out.mkdir(parents=True, exist_ok=True)
d = np.load(source / 'series.npz')
fault = json.loads((source / 'details.json').read_text())['fault_time']
plt.rcParams.update({'font.size': 10, 'axes.grid': True, 'grid.alpha': .25})

def finish(fig, axes, name, limits):
    for ax in axes:
        ax.axvline(fault, color='crimson', ls='--', lw=1)
        ax.set_xlim(*limits)
    axes[-1].set_xlabel('Time from bag start (s)')
    fig.savefig(out / name, dpi=160)
    plt.close(fig)

fig, axes = plt.subplots(6, 1, figsize=(12, 13), sharex=True, layout='constrained')
fig.suptitle('diag04: real force-MPC review; fault at 104.208 s')
axes[0].plot(d['tw'], d['w'][:, :3], lw=.8)
axes[0].set_ylabel('TCP force (N)'); axes[0].legend(['Fx', 'Fy', 'Fz'], ncol=3)
for ax, axis, label in [(axes[1], 0, 'Base v (m/s)'), (axes[2], 1, 'Base w (rad/s)')]:
    ax.plot(d['tb'], d['bc'][:, axis], label='Command', lw=.9)
    ax.plot(d['td'], d['od'][:, 3+axis], label='Reported velocity', lw=.8, alpha=.8)
    ax.set_ylabel(label); ax.legend(loc='upper left', ncol=2)
axes[3].plot(d['to'], d['sigma'], lw=1)
axes[3].axhline(.05, ls=':', color='darkorange', label='Soft reference 0.05')
axes[3].set_ylabel('Min singular value'); axes[3].legend()
axes[4].plot(d['to'], d['poserr']*1000, lw=1)
axes[4].axhline(100, ls=':', color='crimson'); axes[4].set_ylabel('Position error (mm)')
axes[5].plot(d['to'], np.rad2deg(d['angerr']), lw=1)
axes[5].axhline(np.rad2deg(.35), ls=':', color='crimson', label='Safety limit 0.35 rad')
axes[5].set_ylabel('Orientation error (deg)'); axes[5].legend()
finish(fig, axes, 'overview.png', (0, 125))

fig, axes = plt.subplots(5, 1, figsize=(12, 12), sharex=True, layout='constrained')
fig.suptitle('Sustained pull: target progression, elbow posture and tracking fault')
axes[0].plot(d['tw'], np.linalg.norm(d['w'][:, :3], axis=1), label='Processed force norm')
axes[0].set_ylabel('Force (N)'); axes[0].legend()
g = np.arange(90.1, 105.9, .02)
def velocity(ts, xyz):
    return np.column_stack([(np.interp(g+.1, ts, xyz[:, i])-np.interp(g-.1, ts, xyz[:, i]))/.2 for i in range(3)])
vt = np.linalg.norm(velocity(d['tt'], d['target'][:, :3]), axis=1)
va = np.linalg.norm(velocity(d['to'], d['positions']), axis=1)
# Fault replaces the moving target with a latched hold pose. Exclude that
# discontinuity from finite-difference speed; it is not a physical velocity.
vt[abs(g-fault) <= .15] = np.nan
axes[1].plot(g, vt, label='Target speed (0.2 s difference)')
axes[1].plot(g, va, label='Actual TCP speed (0.2 s difference)')
axes[1].set_ylabel('Speed (m/s)'); axes[1].legend()
axes[2].plot(d['to'], d['sigma'], label='Min singular value')
axes[2].axhline(.05, ls=':', color='darkorange', label='Soft reference')
axes[2].set_ylabel('Min singular value'); axes[2].legend(loc='upper right')
axq = axes[2].twinx()
axq.plot(d['to'], np.rad2deg(d['obs'][:, 5]), color='gray', alpha=.6, label='Joint 3')
axq.set_ylabel('Joint 3 (deg)'); axq.grid(False)
axes[3].plot(d['to'], d['poserr']*1000)
axes[3].axhline(100, ls=':', color='crimson'); axes[3].set_ylabel('Position error (mm)')
axes[4].plot(d['to'], np.rad2deg(d['angerr']))
axes[4].axhline(np.rad2deg(.35), ls=':', color='crimson', label='Safety limit 20.05 deg')
axes[4].set_ylabel('Orientation error (deg)'); axes[4].legend()
axes[4].text(105.9, 14, 'Hold target latched\nafter fault', fontsize=9, ha='right')
finish(fig, axes, 'fault_detail.png', (95, 106))
