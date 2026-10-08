#!/usr/bin/env python3
"""Compare XML and tuned physical servos on the complete six-joint model.

No ROS/MPC or viewer: results isolate the execution layer, not reference jitter.
Run from WBMM root with PYTHONPATH=src/sim/tracer_jaka_mujoco.
"""
import argparse
import json
from pathlib import Path
import mujoco
import numpy as np
import yaml
from tracer_jaka_mujoco.arm_servo import ArmBiasCompensator, configure_position_servos

PACKAGE = Path(__file__).resolve().parents[1]


def run(pose, tuned, kind, joint=0, sign=1):
    model = mujoco.MjModel.from_xml_path(str(PACKAGE / 'models/scene_force_follow_infinite.xml'))
    data = mujoco.MjData(model)
    joints = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'joint_{i}') for i in range(1, 7)]
    aids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f'joint_{i}_servo') for i in range(1, 7)]
    qa = model.jnt_qposadr[joints]; da = model.jnt_dofadr[joints]
    if tuned:
        params = yaml.safe_load((PACKAGE / 'config/arm_servo.yaml').read_text())['mujoco_bridge']['ros__parameters']
        config = params['arm_servo']
        configure_position_servos(model, aids, da, config['kp'], config['kv'], config['integrator'])
    mujoco.mj_resetDataKeyframe(model, data, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, pose))
    mujoco.mj_forward(model, data)
    compensator = ArmBiasCompensator(model, aids, da)
    initial = data.qpos[qa].copy()
    target = initial.copy()
    errors, positions, forces = [], [], []
    dt = model.opt.timestep
    duration = 4 if kind == 'sine' else 3
    contacts = set()
    for step in range(round(duration / dt)):
        t = step * dt
        if kind == 'step' and t >= 0.5:
            target[joint] = initial[joint] + sign * 0.15
        elif kind == 'sine':
            # ROS position-command sampling at 125 Hz, with <0.15 rad/s speed.
            sampled_t = np.floor(t * 125) / 125
            target = initial + 0.02 * np.sin(2 * np.pi * sampled_t + np.arange(6) * 0.4)
        data.ctrl[aids] = compensator.controls(data, target) if tuned else target
        mujoco.mj_step(model, data)
        # Same planar-base enforcement as MujocoBridge.step().
        data.qpos[:3] = 0; data.qvel[:3] = 0
        mujoco.mj_forward(model, data)
        for contact in data.contact:
            names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) for g in (contact.geom1, contact.geom2)]
            if 'floor' not in names:
                contacts.add(str(names))
        errors.append(target - data.qpos[qa])
        positions.append(data.qpos[qa].copy())
        forces.append(data.actuator_force[aids].copy())
    errors = np.array(errors); positions = np.array(positions); forces = np.array(forces)
    tail = round((duration - 0.5) / dt)
    overshoot = max(0., float(np.max(sign * (positions[round(0.5 / dt):, joint] - target[joint])))) if kind == 'step' else 0.
    return {
        'pose': pose, 'kind': kind, 'joint': joint + 1, 'sign': sign,
        'settled_max_error_rad': float(np.max(np.abs(errors[tail:]))),
        'hold_peak_to_peak_rad': float(np.max(np.ptp(positions[tail:], axis=0))),
        'tracking_rms_rad': float(np.sqrt(np.mean(errors[round(1/dt):] ** 2))),
        'overshoot_rad': overshoot,
        'force_limits_respected': bool(np.all(forces >= model.actuator_forcerange[aids, 0] - 1e-9) and np.all(forces <= model.actuator_forcerange[aids, 1] + 1e-9)),
        'nonfloor_contacts': sorted(contacts),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    comparison = {}
    for name, tuned in [('xml_without_bias', False), ('configured_with_bias', True)]:
        comparison[name] = [run(pose, tuned, kind) for pose in ['low', 'home', 'task_contact'] for kind in ['hold', 'sine']]
    steps = [run(pose, True, 'step', joint, sign) for pose in ['low', 'home', 'task_contact'] for joint in range(6) for sign in [-1, 1]]
    passed = all(r['force_limits_respected'] and not r['nonfloor_contacts'] and r['settled_max_error_rad'] < 0.001 and r['overshoot_rad'] < 0.005 for r in steps)
    passed &= all(r['force_limits_respected'] and not r['nonfloor_contacts'] for rows in comparison.values() for r in rows)
    for old, new in zip(comparison['xml_without_bias'], comparison['configured_with_bias']):
        passed &= new['tracking_rms_rad'] <= old['tracking_rms_rad'] + 1e-12
    report = {'passed': bool(passed), 'servo_parameters': yaml.safe_load((PACKAGE / 'config/arm_servo.yaml').read_text()), 'mujoco_version': mujoco.__version__, 'scope': 'complete model physics; no ROS/MPC, no contact task, no hardware validation', 'comparison': comparison, 'steps': steps}
    Path(args.output).write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'passed': bool(passed), 'max_step_error_rad': max(r['settled_max_error_rad'] for r in steps), 'max_step_overshoot_rad': max(r['overshoot_rad'] for r in steps), 'low_pose_comparison': {name: rows[:2] for name, rows in comparison.items()}}, indent=2))
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
