#!/usr/bin/env python3
"""Run identical keyboard histories in isolated MuJoCo launches, then clean up.

Source ROS/workspace first; invoke /usr/bin/python3 -s. This runner refuses real
backends: its launch arguments always specify backend:=sim fake_wrench:=true.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', required=True)
    parser.add_argument('--cases', default='baseline,sigma,sigma_yoshikawa')
    parser.add_argument('--sequence', default='f:8,zero:4,r:6,zero:4')
    parser.add_argument('--domain', type=int, default=68)
    parser.add_argument('--integration-probe', action='store_true',
                        help='run the existing C++ force/feedback-loss/recovery regression instead of keyboard probe')
    args = parser.parse_args()
    folder = Path(args.directory).resolve()
    probe = Path(__file__).with_name('force_mpc_arm_margin_probe.py')
    results = []
    for name in args.cases.split(','):
        task = folder / f'{name}.info'
        if not task.is_file():
            raise FileNotFoundError(task)
        env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), ROS_LOG_DIR=f'/tmp/wbmm_arm_margin_{name}_ros')
        cmd = ['ros2', 'launch', 'tracer_jaka_bringup', 'force_mpc.launch.py',
               'backend:=sim', 'fake_wrench:=true',
               f'keyboard_wrench:={"false" if args.integration_probe else "true"}',
               'use_rviz:=false', f'task_file:={task}']
        print('LAUNCH', name, flush=True)
        with (folder / f'{name}_launch.log').open('w') as launch_log:
            launch = subprocess.Popen(cmd, env=env, stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                with (folder / f'{name}_probe.log').open('w') as probe_log:
                    probe_cmd = [sys.executable, '-s', str(probe), '--name', name,
                                 '--sequence', args.sequence, '--output', str(folder / f'{name}.json')]
                    if args.integration_probe:
                        probe_cmd = ['ros2', 'run', 'whole_body_force_control', 'force_mpc_integration_probe',
                                     '--ros-args', '-p', f'report:={folder / (name + ".json")}']
                    run = subprocess.run(probe_cmd,
                                         env=env, stdout=probe_log, stderr=subprocess.STDOUT, timeout=360)
                    results.append((name, run.returncode))
                    print('FINISHED', name, 'exit', run.returncode, flush=True)
            finally:
                if launch.poll() is None:
                    os.killpg(launch.pid, signal.SIGINT)
                    try:
                        launch.wait(timeout=12)
                    except subprocess.TimeoutExpired:
                        os.killpg(launch.pid, signal.SIGKILL)
                        launch.wait(timeout=5)
    print(results, flush=True)
    return int(any(code for _, code in results))


if __name__ == '__main__':
    raise SystemExit(main())
