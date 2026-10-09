import os, sys, subprocess, signal
from pathlib import Path
folder=Path(__file__).resolve().parent
for name in ['sigma_010','sigma_020','sigma_050']:
    env=dict(os.environ,ROS_DOMAIN_ID='97',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR='/tmp/wbmm_force_repair_sim_logs')
    cmd=['ros2','launch','tracer_jaka_bringup','force_mpc.launch.py','backend:=sim','fake_wrench:=true','keyboard_wrench:=false','viewer:=false','use_rviz:=false',f'task_file:={folder/(name+".info")}']
    with (folder/(name+'_launch.log')).open('w') as log:
        process=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            with (folder/(name+'_probe.log')).open('w') as out:
                probe=subprocess.run([sys.executable,'-s',str(folder/'virtual_probe.py'),'--name',name,'--output',str(folder/(name+'.json')),'--sequence','f:8,zero:4,r:6,zero:4'],env=env,stdout=out,stderr=subprocess.STDOUT,timeout=200)
            print(name,probe.returncode,flush=True)
        finally:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);process.wait()
    if probe.returncode:raise SystemExit(probe.returncode)
