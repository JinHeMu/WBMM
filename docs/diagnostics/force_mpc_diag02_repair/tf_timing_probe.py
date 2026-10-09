import os,sys,time,signal,subprocess,json
from pathlib import Path
import yaml,numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
out=Path(__file__).resolve().parent;root=out.parents[2]
urdf=(root/'src/robotics/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf').read_text()
reports=[]
for frequency in [20.,125.,250.]:
    env=dict(os.environ,ROS_DOMAIN_ID='98',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR='/tmp/wbmm_force_repair_tf_logs')
    os.environ.update({k:env[k] for k in ['ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','ROS_LOG_DIR']})
    config=out/f'tf_{int(frequency)}.yaml'
    config.write_text(yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{'robot_description':urdf,'publish_frequency':frequency}},'force_sensor_processor':{'ros__parameters':{'force_sensor.require_stamped_wrench':True,'force_sensor.tare_on_start':False,'force_sensor.tare_after_compensation':False}}}))
    rsp=['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args','--params-file',str(config)]
    sensor=['ros2','run','whole_body_force_control','force_sensor_processor_node','--ros-args']
    for f in ['config/common/force_control.yaml','config/real/force_control.yaml','config/common/force_mpc.yaml','config/real/force_mpc_calibration.yaml']:
        sensor+=['--params-file',str(root/'src/bringup'/f)]
    sensor+=['--params-file',str(config)]
    processes=[]
    with (out/f'tf_{int(frequency)}_launch.log').open('w') as log:
        try:
            for cmd in [rsp,sensor]:processes.append(subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
            rclpy.init();node=rclpy.create_node('synthetic_joint_wrench_tf_probe')
            jsp=node.create_publisher(JointState,'/joint_states',10)
            wp=node.create_publisher(WrenchStamped,'/fts_broadcaster/wrench',10)
            stamps=[];lag=[];states=[];sent=[0]
            def tf(m):
                for t in m.transforms:
                    if t.child_frame_id=='Link_1':stamps.append(t.header.stamp.sec+t.header.stamp.nanosec*1e-9)
            def processed(m):lag.append((node.get_clock().now().nanoseconds-(m.header.stamp.sec*10**9+m.header.stamp.nanosec))*1e-6)
            subs=[node.create_subscription(TFMessage,'/tf',tf,100),node.create_subscription(WrenchStamped,'/whole_body_force_control/processed_wrench',processed,qos_profile_sensor_data),node.create_subscription(String,'/whole_body_force_control/force_sensor_states',lambda m:states.append(m.data),10)]
            def send():
                j=JointState();j.header.stamp=node.get_clock().now().to_msg();j.name=[f'joint_{i}' for i in range(1,7)];j.position=[0.,1.,1.,1.14,1.57,.785];j.velocity=[0.]*6;jsp.publish(j)
                w=WrenchStamped();w.header.stamp=j.header.stamp;w.header.frame_id='jk_se_vi_200_link';wp.publish(w);sent[0]+=1
            # Fill discovery before introducing input; no backend/controller is launched.
            until=time.monotonic()+2
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
            timer=node.create_timer(.008,send);until=time.monotonic()+10
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.002)
            gaps=np.diff(stamps)*1000
            report={'publish_frequency':frequency,'sent':sent[0],'processed':len(lag),'states':sorted(set(states)),'tf_count':len(stamps),'tf_gap_ms_p50':float(np.median(gaps)) if len(gaps) else None,'tf_gap_ms_p95':float(np.percentile(gaps,95)) if len(gaps) else None,'processed_age_ms_p50':float(np.median(lag)) if lag else None,'processed_age_ms_p95':float(np.percentile(lag,95)) if lag else None}
            reports.append(report);print(json.dumps(report),flush=True)
            node.destroy_node();rclpy.shutdown()
        finally:
            for p in processes:
                if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
            for p in processes:
                try:p.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
(out/'tf_timing.json').write_text(json.dumps(reports,indent=2)+'\n')
if not reports[-1]['processed'] or any(x.startswith('FAULT') for x in reports[-1]['states']):raise SystemExit(1)
