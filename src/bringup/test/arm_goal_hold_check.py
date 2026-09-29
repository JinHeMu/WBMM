#!/usr/bin/env python3
"""MuJoCo regression: repeated current-base goals must not ratchet the arm down.

Run in an unused nonzero ROS_DOMAIN_ID with ROS_LOCALHOST_ONLY=1 after sourcing
install/setup.bash. Outputs include observations, commands, references and the
base-relative tool height. Never starts hardware.
"""
import argparse
import os,sys,time,json,signal,subprocess
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray,String
from geometry_msgs.msg import PoseStamped
from ocs2_msgs.msg import MpcTargetTrajectories
from wbmm_planner_ros.msg import WholeBodyTrajectory
from tf2_ros import Buffer,TransformListener
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', default='/tmp/wbmm_arm_goal_hold')
parser.add_argument('--launch-arg', action='append', default=[])
parser.add_argument('--record-only', action='store_true', help='Record a failing baseline without asserting stability')
args = parser.parse_args()
if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or int(os.environ.get('ROS_DOMAIN_ID', '0')) == 0:
    parser.error('Use ROS_LOCALHOST_ONLY=1 and a dedicated nonzero ROS_DOMAIN_ID')
out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
os.environ['CYCLONEDDS_URI']='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
f=(out/'launch.log').open('w')
p=subprocess.Popen(['ros2','launch','tracer_jaka_bringup','wbmm_tracking.launch.py','esdf_file:=maps/map1/site_remani.npz','viewer:=false','use_rviz:=false',*args.launch_arg],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();node=rclpy.create_node('arm_goal_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
s={};hist=[];step='startup'
def record(k,m):
 s[k]=m
 if k=='q':
  order=[m.name.index('joint_'+str(i)) for i in range(1,7)]; hist.append(dict(t=node.get_clock().now().nanoseconds/1e9,step=step,q=[m.position[i] for i in order]))
for cls,topic,key,qos in [(JointState,'/joint_states','q',qos_profile_sensor_data),(Odometry,'/wheel/odometry','odom',qos_profile_sensor_data),(Float64MultiArray,'/arm_controller/commands','cmd',10),(MpcTargetTrajectories,'/mobile_manipulator_whole_body_target','ref',10),(WholeBodyTrajectory,'/wbmm/whole_body_trajectory','plan',10),(String,'/wbmm/planning/status','status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]:
 node.create_subscription(cls,topic,lambda m,k=key:record(k,m),qos)
pub=node.create_publisher(PoseStamped,'/goal_pose',10);buf=Buffer();listener=TransformListener(buf,node)
def wait(seconds):
 until=time.monotonic()+seconds
 while time.monotonic()<until:
  if p.poll() is not None:raise RuntimeError('launch exited')
  rclpy.spin_once(node,timeout_sec=.02)
def snapshot(label):
 q=s['q'];values=[q.position[q.name.index('joint_'+str(i))] for i in range(1,7)]
 r=dict(label=label,q=values,command=list(s['cmd'].data),status=s['status'].data)
 if 'ref' in s:r['ref']=list(s['ref'].state_trajectory[0].value)
 if 'plan' in s:r['plan_first']=list(s['plan'].joint_positions[:6]);r['plan_last']=list(s['plan'].joint_positions[-6:])
 r['z']=buf.lookup_transform('base_link','tool0',rclpy.time.Time()).transform.translation.z
 print(json.dumps(r),flush=True);return r
report=[]
try:
 for _ in range(120):
  wait(1)
  if all(k in s for k in ['q','odom','cmd','status']) and buf.can_transform('base_link','tool0',rclpy.time.Time()):break
 wait(12);report.append(snapshot('initial_settled'))
 wait(8);report.append(snapshot('no_goal_wait'))
 for i in range(4):
  step='goal_'+str(i+1)
  g=PoseStamped();g.header.frame_id='map';g.header.stamp=node.get_clock().now().to_msg();g.pose=s['odom'].pose.pose
  pub.publish(g);wait(8);report.append(snapshot(step))
 if not args.record_only:
  # The unchanged MPC joint-limit barrier leaves about 1 mm/goal of upward
  # bias. Accept that small residual, but reject the original ~20 mm sag.
  for before, after in zip(report[1:], report[2:]):
   assert after['status'] == 'SUCCEEDED', after['status']
   assert abs(after['z'] - before['z']) < 0.003, (before, after)
   assert np.max(np.abs(np.array(after['q']) - before['q'])) < 0.005, (before, after)
  assert abs(report[-1]['z'] - report[1]['z']) < 0.006, report
  print('PASS: repeated goals preserve arm posture within the accepted small MPC bias', flush=True)
finally:
 (out/'report.json').write_text(json.dumps(report,indent=2));(out/'joints.json').write_text(json.dumps(hist))
 if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 try:p.wait(timeout=15)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 f.close();node.destroy_node();rclpy.shutdown()
