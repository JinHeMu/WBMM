#!/usr/bin/env python3
"""Isolated planner/bridge regression; starts no controller or hardware.
Run in a dedicated ROS_DOMAIN_ID after sourcing the workspace, with URDF and
ESDF paths as the two positional arguments. Requires a map with the origin and
straight 0.8 m test segment collision-free for the zero arm configuration.
"""
import copy
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from ocs2_msgs.msg import MpcObservation, MpcTargetTrajectories
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from std_msgs.msg import Header, String
from tf2_ros import StaticTransformBroadcaster
from wbmm_planning_msgs.msg import WholeBodyTrajectory, WholeBodyGoal


def main():
    urdf, esdf = sys.argv[1:3]
    processes, logs = [], []
    rclpy.init()
    node = rclpy.create_node('migration_regression')
    trajectories, targets, statuses = [], [], []
    node.create_subscription(String, "/wbmm/planning/status", lambda msg: statuses.append(msg.data), 10)
    node.create_subscription(WholeBodyTrajectory, '/wbmm/whole_body_trajectory', trajectories.append, 10)
    node.create_subscription(MpcTargetTrajectories, '/mobile_manipulator_whole_body_target', targets.append, 10)
    odom_pub = node.create_publisher(Odometry, '/wheel/odometry', qos_profile_sensor_data)
    joints_pub = node.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
    obs_pub = node.create_publisher(MpcObservation, '/mobile_manipulator_mpc_observation', qos_profile_sensor_data)
    goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 10)
    cancel_pub = node.create_publisher(Header, "/wbmm/planning/cancel", 10)
    whole_goal_pub = node.create_publisher(WholeBodyGoal, "/wbmm/whole_body_goal", 10)
    traj_pub = node.create_publisher(WholeBodyTrajectory, '/wbmm/whole_body_trajectory', 10)
    client = node.create_client(SetBool, '/wbmm_reference_bridge/set_reference_enabled')
    broadcaster = StaticTransformBroadcaster(node)
    names = [f'joint_{i}' for i in range(1, 7)]
    odom_frame = 'map'
    publish_feedback = True
    publish_observation = True
    feedback_lead = 0.0
    epoch = time.monotonic()

    def tick(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            stamp = (node.get_clock().now()+Duration(seconds=feedback_lead)).to_msg()
            if publish_feedback:
                odom = Odometry(); odom.header.stamp = stamp
                odom.header.frame_id = odom_frame
                odom.child_frame_id = 'base_footprint'
                odom.pose.pose.orientation.w = 1.0
                if odom_frame == 'odom':
                    odom.pose.pose.position.x = -2.0
                    odom.pose.pose.position.y = 1.0
                    odom.pose.pose.orientation.z = -math.sqrt(0.5)
                    odom.pose.pose.orientation.w = math.sqrt(0.5)
                odom_pub.publish(odom)
                joints = JointState(); joints.header.stamp = stamp
                joints.name = names; joints.position = [0.0]*6; joints.velocity = [0.0]*6
                joints_pub.publish(joints)
            if publish_observation:
                obs = MpcObservation(); obs.time = time.monotonic() - epoch
                obs.state.value = [-2., 1., -math.pi/2] + [0.]*6
                obs.input.value = [0.]*8
                obs_pub.publish(obs)
            rclpy.spin_once(node, timeout_sec=0.02)

    def goal(frame='map', invalid=False):
        msg = PoseStamped(); msg.header.frame_id = frame
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.position.x = 0.8
        msg.pose.orientation.w = 0.0 if invalid else 1.0
        goal_pub.publish(msg)

    def until(predicate, timeout=15):
        end = time.monotonic()+timeout
        while not predicate() and time.monotonic()<end:
            tick(0.05)
        assert predicate(), 'runtime condition timed out'

    def owner(value):
        assert client.wait_for_service(timeout_sec=2)
        req = SetBool.Request(); req.data = value
        future = client.call_async(req)
        until(future.done, 3)
        assert future.result().success
        tick(0.1)

    try:
        for pkg, exe, params in [
            ('wbmm_planner_ros', 'wbmm_planner_ros_node',
             ['urdf_file:='+urdf, 'esdf_file:='+esdf, 'world_frame:=map']),
            ('wbmm_reference_bridge', 'wbmm_reference_bridge_node', ['world_frame:=odom'])]:
            log = open(pkg+'.log', 'w'); logs.append(log)
            binary = Path(get_package_prefix(pkg))/'lib'/pkg/exe
            command = [str(binary), '--ros-args']
            for param in params: command += ['-p', param]
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        tick(2)
        assert all(p.poll() is None for p in processes), 'node failed to start'
        goal(invalid=True); tick(0.3)
        assert not trajectories, 'invalid quaternion accepted'
        publish_feedback = False; tick(0.7); goal(); tick(0.3)
        assert not trajectories, 'stale state accepted'
        publish_feedback = True; tick(0.3); goal()
        until(lambda: bool(trajectories))
        tick(0.3)
        assert all(all(abs(v)<1e-8 for inp in target.input_trajectory for v in inp.value)
                   for target in targets), 'navigation reference published without map/odom TF'
        targets.clear()
        print('PASS invalid quaternion, stale state, missing TF rejection', flush=True)
        tf = TransformStamped(); tf.header.stamp = node.get_clock().now().to_msg()
        tf.header.frame_id = 'map'; tf.child_frame_id = 'odom'
        tf.transform.translation.x = 1.0; tf.transform.translation.y = 2.0
        tf.transform.rotation.z = math.sqrt(0.5); tf.transform.rotation.w = math.sqrt(0.5)
        broadcaster.sendTransform(tf)
        until(lambda: bool(targets))
        sample = targets[-1].state_trajectory[0].value
        assert abs(sample[0]+2.0)<0.01, sample
        assert -0.1 <= sample[1] <= 1.05, sample
        assert abs(sample[2]+math.pi/2)<0.05, sample
        print('PASS map-to-odom translation and 90-degree rotation', flush=True)
        # Feed the planner the same physical state expressed in odom.
        odom_frame='odom'; tick(0.5)
        before=len(trajectories); goal(); until(lambda:len(trajectories)>before)
        latest=trajectories[-1]
        assert abs(latest.base_x[0])<1e-6 and abs(latest.base_y[0])<1e-6
        print('PASS odom-to-map planning state conversion', flush=True)
        owner(False); owner(True)
        targets.clear(); tick(0.3)
        assert not targets, 'old trajectory resumed after ownership handoff'
        malformed=copy.deepcopy(latest)
        malformed.joint_names=list(reversed(names))
        traj_pub.publish(malformed); tick(0.3)
        assert not targets, 'permuted joint order accepted'
        valid=copy.deepcopy(latest); valid.header.stamp=node.get_clock().now().to_msg()
        traj_pub.publish(valid); until(lambda: bool(targets))
        print('PASS ownership invalidation and joint-order rejection', flush=True)
        publish_observation=False; tick(0.7); targets.clear(); tick(0.2)
        assert not targets, 'expired observation kept publishing'
        publish_observation=True; tick(0.3)
        assert not targets, 'old plan resumed after observation expiry'
        print('PASS observation expiry invalidates old plan', flush=True)
        # Cancellation installs a measured-state hold and prevents delayed old
        # navigation messages from reactivating a trajectory.
        delayed_cancel=Header(); delayed_cancel.stamp=node.get_clock().now().to_msg()
        tick(0.05)
        valid.header.stamp=node.get_clock().now().to_msg()
        traj_pub.publish(valid); until(lambda: bool(targets))
        tick(0.1)
        targets.clear();cancel_pub.publish(delayed_cancel);tick(0.2)
        assert any(abs(inp.value[0])>1e-3 for target in targets for inp in target.input_trajectory), \
            'delayed cancellation replaced a newer plan with a hold'
        print('PASS cross-topic delayed cancellation ordering', flush=True)
        cancel=Header(); cancel.stamp=node.get_clock().now().to_msg()
        cancel_pub.publish(cancel); tick(0.2); targets.clear()
        traj_pub.publish(valid); tick(0.3)
        assert targets, 'cancellation did not install a measured hold'
        for target in targets:
            for state, inp in zip(target.state_trajectory, target.input_trajectory):
                assert abs(state.value[0]+2)<1e-5 and abs(state.value[1]-1)<1e-5
                assert all(abs(v)<1e-8 for v in inp.value)
        print('PASS cancellation holds measured state and rejects delayed old plan', flush=True)
        before=len(trajectories);statuses.clear();goal()
        until(lambda:any(value == 'PLANNING' for value in statuses))
        cancel=Header();cancel.stamp=node.get_clock().now().to_msg()
        cancel_pub.publish(cancel);tick(1.0)
        cutoff=cancel.stamp.sec*10**9+cancel.stamp.nanosec
        assert all(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec<=cutoff
                   for msg in trajectories[before:]), 'cancelled planning result was published'
        print('PASS in-flight planning cancellation', flush=True)
        # Mimic feedback reaching the subscriber slightly before /clock. The
        # executor must use the same small future tolerance as the callbacks.
        feedback_lead=0.02
        tick(0.1)
        # Named full-body goal is accepted in arbitrary message joint order;
        # published trajectory remains in the configured controller order.
        whole=WholeBodyGoal();whole.header.frame_id='map'
        whole.header.stamp=node.get_clock().now().to_msg()
        whole.base_pose.orientation.w=1.0
        whole.joint_names=list(reversed(names))
        whole.joint_positions=[0.2]+[0.0]*5
        before=len(trajectories);whole_goal_pub.publish(whole)
        until(lambda:len(trajectories)>before)
        latest=trajectories[-1]
        assert latest.joint_names==names
        assert abs(latest.joint_positions[-1]-0.2)<1e-6
        assert all(abs(v)<1e-6 for v in latest.joint_positions[:6])
        print('PASS full-body goal, initial joints, and small feedback clock skew', flush=True)
        print('PASS isolated ROS planner/bridge regression', flush=True)
    finally:
        for p in processes:
            if p.poll() is None: os.killpg(p.pid, signal.SIGINT)
        for p in processes:
            try: p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL); p.wait()
        for log in logs: log.close()
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
