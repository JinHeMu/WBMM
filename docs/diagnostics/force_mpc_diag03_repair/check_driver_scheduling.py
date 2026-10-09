"""Driver simulated mode only: never opens a CAN connection."""
import json, os, signal, subprocess, time
from pathlib import Path
import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry

folder = Path(__file__).parent
log = (folder / 'driver_scheduling.log').open('w')
child = subprocess.Popen([
    str(Path.cwd() / 'install/tracer_base/lib/tracer_base/tracer_base_node'), '--ros-args',
    '-r', '__node:=tracer_base_node', '-p', 'simulated_robot:=true',
    '-p', 'state_publish_rate:=1.0', '-p', 'publish_command_timing:=true',
    '-p', 'port_name:=NO_CAN_DIAGNOSTIC'], stdout=log, stderr=subprocess.STDOUT)
rclpy.init()
node = rclpy.create_node('driver_scheduling_probe')
sent, received, dispatched, odom = {}, {}, {}, []
def on_event(msg, dest):
    index = round(msg.twist.linear.x / .0001)
    dest[index] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
pub = node.create_publisher(Twist, '/cmd_vel', 1)
subs = [node.create_subscription(TwistStamped, '/tracer_base_node/command_received', lambda m:on_event(m, received), 100),
        node.create_subscription(TwistStamped, '/tracer_base_node/command_dispatched', lambda m:on_event(m, dispatched), 100),
        node.create_subscription(Odometry, '/wheel/odometry', lambda m:odom.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9), 10)]
report = {}
try:
    deadline = time.monotonic()+10
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=.01)
        if time.monotonic()>deadline: raise RuntimeError('driver startup timeout')
    start=time.monotonic(); next_send=start
    for i in range(1,301):
        while time.monotonic()<next_send:
            rclpy.spin_once(node, timeout_sec=.001)
        msg=Twist();msg.linear.x=i*.0001
        sent[i]=node.get_clock().now().nanoseconds * 1e-9
        pub.publish(msg);next_send=start+i*.008
    until=time.monotonic()+.2
    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.002)
    matched=sorted(set(sent)&set(received)&set(dispatched))
    latency=np.array([received[i]-sent[i] for i in matched])*1000
    dispatch=np.array([dispatched[i]-received[i] for i in matched])*1000
    report={'simulation_only':True,'state_publish_rate_hz':1,'command_rate_hz':125,
            'sent':len(sent),'received':len(received),'matched':len(matched),
            'receive_latency_ms_p50_p95_max':np.percentile(latency,[50,95,100]).tolist(),
            'callback_dispatch_ms_p50_p95_max':np.percentile(dispatch,[50,95,100]).tolist(),
            'odom_samples':len(odom)}
    assert len(matched)>200,report
    assert np.percentile(latency,95)<100,report
finally:
    child.send_signal(signal.SIGINT)
    try:child.wait(timeout=5)
    except subprocess.TimeoutExpired:child.kill();child.wait()
    report['shutdown_returncode']=child.returncode
    node.destroy_node();rclpy.shutdown();log.close()
    (folder/'driver_scheduling.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
assert child.returncode==0
