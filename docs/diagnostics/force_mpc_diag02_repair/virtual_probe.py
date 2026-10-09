import sys, types, threading, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/bringup/test'))
import rclpy
from geometry_msgs.msg import WrenchStamped
class Keyboard:
    def __init__(self):
        self.node=rclpy.create_node('headless_virtual_force_test')
        self.pub=self.node.create_publisher(WrenchStamped,'/whole_body_force_control/virtual_wrench_command',10)
        self.force=[0.,0.,0.]
        threading.Thread(target=self.send,daemon=True).start()
    def send(self):
        while rclpy.ok():
            msg=WrenchStamped();msg.header.frame_id='tool0'
            msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z=self.force
            try:self.pub.publish(msg)
            except Exception:break
            time.sleep(.02)
    def focus(self):pass
    def key(self,name,pressed):
        mapping={'w':(0,5.),'s':(0,-5.),'a':(1,5.),'d':(1,-5.),'r':(2,5.),'f':(2,-5.)}
        axis,value=mapping[name];self.force[axis]=value if pressed else 0.
stub=types.ModuleType('keyboard_test_events');stub.Keyboard=Keyboard
sys.modules['keyboard_test_events']=stub
from force_mpc_arm_margin_probe import main
raise SystemExit(main())
