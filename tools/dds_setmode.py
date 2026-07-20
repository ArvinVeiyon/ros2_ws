import rclpy, sys, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, VehicleStatus

class SetMode(Node):
    def __init__(self, main, sub):
        super().__init__('dds_setmode')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.cb, qos)
        self.nav = None; self.arm = None
        self.main, self.subm = main, sub
    def cb(self, msg):
        self.nav, self.arm = msg.nav_state, msg.arming_state
    def send(self):
        m = VehicleCommand()
        m.timestamp = int(self.get_clock().now().nanoseconds/1000)
        m.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        m.param1, m.param2, m.param3 = 1.0, float(self.main), float(self.subm)
        m.target_system, m.target_component = 1, 1
        m.source_system, m.source_component = 1, 1
        m.from_external = True
        self.pub.publish(m)

def run(main, sub, label):
    n = SetMode(main, sub)
    t0=time.time()
    while time.time()-t0 < 3: rclpy.spin_once(n, timeout_sec=0.2)
    print(f"before: nav_state={n.nav} arming_state={n.arm}")
    n.send(); print(f"sent DO_SET_MODE main={main} sub={sub} ({label})")
    t0=time.time()
    while time.time()-t0 < 5: rclpy.spin_once(n, timeout_sec=0.2)
    print(f"after : nav_state={n.nav} arming_state={n.arm}")
    n.destroy_node()
    return n.nav

rclpy.init()
run(4, 11, "AUTO/EXTERNAL1 = AutoNav")
time.sleep(2)
run(4, 3, "AUTO/LOITER = Hold (restore)")
rclpy.shutdown()
