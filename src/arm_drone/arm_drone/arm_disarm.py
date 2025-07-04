import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand

class ArmDisarmNode(Node):
    def __init__(self):
        super().__init__('arm_disarm_node')
        self.publisher_ = self.create_publisher(VehicleCommand, 'fmu/in/vehicle_command', 10)

    def send_command(self, arm: bool):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.param1 = 1.0 if arm else 0.0
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.publisher_.publish(msg)
        self.get_logger().info(f'{"Arm" if arm else "Disarm"} command sent')

def main(args=None):
    rclpy.init(args=args)
    node = ArmDisarmNode()
    try:
        while rclpy.ok():
            command = input("Press 'a' to arm, 'd' to disarm: ")
            if command == 'a':
                node.send_command(True)
            elif command == 'd':
                node.send_command(False)
            else:
                print("Invalid input. Please press 'a' to arm or 'd' to disarm.")
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
