#!/usr/bin/env python3
import os
from ruamel.yaml import YAML
import rclpy
from rclpy.node import Node
from px4_msgs.msg import InputRc

# ─ locate the YAML in your source tree ───────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.realpath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
YAML_PATH    = os.path.join(PROJECT_ROOT, 'config', 'rc_mapping.yaml')
# ─────────────────────────────────────────────────────────────────────

class ChannelCalibrator(Node):
    def __init__(self):
        super().__init__('channel_calibrator')
        self.yaml_path = YAML_PATH

        # load YAML (preserves comments & ordering)
        self.yaml = YAML()
        with open(self.yaml_path) as f:
            self.data = self.yaml.load(f)

        # subscribe to the RC topic
        self.latest = None
        self.create_subscription(
            InputRc,
            '/fmu/out/input_rc',
            lambda msg: setattr(self, 'latest', msg),
            10
        )

    def record_all(self):
        for ch_str, slots in sorted(self.data['rc_channels'].items(),
                                    key=lambda x: int(x[0])):
            idx = int(ch_str)
            self.get_logger().info(f"=== Calibrating channel {idx} ===")

            for name in slots.keys():
                if not name or name.lower() == 'null':
                    continue

                input(f" Center stick on channel {idx} to '{name}', then press [Enter] ")
                # wait for an RC message
                for _ in range(30):
                    rclpy.spin_once(self, timeout_sec=0.1)
                    if self.latest:
                        break

                new_pwm = int(self.latest.values[idx])
                self.data['rc_channels'][ch_str][name] = new_pwm
                self.get_logger().info(f"  Recorded {name} = {new_pwm}")

                # save immediately
                with open(self.yaml_path, 'w') as f:
                    self.yaml.dump(self.data, f)

        self.get_logger().info("All channels calibrated. Exiting.")

def main():
    rclpy.init()
    calib = ChannelCalibrator()
    calib.record_all()
    calib.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
