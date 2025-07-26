#!/usr/bin/env python3
import os
from ruamel.yaml import YAML

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import InputRc
import subprocess

class CameraNodeSw(Node):
    def __init__(self):
        super().__init__('camera_node_sw')

        # ─── Locate and load our YAML ─────────────────────────────
        share_dir = get_package_share_directory('rc_control')
        cfg_path  = os.path.join(share_dir, 'config', 'camera_sw_params.yaml')
        cfg_block = YAML().load(open(cfg_path))['ros__parameters']
        # ──────────────────────────────────────────────────────────

        # Declare scalar parameters
        self.declare_parameter('channel_index', cfg_block.get('channel_index', 7))
        self.declare_parameter('tolerance',     cfg_block.get('tolerance',    50))

        # Read them back
        ch1b   = self.get_parameter('channel_index').value
        self.ch = ch1b - 1
        self.tol= self.get_parameter('tolerance').value

        # Directly pull dicts from YAML (not via declare_parameter)
        rc_map = cfg_block.get('rc_map', {})
        cams   = cfg_block.get('cameras', {})

        # Extract targets & devices
        self.front_target  = rc_map.get('front')
        self.bottom_target = rc_map.get('bottom')
        self.split_target  = rc_map.get('split')
        self.front_dev     = cams.get('front')
        self.bottom_dev    = cams.get('bottom')

        # Warn if mis‑configured
        if self.front_target  is None: self.get_logger().warn("rc_map['front'] missing")
        if self.bottom_target is None: self.get_logger().warn("rc_map['bottom'] missing")
        if self.split_target  is None: self.get_logger().warn("rc_map['split'] missing")
        if not self.front_dev:         self.get_logger().warn("cameras['front'] missing")
        if not self.bottom_dev:        self.get_logger().warn("cameras['bottom'] missing")

        # Subscribe to RC input
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(
            InputRc, '/fmu/out/input_rc',
            self.listener_callback,
            qos_profile=qos
        )

        self.last_state = None
        self.get_logger().info(
            f"camera_node_sw: CH{ch1b}, tol={self.tol}, "
            f"front={self.front_target}, bottom={self.bottom_target}, split={self.split_target}"
        )

    def switch_camera(self, primary, secondary=None):
        if not primary:
            return
        cmd = ['sudo', '/usr/local/bin/vision_config_manager', primary]
        if secondary:
            cmd.append(secondary)
        try:
            subprocess.run(cmd, check=True)
            self.last_state = (primary, secondary)
            self.get_logger().info(
                f"Switched to {primary}" + (f" + {secondary}" if secondary else "")
            )
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Camera switch failed: {e}")

    def listener_callback(self, msg: InputRc):
        val = int(msg.values[self.ch])

        # Match each target ± tolerance
        if self.front_target is not None and abs(val - self.front_target) <= self.tol:
            desired = (self.front_dev, None)
        elif self.bottom_target is not None and abs(val - self.bottom_target) <= self.tol:
            desired = (self.bottom_dev, None)
        elif self.split_target is not None and abs(val - self.split_target) <= self.tol:
            desired = (self.front_dev, self.bottom_dev)
        else:
            return

        if desired != self.last_state:
            self.get_logger().info(f"Detected PWM={val}; switching cameras")
            self.switch_camera(*desired)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNodeSw()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
