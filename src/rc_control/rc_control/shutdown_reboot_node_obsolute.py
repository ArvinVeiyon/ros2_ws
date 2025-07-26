#!/usr/bin/env python3
import os
from ruamel.yaml import YAML

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import InputRc
import subprocess
import time

class ShutdownRebootNode(Node):
    def __init__(self):
        super().__init__('shutdown_reboot_node')

        # ─── Locate and load our YAML ─────────────────────────────
        share_dir = get_package_share_directory('rc_control')
        cfg_path  = os.path.join(share_dir, 'config', 'shutdown_reboot_params.yaml')
        cfg_block = YAML().load(open(cfg_path))['ros__parameters']
        # ──────────────────────────────────────────────────────────

        # Declare scalar parameters only
        self.declare_parameter('channel_index', cfg_block.get('channel_index', 9))
        self.declare_parameter('tolerance',     cfg_block.get('tolerance',    15))
        self.declare_parameter('hold_time',     cfg_block.get('hold_time',    1.0))

        # Retrieve them
        ch1b       = self.get_parameter('channel_index').value
        self.ch    = ch1b - 1
        self.tol   = self.get_parameter('tolerance').value
        self.hold  = self.get_parameter('hold_time').value

        # Load rc_map dict directly from YAML
        rc_map = cfg_block.get('rc_map', {})
        self.shutdown_target = rc_map.get('shutdown')
        self.reboot_target   = rc_map.get('reboot')

        # Warn if missing
        if self.shutdown_target is None:
            self.get_logger().warn("rc_map['shutdown'] missing — shutdown disabled")
        if self.reboot_target is None:
            self.get_logger().warn("rc_map['reboot'] missing — reboot disabled")

        # State tracking
        self.shutdown_start = None
        self.reboot_start   = None
        self.did_shutdown   = False
        self.did_reboot     = False

        # Subscribe to RC input
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(
            InputRc, '/fmu/out/input_rc',
            self.cb_rc,
            qos_profile=qos
        )

        self.get_logger().info(
            f"[shutdown_reboot_node] CH{ch1b}, tol={self.tol}, "
            f"shutdown@{self.shutdown_target}, reboot@{self.reboot_target}, hold={self.hold}s"
        )

    def cb_rc(self, msg: InputRc):
        v   = int(msg.values[self.ch])
        now = time.time()

        # Shutdown logic
        if self.shutdown_target is not None and not self.did_shutdown:
            if abs(v - self.shutdown_target) <= self.tol:
                if self.shutdown_start is None:
                    self.shutdown_start = now
                    self.get_logger().info("Shutdown armed; hold stick at shutdown position")
                elif now - self.shutdown_start >= self.hold:
                    self.get_logger().warn("→ SHUTDOWN triggered")
                    subprocess.Popen(['sudo', 'shutdown', '-h', 'now'])
                    self.did_shutdown = True
            else:
                self.shutdown_start = None

        # Reboot logic
        if self.reboot_target is not None and not self.did_reboot:
            if abs(v - self.reboot_target) <= self.tol:
                if self.reboot_start is None:
                    self.reboot_start = now
                    self.get_logger().info("Reboot armed; hold stick at reboot position")
                elif now - self.reboot_start >= self.hold:
                    self.get_logger().warn("→ REBOOT triggered")
                    subprocess.Popen(['sudo', 'reboot'])
                    self.did_reboot = True
            else:
                self.reboot_start = None

def main(args=None):
    rclpy.init(args=args)
    node = ShutdownRebootNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
