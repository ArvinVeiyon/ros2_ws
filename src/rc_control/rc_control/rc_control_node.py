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

class RCControlNode(Node):
    def __init__(self):
        super().__init__('rc_control_node')

        # ─── Load master rc_mapping.yaml ─────────────────────────────
        share_dir = get_package_share_directory('rc_control')
        cfg_path  = os.path.join(share_dir, 'config', 'rc_mapping.yaml')
        data      = YAML().load(open(cfg_path))

        # extract camera parameters
        cam_cfg = data['nodes']['camera_node_sw']['ros__parameters']
        self.cam_ch    = cam_cfg['channel_index'] - 1
        self.cam_tol   = cam_cfg['tolerance']
        crm = cam_cfg['rc_map']
        cams= cam_cfg['cameras']
        self.front_pwm  = crm.get('front')
        self.bottom_pwm = crm.get('bottom')
        self.split_pwm  = crm.get('split')
        self.front_dev  = cams.get('front')
        self.bottom_dev = cams.get('bottom')

        # extract shutdown/reboot parameters
        sys_cfg        = data['nodes']['shutdown_reboot_node']['ros__parameters']
        self.sys_ch    = sys_cfg['channel_index'] - 1
        self.sys_tol   = sys_cfg['tolerance']
        self.sys_hold  = sys_cfg['hold_time']
        srm = sys_cfg['rc_map']
        self.shutdown_pwm = srm.get('shutdown')
        self.reboot_pwm   = srm.get('reboot')

        # state trackers
        self.last_cam_state   = None
        self.shutdown_start   = None
        self.reboot_start     = None
        self.did_shutdown     = False
        self.did_reboot       = False

        # subscribe to RC input
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(InputRc, '/fmu/out/input_rc',
                                 self.cb_rc, qos_profile=qos)

        self.get_logger().info(
            f"[RCControl] cam→CH{self.cam_ch+1}@tol±{self.cam_tol}, "
            f"sys→CH{self.sys_ch+1}@tol±{self.sys_tol}, hold={self.sys_hold}s"
        )

    def cb_rc(self, msg: InputRc):
        vals = [int(v) for v in msg.values]

        # ----- camera switching logic -----
        pwm = vals[self.cam_ch]
        if   self.front_pwm  is not None and abs(pwm - self.front_pwm)  <= self.cam_tol:
            desired = (self.front_dev, None)
        elif self.bottom_pwm is not None and abs(pwm - self.bottom_pwm) <= self.cam_tol:
            desired = (self.bottom_dev, None)
        elif self.split_pwm  is not None and abs(pwm - self.split_pwm)  <= self.cam_tol:
            desired = (self.front_dev, self.bottom_dev)
        else:
            desired = None

        if desired and desired != self.last_cam_state:
            cmd = ['sudo', '/usr/local/bin/vision_config_manager', desired[0]]
            if desired[1]:
                cmd.append(desired[1])
            try:
                subprocess.run(cmd, check=True)
                self.get_logger().info(f"Camera → {desired}")
                self.last_cam_state = desired
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"Camera switch failed: {e}")

        # ----- shutdown/reboot logic -----
        pwm2 = vals[self.sys_ch]
        now  = time.time()

        # shutdown
        if (self.shutdown_pwm is not None
            and not self.did_shutdown
            and abs(pwm2 - self.shutdown_pwm) <= self.sys_tol):
            if self.shutdown_start is None:
                self.shutdown_start = now
                self.get_logger().info("Shutdown armed (hold stick)")
            elif now - self.shutdown_start >= self.sys_hold:
                self.get_logger().warn("→ SHUTDOWN triggered")
                subprocess.Popen(['sudo','shutdown','-h','now'])
                self.did_shutdown = True
        else:
            self.shutdown_start = None

        # reboot
        if (self.reboot_pwm is not None
            and not self.did_reboot
            and abs(pwm2 - self.reboot_pwm) <= self.sys_tol):
            if self.reboot_start is None:
                self.reboot_start = now
                self.get_logger().info("Reboot armed (hold stick)")
            elif now - self.reboot_start >= self.sys_hold:
                self.get_logger().warn("→ REBOOT triggered")
                subprocess.Popen(['sudo','reboot'])
                self.did_reboot = True
        else:
            self.reboot_start = None

def main(args=None):
    rclpy.init(args=args)
    node = RCControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
