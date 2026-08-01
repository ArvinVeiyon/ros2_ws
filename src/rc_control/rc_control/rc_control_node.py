#!/usr/bin/env python3
import os
import threading
from ruamel.yaml import YAML

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import InputRc
import subprocess
import time

VISION_CONFIG_MANAGER = '/usr/local/bin/vision_config_manager'

# A failing camera switch is retried a few times in case the failure is transient
# (device mid-rebind, briefly busy), then given up on until the operator actually
# moves the switch. It is NEVER retried per RC message -- see cb_rc.
CAM_MAX_ATTEMPTS = 3
CAM_RETRY_BASE_S = 0.5
CAM_TIMEOUT_S    = 10

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
        # cam_attempted is the selection we most recently STARTED work on. It is
        # latched the instant a change is seen, not when the switch succeeds --
        # see cb_rc for why. cam_applied records what actually took effect.
        self.cam_attempted    = None
        self.cam_applied      = None
        self._cam_lock        = threading.Lock()
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

    def _start_cam_switch(self, target):
        threading.Thread(target=self._cam_worker, args=(target,), daemon=True).start()

    def _cam_worker(self, target):
        """Apply a camera selection off the RC callback thread.

        This runs in its own thread so that a slow, hung or repeatedly failing
        vision_config_manager cannot delay the shutdown/reboot stick detection,
        which shares cb_rc with the camera logic.
        """
        cmd = ['sudo', VISION_CONFIG_MANAGER, target[0]]
        if target[1]:
            cmd.append(target[1])

        with self._cam_lock:            # serialise rapid switch flips
            delay = CAM_RETRY_BASE_S
            last  = None
            for attempt in range(1, CAM_MAX_ATTEMPTS + 1):
                if self.cam_attempted != target:
                    return              # operator moved on; abandon this target
                try:
                    subprocess.run(cmd, check=True, timeout=CAM_TIMEOUT_S,
                                   stdout=subprocess.DEVNULL,
                                   stderr=subprocess.PIPE)
                    self.cam_applied = target
                    self.get_logger().info(f"Camera → {target}")
                    return
                except subprocess.CalledProcessError as e:
                    last = (e.stderr or b'').decode(errors='replace').strip() or e
                except Exception as e:
                    last = e
                if attempt < CAM_MAX_ATTEMPTS:
                    time.sleep(delay)
                    delay *= 2

            # Give up on THIS target, logged once rather than at the RC rate.
            self.get_logger().error(
                f"Camera switch to {target} failed after {CAM_MAX_ATTEMPTS} attempts; "
                f"not retrying. Cycle the RC switch away and back to try again. "
                f"Last error: {last}")

    def cb_rc(self, msg: InputRc):
        vals = [int(v) for v in msg.values]
        n_ch = min(int(msg.channel_count), len(vals))
        if self.cam_ch >= n_ch or self.sys_ch >= n_ch:
            return                      # RX reporting fewer channels than mapped

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

        # Latch the ATTEMPT, not the success, and do the work off-thread.
        # Latching on success alone meant a PERMANENT failure -- e.g. /dev/video0
        # absent because vision_streaming is deliberately stopped for testing --
        # was retried on every RC message at 95 Hz forever: 2181 failures in ten
        # minutes, ~100% of a core, and because the spawn was blocking it also
        # delayed the shutdown/reboot checks below. The camera switch's resting
        # detent requests the front camera, so this was the DEFAULT state during
        # any perception test, not an accident.
        if desired is None:
            # Switch sits between detents. Clear the latch so that returning to a
            # position re-attempts it -- cycling the switch is the natural "try
            # again" gesture once a camera is actually plugged in or streaming.
            self.cam_attempted = None
        elif desired != self.cam_attempted:
            self.cam_attempted = desired
            self._start_cam_switch(desired)

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
