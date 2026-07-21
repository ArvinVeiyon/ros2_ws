#!/bin/bash
# Install systemd units for the Vind-Roz rover autonav stack.
# Written 2026-07-21. Run with: sudo bash install_rover_units.sh
#
# Units created:
#   rover-camera.service      Orbbec Gemini 336L wrapper (IMU enabled)   [enabled]
#   rover-scan.service        depth -> /scan bridge + base_link TF       [enabled]
#   rover-odometry.service    VESC ERPM -> /odom + odom->base_link TF    [enabled]
#   rover-autonav-mode.service  px4_ros2 custom mode "AutoNav"           [enabled]
#   rover-ekf-bridge.service  /odom -> EKF2 external vision velocity     [NOT enabled]
#
# rover-ekf-bridge is deliberately NOT enabled at boot. With the wheels off the
# ground it feeds EKF2 velocity the vehicle is not actually achieving, which drives
# a self-sustaining front/back limit cycle in any closed-loop mode (see
# project_rover_autonav memory). Start it by hand once the rover is on the floor:
#     sudo systemctl start rover-ekf-bridge
set -euo pipefail

if [[ $EUID -ne 0 ]]; then echo "run me with sudo"; exit 1; fi

ROS_SETUP=/opt/ros/jazzy/setup.bash
WS_SETUP=/home/roz/ros2_ws/install/setup.bash

for f in "$ROS_SETUP" "$WS_SETUP"; do
  [[ -f $f ]] || { echo "missing $f -- build the workspace first"; exit 1; }
done

# $1 = unit name, $2 = description, $3 = extra After=, $4 = command
mkunit() {
  cat > "/etc/systemd/system/$1" <<EOF
[Unit]
Description=$2
After=network.target $3
StartLimitIntervalSec=300
StartLimitBurst=10

[Service]
Type=simple
User=roz
Group=roz
Environment=HOME=/home/roz
WorkingDirectory=/home/roz/ros2_ws
ExecStart=/bin/bash -lc 'source $ROS_SETUP && source $WS_SETUP && exec $4'
Restart=always
RestartSec=5
KillMode=control-group
TimeoutStopSec=15

[Install]
WantedBy=multi-user.target
EOF
  echo "  wrote /etc/systemd/system/$1"
}

echo "writing units..."

mkunit rover-camera.service \
  "Orbbec Gemini 336L ROS2 wrapper (depth+color+IMU)" \
  "" \
  "ros2 launch orbbec_camera gemini_330_series.launch.py enable_accel:=true enable_gyro:=true"

# Wants= not Requires= so a camera restart does not hard-kill the scan node;
# it just retries until the depth topics reappear.
mkunit rover-scan.service \
  "Depth image to /scan + base_link->camera_link TF" \
  "rover-camera.service" \
  "ros2 launch /home/roz/ros2_ws/launch/depth_to_scan.launch.py"
sed -i '/^After=/a Wants=rover-camera.service' /etc/systemd/system/rover-scan.service

mkunit rover-odometry.service \
  "Rover wheel odometry (VESC ERPM -> /odom)" \
  "microxrce-agent.service" \
  "ros2 run rover_odometry wheel_odometry_node"

# Restart=always here also covers the known px4_ros2 4s "no request from FMU"
# watchdog abort, which has recurred without an obvious trigger.
mkunit rover-autonav-mode.service \
  "PX4 custom mode AutoNav (px4_ros2 external mode)" \
  "microxrce-agent.service" \
  "ros2 run autonav_mode autonav_mode"

mkunit rover-ekf-bridge.service \
  "Wheel odom -> EKF2 external vision velocity (MANUAL START ONLY)" \
  "microxrce-agent.service rover-odometry.service" \
  "ros2 run rover_ekf_bridge rover_ekf_bridge"

systemctl daemon-reload

echo "enabling boot-start units (ekf-bridge intentionally excluded)..."
systemctl enable rover-camera.service rover-scan.service \
                 rover-odometry.service rover-autonav-mode.service
systemctl disable rover-ekf-bridge.service 2>/dev/null || true

echo
echo "done. units installed but NOT started -- stop your manual setsid nodes first,"
echo "then:  sudo systemctl start rover-camera rover-scan rover-odometry rover-autonav-mode"
echo "and on the floor only:  sudo systemctl start rover-ekf-bridge"
