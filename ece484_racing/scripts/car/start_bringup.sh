#!/bin/bash
# Bringup ONLY (VESC + lidar + joy/joy_teleop + mux + odom TF), slow joystick config.
# Detached like start_mapping.sh. Logs: ~/sid/logs/bringup.log  Stop: ~/sid/stop_pp.sh (or stop_mapping.sh bringup)
source /opt/ros/jazzy/setup.bash; source ~/f1tenth_ws/install/setup.bash
mkdir -p ~/sid/run ~/sid/logs
if [ -f ~/sid/run/bringup.pid ] && kill -0 -- -$(cat ~/sid/run/bringup.pid) 2>/dev/null; then echo "bringup already running"; exit 0; fi
setsid bash -c "source /opt/ros/jazzy/setup.bash; source ~/f1tenth_ws/install/setup.bash; exec ros2 launch f1tenth_stack bringup_launch.py vesc_config:=$HOME/sid/vesc_car.yaml joy_config:=$HOME/sid/joy_slow.yaml" < /dev/null > ~/sid/logs/bringup.log 2>&1 &
echo $! > ~/sid/run/bringup.pid; echo "started bringup (pid $!)  log: ~/sid/logs/bringup.log"
