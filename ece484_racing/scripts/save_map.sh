#!/bin/bash
# Save the current slam_toolbox map: ~/ece484/maps/NAME.{pgm,yaml} (nav2) and NAME.{posegraph,data} (slam_toolbox)
set -e
NAME="${1:?usage: save_map.sh NAME}"
source /opt/ros/jazzy/setup.bash
mkdir -p "$HOME/ece484/maps"
OUT="$HOME/ece484/maps/$NAME"
ros2 run nav2_map_server map_saver_cli -f "$OUT"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$OUT'}"
ls -l "$OUT".*
