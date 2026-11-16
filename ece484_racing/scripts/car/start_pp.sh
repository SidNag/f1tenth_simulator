#!/bin/bash
# AMCL localization + pure pursuit (+ a bag), as detached background jobs. Map-agnostic.
# Run ~/sid/start_bringup.sh FIRST. Stop everything with ~/sid/stop_pp.sh
#
# Usage: ~/sid/start_pp.sh [MAP.yaml] [WAYPOINTS.csv] [nobag] [extra launch args ...]
#   MAP.yaml       default $MAP or ~/sid/maps/track_fixed.yaml
#   WAYPOINTS.csv  default $WP  or ~/sid/maps/car_track_fixed.csv
#   start pose     from MAP.start next to the map ("x:=.. y:=.. yaw:=..", written by make_track_wp.py),
#                  else x=0 y=0 yaw=0 (= where/how the car sat when that mapping run started)
#   extra launch args override anything, e.g.  x:=0.5 y:=0.04 yaw:=0.26  speed:=0.8 use_csv_speed:=false  pp:=false
# Examples:
#   ~/sid/start_pp.sh                                        # track_fixed + car_track_fixed.csv
#   ~/sid/start_pp.sh ~/sid/maps/new.yaml ~/sid/maps/new_wp.csv
#   MAP=~/sid/maps/new.yaml WP=~/sid/maps/new_wp.csv ~/sid/start_pp.sh pp:=false   # localization only
# DRIVE: hold RB (right bumper) ONLY. Release RB = stop. LB + sticks = manual override (teleop wins the mux).
SETUP="source /opt/ros/jazzy/setup.bash; source ~/f1tenth_ws/install/setup.bash; source ~/sid/ws/install/setup.bash"
eval "$SETUP"
mkdir -p ~/sid/run ~/sid/logs ~/sid/bags
MAP=${MAP:-$HOME/sid/maps/track_fixed.yaml}; WP=${WP:-$HOME/sid/maps/car_track_fixed.csv}; BAG=1
while [ $# -gt 0 ]; do
  case "$1" in
    *.yaml) MAP=$(readlink -f "$1");; *.csv) WP=$(readlink -f "$1");; nobag) BAG=0;; *) break;;
  esac; shift
done
[ -f "$MAP" ] || { echo "ERROR: map $MAP not found"; exit 1; }
[ -f "$WP" ] || { echo "ERROR: waypoints $WP not found"; exit 1; }
START="x:=0.0 y:=0.0 yaw:=0.0"; [ -f "${MAP%.yaml}.start" ] && START=$(cat "${MAP%.yaml}.start")
echo "map: $MAP"; echo "waypoints: $WP ($(($(wc -l < "$WP")-1)) pts)"; echo "initial pose: $START $*"
start() { # name, command...
  local name=$1; shift
  if [ -f ~/sid/run/$name.pid ] && kill -0 -- -$(cat ~/sid/run/$name.pid) 2>/dev/null; then echo "$name already running"; return; fi
  setsid bash -c "$SETUP; exec $*" < /dev/null > ~/sid/logs/$name.log 2>&1 &
  echo $! > ~/sid/run/$name.pid; echo "started $name (pid $!)  log: ~/sid/logs/$name.log"
}
timeout 5 ros2 topic list 2>/dev/null | grep -qx /scan || echo "WARNING: no /scan topic -- is bringup running? (~/sid/start_bringup.sh)"
start pp ros2 launch ece484_racing car_pp.launch.py map:=$MAP waypoint_file:=$WP $START "$@"
if [ $BAG = 1 ]; then
  sleep 3
  start ppbag ros2 bag record -o $HOME/sid/bags/pp_$(date +%H%M) /scan /odom /tf /tf_static /sensors/core /joy /drive /teleop /ackermann_drive /amcl_pose /particle_cloud /pure_pursuit/lookahead
fi
echo "Check: tail -f ~/sid/logs/pp.log   (want: 'Loaded N waypoints', amcl 'Setting pose', lifecycle 'Managed nodes are active')"
