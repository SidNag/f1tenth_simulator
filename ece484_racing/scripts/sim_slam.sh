#!/bin/bash
# Launch the F1TENTH gym sim (no map_server) + slam_toolbox online async mapping.
# Ctrl-C stops both cleanly. Logs: /tmp/sim.log, /tmp/slam.log
source /opt/ros/jazzy/setup.bash
source "$HOME/ece484/sim_ws/install/setup.bash"
export PYTHONPATH=$HOME/ece484/f1tenth_simulator/gym:$HOME/ece484/f1tenth_simulator/f1tenth_gym:$HOME/virtual/f1tenth/lib/python3.12/site-packages:$PYTHONPATH
PARAMS="${SLAM_PARAMS:-$HOME/ece484/params/slam_sim.yaml}"

SIM_PID=""; SLAM_PID=""
cleanup() {
  trap - INT TERM EXIT
  echo; echo "Stopping SLAM and sim..."
  for p in $SLAM_PID $SIM_PID; do kill -INT -- -"$p" 2>/dev/null; done
  for i in $(seq 1 20); do
    alive=0; for p in $SLAM_PID $SIM_PID; do kill -0 -- -"$p" 2>/dev/null && alive=1; done
    [ $alive = 0 ] && break; sleep 0.5
  done
  for p in $SLAM_PID $SIM_PID; do kill -KILL -- -"$p" 2>/dev/null; done
  ros2 daemon stop >/dev/null 2>&1
  echo "Done."
  exit 0
}
trap cleanup INT TERM EXIT

echo "Starting sim (log: /tmp/sim.log)..."
setsid ros2 launch f1tenth_gym_ros gym_bridge_launch.py use_map_server:=false < /dev/null > /tmp/sim.log 2>&1 &
SIM_PID=$!
sleep 5
echo "Starting slam_toolbox with $PARAMS (log: /tmp/slam.log)..."
setsid ros2 launch slam_toolbox online_async_launch.py slam_params_file:="$PARAMS" use_sim_time:=false < /dev/null > /tmp/slam.log 2>&1 &
SLAM_PID=$!
echo "Running. Drive in another terminal; save with ~/ece484/scripts/save_map.sh NAME. Ctrl-C here to stop."
wait $SIM_PID
