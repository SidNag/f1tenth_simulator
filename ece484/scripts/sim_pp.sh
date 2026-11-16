#!/bin/bash
# Sim (with map_server) + pure pursuit on ground truth (static map->odom).
# Extra args are passed to sim_pp.launch.py, e.g.:  sim_pp.sh speed:=2.0 lookahead_min:=0.6
# Ctrl-C stops both. Logs: /tmp/sim.log, /tmp/pp.log
source /opt/ros/jazzy/setup.bash
source "$HOME/ece484/sim_ws/install/setup.bash"
export PYTHONPATH=$HOME/ece484/f1tenth_simulator/gym:$HOME/ece484/f1tenth_simulator/f1tenth_gym:$HOME/virtual/f1tenth/lib/python3.12/site-packages:$PYTHONPATH

SIM_PID=""; PP_PID=""
cleanup() {
  trap - INT TERM EXIT
  echo; echo "Stopping pure pursuit and sim..."
  for p in $PP_PID $SIM_PID; do kill -INT -- -"$p" 2>/dev/null; done
  for i in $(seq 1 20); do
    alive=0; for p in $PP_PID $SIM_PID; do kill -0 -- -"$p" 2>/dev/null && alive=1; done
    [ $alive = 0 ] && break; sleep 0.5
  done
  for p in $PP_PID $SIM_PID; do kill -KILL -- -"$p" 2>/dev/null; done
  ros2 daemon stop >/dev/null 2>&1
  echo "Done."
  exit 0
}
trap cleanup INT TERM EXIT

echo "Starting sim (log: /tmp/sim.log)..."
setsid ros2 launch f1tenth_gym_ros gym_bridge_launch.py < /dev/null > /tmp/sim.log 2>&1 &
SIM_PID=$!
sleep 5
echo "Starting pure pursuit $* (log: /tmp/pp.log)..."
setsid ros2 launch ece484_racing sim_pp.launch.py "$@" < /dev/null > /tmp/pp.log 2>&1 &
PP_PID=$!
echo "Running. Ctrl-C here to stop."
wait $SIM_PID
