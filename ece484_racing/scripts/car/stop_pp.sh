#!/bin/bash
# Stop in order: pure pursuit+AMCL, bag, then bringup. Usage: stop_pp.sh [pp|ppbag|bringup ...]
names=${*:-pp ppbag bringup}
for n in $names; do
  f=~/sid/run/$n.pid; [ -f $f ] || continue; p=$(cat $f)
  kill -INT -- -$p 2>/dev/null && echo "SIGINT -> $n"
  for i in $(seq 1 20); do kill -0 -- -$p 2>/dev/null || break; sleep 0.5; done
  kill -0 -- -$p 2>/dev/null && { kill -KILL -- -$p; echo "killed $n"; }
  rm -f $f
done
