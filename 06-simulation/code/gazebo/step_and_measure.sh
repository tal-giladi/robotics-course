#!/usr/bin/env bash
# 06.03 — Step a paused Gazebo world a known number of physics steps and print model positions.
#
#   bash step_and_measure.sh                    # first_world.sdf as it is (mu = 0.3)
#   bash step_and_measure.sh 0.5                # same world, ramp and slider friction set to 0.5
#
# Runs gz sim headless and paused, then drives it with the /world/<name>/control service, so the
# numbers are identical on every machine regardless of CPU speed (lockstep, not wall clock).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MU="${1:-0.3}"
WORLD=/tmp/first_world_mu.sdf
sed "s#<mu>0.3</mu><mu2>0.3</mu2>#<mu>${MU}</mu><mu2>${MU}</mu2>#g" "$HERE/first_world.sdf" > "$WORLD"

gz sim -s -v 1 "$WORLD" > /tmp/step_and_measure_gz.log 2>&1 &   # -s server only, no -r: starts paused
GZ_PID=$!
trap 'kill -INT $GZ_PID 2>/dev/null; wait $GZ_PID 2>/dev/null' EXIT

# wait until the world answers
for _ in $(seq 1 30); do
  gz service -l 2>/dev/null | grep -q "/world/first_world/control" && break
  sleep 1
done

position() {  # position <model>  ->  "x y z"
  gz model -m "$1" -p | awk '/Pose \[/ {getline; gsub(/[\[\]]/, ""); print $1, $2, $3}'
}
step() {      # step <n>: advance exactly n physics steps (1 ms each), stay paused
  gz service -s /world/first_world/control --reqtype gz.msgs.WorldControl \
    --reptype gz.msgs.Boolean --timeout 5000 --req "pause: true, multi_step: $1" > /dev/null
  sleep 1   # the service returns before the steps finish; give the server a moment
}

echo "friction mu = $MU"
printf "%8s  %-28s  %-28s\n" "t (s)" "drop_box x y z" "slider x y z"
t=0
for n in 0 400 40 560; do
  [[ $n -gt 0 ]] && step "$n"
  t=$((t + n))
  printf "%4d.%03d  %-28s  %-28s\n" $((t / 1000)) $((t % 1000)) "$(position drop_box)" "$(position slider)"
done
