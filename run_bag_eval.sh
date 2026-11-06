#!/usr/bin/env bash
# Replay a rosbag through both detectors + polar histograms and record their output.
# Usage: bash run_bag_eval.sh [run_name] [bag_file]
# Runs on an isolated ROS domain so the bag's /cmd_vel never reaches a real rover.
set -u

RUN=${1:-eval_run1}
BAG=${2:-$HOME/bags/avoidance_cleaned_v2.mcap}
OUT=$HOME/bags/$RUN
LOGS=$HOME/bags/logs/$RUN
REPO=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$REPO/../.." && pwd)

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_REPLAY:-77}
if [ "$ROS_DOMAIN_ID" = "4" ] || [ "$ROS_DOMAIN_ID" = "42" ]; then
    echo "Refusing to replay on domain $ROS_DOMAIN_ID (rover / default lab domain)."; exit 1
fi
if [ -e "$OUT" ]; then echo "$OUT already exists; pick another run name."; exit 1; fi
[ -f "$BAG" ] || { echo "Bag not found: $BAG"; exit 1; }

set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
# Job control: each background job gets its own process group, so cleanup can stop
# the `ros2 run` wrapper AND the node process it spawns (otherwise nodes outlive the run).
set -m
if pgrep -f "lib/(object_detection|obstacle_detection)/detector|lib/polar_hist/histogram" >/dev/null; then
    echo "Detector/histogram nodes from an earlier run are still running:"
    pgrep -af "lib/(object_detection|obstacle_detection)/detector|lib/polar_hist/histogram"
    echo "Stop them first:  pkill -f 'lib/(object_detection|obstacle_detection)/detector|lib/polar_hist/histogram'"
    exit 1
fi
mkdir -p "$LOGS"
OVR="$REPO/bag_replay.yaml"
pids=()

cleanup() {
    echo "Stopping nodes (recording is finalized)..."
    # Signal each job's whole process group (wrapper + node); SIGINT lets
    # rosbag2 finalize, SIGTERM/SIGKILL make sure nothing outlives the run.
    for pid in "${pids[@]}"; do kill -INT -- "-$pid" 2>/dev/null; done
    for _ in $(seq 10); do
        alive=0
        for pid in "${pids[@]}"; do kill -0 -- "-$pid" 2>/dev/null && alive=1; done
        [ "$alive" = 0 ] && break
        sleep 0.5
    done
    for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null; done
    sleep 1
    for pid in "${pids[@]}"; do kill -KILL -- "-$pid" 2>/dev/null; done
    wait 2>/dev/null
    echo "Recording: $OUT"
    echo "Logs:      $LOGS"
}
trap cleanup EXIT

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  run=$RUN"
ros2 run object_detection detector --ros-args -r __ns:=/object_detection -r __node:=detector \
    --params-file "$REPO/src/object_detection/config/detector.yaml" --params-file "$OVR" \
    < /dev/null > "$LOGS/depth.log" 2>&1 & pids+=($!)
ros2 run obstacle_detection detector --ros-args -r __ns:=/obstacle_detection -r __node:=detector \
    --params-file "$REPO/src/obstacle_detection/config/detector.yaml" --params-file "$OVR" \
    < /dev/null > "$LOGS/lidar.log" 2>&1 & pids+=($!)
ros2 launch polar_hist histogram.launch.py use_sim_time:=true \
    < /dev/null > "$LOGS/hist.log" 2>&1 & pids+=($!)
ros2 bag record -o "$OUT" \
    /object_detection/detections /obstacle_detection/detections \
    /object_detection/debug/compressed /obstacle_detection/debug/compressed \
    /object_detection/depth/compressed \
    /polar_hist/object/histogram/compressed /polar_hist/obstacle/histogram/compressed \
    /polar_hist/object/gaps /polar_hist/obstacle/gaps \
    < /dev/null > "$LOGS/record.log" 2>&1 & pids+=($!)
if [ "${NO_VIEWER:-0}" != "1" ]; then
    ros2 run rqt_image_view rqt_image_view /object_detection/debug/compressed \
        < /dev/null > "$LOGS/viewer.log" 2>&1 & pids+=($!)
fi

sleep 4   # nodes must be up before /tf_static is replayed (~2.6 s into the bag)
ros2 bag play -s mcap "$BAG" --clock
sleep 2   # let the last messages reach the recorder
