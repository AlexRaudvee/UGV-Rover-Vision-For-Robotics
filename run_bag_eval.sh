#!/usr/bin/env bash
# Replay a rosbag through both detectors + polar histograms and record their output.
# Usage: bash run_bag_eval.sh [run_name] [bag] [params_yaml]
#   bag:         an .mcap file or a rosbag2 folder (with metadata.yaml)
#   params_yaml: detector overrides for that bag (default: bag_replay.yaml)
# DETECTORS=improved runs the detection_improved package instead of object_detection,
# obstacle_detection and polar_hist, under the same topic names (default: DETECTORS=default).
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
[ -e "$BAG" ] || { echo "Bag not found: $BAG"; exit 1; }
if [ -d "$BAG" ]; then PLAY_ARGS=("$BAG"); else PLAY_ARGS=(-s mcap "$BAG"); fi

set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
# Job control: each background job gets its own process group, so cleanup can stop
# the `ros2 run` wrapper AND the node process it spawns (otherwise nodes outlive the run).
set -m
NODES="lib/(object_detection|obstacle_detection)/detector|lib/polar_hist/histogram|lib/detection_improved/"
if pgrep -f "$NODES" >/dev/null; then
    echo "Detector/histogram nodes from an earlier run are still running:"
    pgrep -af "$NODES"
    echo "Stop them first:  pkill -f '$NODES'"
    exit 1
fi
mkdir -p "$LOGS"
OVR=${3:-$REPO/bag_replay.yaml}
DETECTORS=${DETECTORS:-default}
case "$DETECTORS" in default|improved) ;; *) echo "DETECTORS must be default or improved"; exit 1;; esac
[ -f "$OVR" ] || { echo "Params file not found: $OVR"; exit 1; }
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

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  run=$RUN  bag=$BAG  params=$OVR  detectors=$DETECTORS"
if [ "$DETECTORS" = improved ]; then
    IMP="$REPO/src/detection_improved/config"
    # Same node names and topics as the default packages, so the recording and the
    # export scripts do not depend on which detectors produced it.
    ros2 run detection_improved depth_detector --ros-args -r __ns:=/object_detection \
        -r __node:=detector --params-file "$IMP/depth.yaml" --params-file "$OVR" \
        < /dev/null > "$LOGS/depth.log" 2>&1 & pids+=($!)
    ros2 run detection_improved lidar_detector --ros-args -r __ns:=/obstacle_detection \
        -r __node:=detector --params-file "$IMP/lidar.yaml" --params-file "$OVR" \
        < /dev/null > "$LOGS/lidar.log" 2>&1 & pids+=($!)
    ros2 run detection_improved histogram --ros-args -r __ns:=/polar_hist \
        -r __node:=polar_hist --params-file "$IMP/histogram.yaml" \
        -p object_topic:=/object_detection/detections \
        -p obstacle_topic:=/obstacle_detection/detections \
        -p frame_prefix:=polar_hist -p use_sim_time:=true \
        < /dev/null > "$LOGS/hist.log" 2>&1 & pids+=($!)
else
    ros2 run object_detection detector --ros-args -r __ns:=/object_detection -r __node:=detector \
        --params-file "$REPO/src/object_detection/config/detector.yaml" --params-file "$OVR" \
        < /dev/null > "$LOGS/depth.log" 2>&1 & pids+=($!)
    ros2 run obstacle_detection detector --ros-args -r __ns:=/obstacle_detection -r __node:=detector \
        --params-file "$REPO/src/obstacle_detection/config/detector.yaml" --params-file "$OVR" \
        < /dev/null > "$LOGS/lidar.log" 2>&1 & pids+=($!)
    ros2 launch polar_hist histogram.launch.py use_sim_time:=true \
        < /dev/null > "$LOGS/hist.log" 2>&1 & pids+=($!)
fi
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
ros2 bag play "${PLAY_ARGS[@]}" --clock
sleep 2   # let the last messages reach the recorder
