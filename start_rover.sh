#!/usr/bin/env bash
# Run from the robot host over SSH, outside the ROS container.
set -euo pipefail

build_mode=skip
case "${1:-}" in
    --build) build_mode=build; shift ;;
    --build-only) build_mode=only; shift ;;
    -h|--help)
        cat <<'HELP'
Usage: bash start_rover.sh [--build | --build-only] [ROS launch arguments...]

  --build       Build updated packages, then start the complete rover session.
  --build-only  Build updated packages without starting robot nodes.

Examples:
  bash start_rover.sh --build
  bash start_rover.sh
  bash start_rover.sh objects:=false obstacles:=false polar_hist:=false oak_camera:=false
  bash start_rover.sh improved_detection:=false objects:=true obstacles:=true polar_hist:=true

Run on the robot host. Stop existing robot launches before starting a session.
Ctrl+C stops the foreground session. The container remains running.
HELP
        exit 0 ;;
esac

container_name=${ROVER_CONTAINER:-ugv_jetson_ros_humble}
container_workspace=${ROVER_WORKSPACE:-/home/ws/ugv_ws}

docker start "$container_name" >/dev/null

exec_options=()
if [[ "$build_mode" != only ]]; then
    exec_options=(-it)
fi

docker exec "${exec_options[@]}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-4}" \
    -e UGV_MODEL="${UGV_MODEL:-ugv_rover}" \
    -e LDLIDAR_MODEL="${LDLIDAR_MODEL:-ld19}" \
    "$container_name" bash -c '
        set -e
        source /opt/ros/humble/setup.bash
        cd "$1"
        if [ -f install/setup.bash ]; then
            source install/setup.bash
        fi
        if [ "$2" != skip ]; then
            colcon build --packages-select apriltag --cmake-args -DBUILD_PYTHON_WRAPPER=ON -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF
            source install/setup.bash
            python3 scripts/clean_perception_metadata.py install
            MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --executor sequential --packages-select \
                usb_cam ldlidar rf2o_laser_odometry ugv_description ugv_bringup ugv_vision object_detection obstacle_detection polar_hist \
                detection_improved obstacle_avoidance_adv l2s1_marker_detection l2s2_localisation \
                vizanti_cpp vizanti_server
        fi
        source install/setup.bash
        if [ "$2" = only ]; then
            exit 0
        fi
        # Safely hand over any identified rollout helpers to the daily launch.
        python3 scripts/stop_rover_helpers.py --domain "${ROS_DOMAIN_ID:-4}"
        # Use the editable source YAML directly: start-pose edits need no rebuild.
        exec ros2 launch ugv_bringup rover.launch.py \
            "field_config:=$1/src/ugv_main/ugv_bringup/param/football_field.yaml" \
            "marker_range_config:=$1/src/l2s2_localisation/config/marker_ranges.yaml" \
            "triangulation_config:=$1/src/l2s2_localisation/config/triangulation.yaml" \
            "pan_tracking_config:=$1/src/l2s2_localisation/config/pan_tracking.yaml" "${@:3}"
    ' rover-session "$container_workspace" "$build_mode" "$@"
