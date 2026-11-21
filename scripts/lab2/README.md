# Lab 2 rover scripts

Run inside the rover container (`docker exec -it -e ROS_DOMAIN_ID=4 ugv_jetson_ros_humble bash`,
then `source /opt/ros/humble/setup.bash && source /home/ws/ugv_ws/install/setup.bash`).

| Script | What it does |
|---|---|
| `sweep_pan.py` | Turns the camera in 30 deg steps from 0 to 180 to -150 and back to 0 (3 s each) |
| `sweep_continuous.py SECONDS` | Swings the camera back and forth between -120 and +120 deg at 30 deg/s |
| `drive_straight.py` | Drives 2 m straight at 0.15 m/s (stops by odometry), then stops |
| `drive_arc.py` | Drives a slow left arc, then stops |

The sweep scripts take over the camera, so stop the automatic pan tracker first:

    kill -INT $(pgrep -f lib/l2s2_localisation/pan_tracker)

and start it again afterwards:

    ros2 run l2s2_localisation pan_tracker --ros-args -r __node:=marker_pan_tracker \
        --params-file /home/ws/ugv_ws/src/l2s2_localisation/config/pan_tracking.yaml

The drive scripts move the rover: check the path is clear, also straight ahead.
To record while a script runs, start `ros2 bag record ...` in another terminal first.
