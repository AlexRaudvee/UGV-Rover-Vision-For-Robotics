# Lab 2 Session 3: curling

This package contains the curling controller and the separate path preview.

- `l2s3_curling/path_plan.py`: the five assignment targets (metres), straight-line
  planning, replanning after pose jumps or lateral drift, and arrival checks.
- `l2s3_curling/path_node.py`: ROS pose/goal subscriptions, path publication and
  Vizanti markers.
- `l2s3_curling/node.py` and `planner.py`: the obstacle-avoiding curling controller.
- `test/test_path_plan.py`: target coordinates and planner behaviour tests.

The path preview displays a route. The curling controller drives toward a fixed
field target using the marker EKF, avoids obstacles using LiDAR alone, and records decisions. The
shared detection, triangulation, pan tracking and EKF live in the Session 1/2 packages.

After rebuilding, select the curling target from the host with
`bash start_rover.sh curling:=true target_run:=1` (runs 1 through 5).
This starts the controller idle with private preview commands. Add
`curling_drive:=true` to enable motor output, then call `/curling_controller/start`.
See the [controller guide](CONTROLLER.md#start).
Vizanti's field and 3D views draw the controller's selected goal as a yellow duck,
including before movement starts. They also show a cyan LOCAL pose, a dashed
goal reference, an orange current-action preview and a prominent controller state
panel. The visual path planner remains separate;
select the same run if using its additional path/marker overlays.

## Build and run

On the robot host, `bash start_rover.sh --build-only` includes this package.
Alternatively, inside the sourced ROS container at the workspace root:

```bash
colcon build --packages-select l2s2_localisation l2s3_curling
source install/setup.bash
```

The normal rover session starts the marker EKF and its corrected `map -> odom`
transform automatically. Run only the planner in another sourced container terminal:

```bash
ros2 run l2s3_curling path_planner --ros-args -p target_run:=1
```

Choose `target_run` from 1 to 5. The default pose input is `/marker_ekf/pose`;
`/goal_pose` accepts a clicked target. Outputs are `/path_planner/path`,
`/path_planner/markers` and `/path_planner/state`. The normal rover launch
does not automatically start the planner. Do not start a second marker EKF.

## Tests

From the workspace root (no ROS required):

```bash
PYTHONPATH=src/l2s3_curling python3 -m pytest -q src/l2s3_curling/test
```
