# Lab 2 Session 3: curling

Session 3 target-planning code lives here, moved from `l2s2_localisation`.

- `l2s3_curling/path_plan.py`: the five assignment targets (metres), straight-line
  planning, replanning after pose jumps or lateral drift, and arrival checks.
- `l2s3_curling/path_node.py`: ROS pose/goal subscriptions, path publication and
  Vizanti markers.
- `test/test_path_plan.py`: target coordinates and planner behaviour tests.

The planner displays a path; it does not publish motor commands or avoid obstacles.
The shared detection, triangulation, pan tracking and EKF remain in the Session 1/2
packages. Existing obstacle avoidance remains in `obstacle_avoidance_adv` and can
now drive towards these targets using the marker EKF. Intermediate stops are
permitted under the TA's update.

After rebuilding, select the avoidance target from the host with
`bash start_rover.sh avoidance:=true target_run:=1` (runs 1 through 5).
This starts the controller idle with private preview commands. The existing
`avoidance_drive:=true` option enables motor output; the start service remains
required. See the [controller guide](../obstacle_avoidance_adv/README.md#start).
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
