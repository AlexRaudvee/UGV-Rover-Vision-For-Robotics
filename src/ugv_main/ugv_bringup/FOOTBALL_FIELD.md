# Football field in Vizanti

The daily rover launch starts `football_field` and supplies a 9 x 6 m RoboCup SPL
field to both the 2D view and the 3D Rover + Depth panel. Both use geometry from
one ROS publisher and the same `map -> odom` alignment. The field server never
publishes velocity or pan-tilt commands.

## Change the starting position

Edit these three numbers in **`param/football_field.yaml`**:

```yaml
football_field:
  ros__parameters:
    start_x_m: -3.2
    start_y_m: 0.0
    start_heading_deg: 0.0
```

`start_rover.sh` passes the **source file** directly to the node, so start edits
need no rebuild. During a session, the file is checked every second. A valid
start edit saves the desired pose and aligns the current fresh odometry pose
with it. If odometry is unavailable, initialization waits for its next valid
message. Partial/invalid YAML leaves the last valid alignment unchanged and
shows an error in Vizanti. Other field geometry/frame changes need a restart.

Alternatively, open **Football Field** in the 2D toolbar, or expand **Starting
position** at the bottom right of the 3D view. Enter x, y and heading, then click
**Save & apply start**. This writes the same three numbers back to the YAML,
preserving its comments and other settings. All connected views receive the
updated alignment. **Apply saved start here** reuses the saved numbers for a new
run without changing them. It requires fresh odometry.

Place the robot's `base_footprint` reference point over the intended mark and
point its body in the intended direction before applying. These controls change
coordinates; they do not physically move the robot. Saving a new pose during a
run declares that the robot is currently at that pose, so apply while stationary.

The field centre is `(0, 0)`, +x points toward the opposite goal, +y points left,
and heading is counter-clockwise positive:

| Starting place | x (m) | y (m) | Heading (degrees) |
|---|---:|---:|---:|
| Left penalty mark, facing centre | -3.2 | 0.0 | 0.0 |
| Right penalty mark, facing centre | 3.2 | 0.0 | 180.0 |
| Centre, facing +x | 0.0 | 0.0 | 0.0 |

These nominal values follow the [official SPL 2025 rules, figures 1-3](https://spl.robocup.org/wp-content/uploads/SPL-Rules-2025.pdf).
The carpet is 10.4 x 7.4 m including the 0.7 m border. The penalty marks are
1.3 m inward from the goal lines, hence 3.2 m from the centre.

On startup, `initialize_on_first_odom: true` assumes the first valid odometry
message is at the configured start. Set it to `false` to require explicitly
applying the saved start each session. Applying works with nonzero odometry;
restarting/resetting the odometry estimator is not required. Clock rewinds clear
the old anchor. Physically repositioning the robot or resetting odometry during
a session requires applying the saved start again. Ordinary odometry drift still
requires external localization to correct it.

## Startup and views

Build once from the host, without starting hardware:

```bash
bash start_rover.sh --build-only
```

Then use the normal rover session (stop the old session before starting another):

```bash
bash start_rover.sh
```

Reload Vizanti. Its existing saved layout receives **Football Field** and the
2D robot widget automatically. The initial fixed frame becomes `map`. Other
camera widgets and their positions are preserved. Use **Back to 2D**, **3D**,
**View whole field**, and **Fit field to view** to switch views and fit the pitch.
The gold marker is the configured start; the robot follows odometry in field
coordinates. Applying a start clears the 3D trail and old depth samples. The
rover, new depth points and trail all use the same rotation and translation.

Start only the field server in an already sourced ROS terminal:

```bash
ros2 launch ugv_bringup football_field.launch.py \
  field_config:=/home/ws/ugv_ws/src/ugv_main/ugv_bringup/param/football_field.yaml
```

This standalone launch starts no drivers. Do not start a second field server if
the daily launch already started one. `football_field:=false` disables it in
the daily launch; `field_config:=/absolute/path/custom.yaml` selects another file.
The file path displayed in Vizanti is the authoritative file being read/written.

If AMCL, SLAM or another localizer already publishes `map -> odom`, set
`publish_tf: false` before starting this field server. It will display that
external transform instead, and the start-alignment controls are disabled.
Do not keep an old manual static `map -> odom` publisher running alongside it.

## Occupancy map and ROS outputs

Field markings are visual graphics, not obstacles. `/field_map` publishes a
retained `nav_msgs/OccupancyGrid` with 1 cm resolution and origin
`[-5.2, -3.7, 0]`. The default contains free carpet with **no invented perimeter
walls**. It is a field footprint, not a surveyed LiDAR localization map. It does
not include people, furniture, or other real obstacles.

`map_goal_obstacles: true` includes nominal goalpost and net footprints if those
structures are physically present. This option affects occupancy, not painted
lines. A Map widget may display `/field_map` when needed, but an opaque occupancy
layer will cover the green/white field graphics.

`maps/robocup/robocup_field.yaml` and `.pgm` are also supplied at workspace level.
Regenerate them after geometry/occupancy edits:

```bash
python3 scripts/generate_football_map.py
```

| Output/service | Purpose |
|---|---|
| `/football_field/layout` | Shared metre-based drawing primitives (JSON String) |
| `/football_field/state` | Start, alignment, freshness and config path (JSON String) |
| `/field_map` | Free carpet / optional goal occupancy |
| `/tf` | `map -> odom` alignment when enabled |
| `/football_field/apply_start` | Trigger: align the current robot pose to the saved start |
| `/football_field/set_parameters_atomically` | Save/apply the three `start_*` parameters together |

The marker-localization marker map still needs its actual landmark coordinates;
the standard pitch geometry does not determine arbitrary marker placements.

## Verification

Tests use synthetic odometry on isolated ROS domain **96**, temporary YAML files,
and no hardware or movement commands. Run in the sourced container:

```bash
PYTHONPATH=src/ugv_main/ugv_bringup:$PYTHONPATH python3 -m pytest -q \
  src/ugv_main/ugv_bringup/test/test_field_model.py \
  src/ugv_main/ugv_bringup/test/test_football_field.py
node scripts/test_vizanti_field.cjs
node scripts/test_vizanti_scene3d.cjs
```

After physically initializing on the left penalty mark:

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

Expected initial pose is `(-3.2, 0.0, 0 degrees)`. Moving 1 m toward the centre
should give approximately `(-2.2, 0.0, 0 degrees)`.
