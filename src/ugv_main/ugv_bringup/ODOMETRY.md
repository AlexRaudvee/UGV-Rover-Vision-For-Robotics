# LiDAR, wheel and IMU odometry

Daily startup now selects `odometry:=lidar`. No wheel calibration procedure is
required to start using it. Existing wheel-distance units/scaling are retained,
with a deliberately weak forward-velocity weight. Wheel separation and wheel
turn estimates are not used by this mode. Wheel scale error can still affect the
result; this is an uncalibrated baseline, not a claim of measured accuracy.

Stop the previous rover launch before starting a new one. From the robot host:

```bash
cd ~/ugv_ws
bash start_rover.sh --build
```

Use `bash start_rover.sh odometry:=legacy` to select the previous estimator.
The standalone `bringup_imu_origin.launch.py` keeps its original `legacy` default;
select `odometry:=lidar` explicitly there. Never run both estimators together.
Obstacle detection/avoidance packages are independent of this integration.

## Data flow

- Existing UART feedback publishes accumulated metres on `/odom/odom_raw`.
- `odometry_inputs` differences consecutive distances into `/odom/wheels`.
  Forward velocity is fused with standard deviation 0.20 m/s. A soft zero-sideways
  velocity constraint (0.05 m/s standard deviation) models the wheeled chassis;
  this prevents an old RF2O lateral velocity from coasting forever after LiDAR loss.
  No wheel radius, separation or encoder tick count is needed by this adapter.
  The legacy array has no timestamp; receipt time is used for wheel velocity.
- The existing complementary filter publishes the physical IMU on `/imu/data`.
  The adapter preserves its frame/timestamp and publishes gyro measurements with
  explicit covariance on `/imu/estimation`. Only yaw rate is fused, with an
  assumed standard deviation of 0.03 rad/s. Neither IMU yaw nor acceleration is
  fused, avoiding reliance on unreferenced yaw or acceleration integration.
- RF2O matches consecutive `/scan` messages, applies the scanner extrinsics and
  publishes base-frame forward/lateral velocity and yaw rate on `/odom_rf2o`.
  Its assumed velocity standard deviations are 0.05 m/s and 0.05 rad/s.
  These are provisional uncertainty settings, not experimentally calibrated.
  The EKF rejects LiDAR twist innovations beyond four standard deviations.
- `robot_localization` produces `/odom` at 30 Hz and is the only publisher of
  `odom -> base_footprint`. `ekf_lidar.yaml` selects velocity measurements only;
  it does not count the corresponding integrated poses again.
- Advanced avoidance uses fused forward velocity together with physical gyro
  heading for its separate mission estimate. It no longer publishes/fuses a
  known-square landmark correction. See the controller README for its limits.

The estimator origin is at startup, with zero initial yaw. For a later mission
start, save its current pose and express subsequent poses relative to that pose.
`/odom_rf2o` is a diagnostic standalone trajectory, not a second TF authority.

## RF2O corrections

The original copy published LiDAR orientation as a fake IMU on `/imu/data`;
that output has been removed. It also negated position coordinates. Instead,
this version accounts for the scan angular center in the laser-to-base transform,
so both 0..2pi and -pi..pi scans use the same ROS coordinate convention. It waits
for a real sensor TF, seeds its first scan pyramid, initializes pose/state, uses
a valid identity quaternion, and reports lateral base velocity including the
scanner lever arm. Stationary scans no longer divide by zero in robust weights.

Invalid/out-of-range returns are masked. Too few returns, nonfinite solutions,
implausible speeds and old scans are rejected. After a scan gap or scan geometry
change the matcher starts a new scan pair at its last accepted pose. LD19 beam
count fluctuations of up to eight rays with unchanged field of view/frame are
resampled onto the established angular grid using nearest measured rays,
preserving invalid sectors instead of repeatedly resetting the matcher. It cannot
recover distance traveled during missing scans by itself; wheel/IMU prediction
continues in the EKF. Feature-poor scenes or moving objects may still give a
plausible but wrong match. These checks do not certify scan-match quality.

## Freshness and verification

```bash
# Inside the container, after sourcing ROS and the workspace, on ROS_DOMAIN_ID=4:
ros2 topic hz /odom
ros2 topic hz /odom_rf2o
ros2 topic echo /diagnostics
ros2 run tf2_ros tf2_echo odom base_footprint
```

Check `odometry/input_freshness`: wheels, IMU and LiDAR must all be fresh for
full fusion. A fresh `/odom` timestamp alone is insufficient: the EKF can keep
predicting after inputs stop. Diagnostic WARN means missing/rejected/stale input;
OK means fresh measurements, not proven position accuracy. Controllers should
check input diagnostics and stop on degraded localization. This package supplies
estimation and diagnostics; it does not implement command arbitration or stopping.

The LD19 launch uses its stable USB adapter ID and respawns the driver. After
repeated data timeouts the driver now closes its serial descriptor and exits,
allowing respawn to reopen the device if USB reconnect changes ttyACM numbering.
The avoidance controller remains stopped after a sensor-loss fault; restarting
the sensor does not arm a mission.

The default adapter timeout and maximum scan-pair gap are 0.5 s. Wheel jumps
above 1.5 m/s rebase the counters without publishing a velocity. RF2O rejects
motion above 1.5 m/s or 3 rad/s. These are rejection limits, not speed commands.
Long delayed wheel packets cannot be recognized as old by the legacy array API.

Build and run synthetic regression checks inside the sourced container:

```bash
colcon build --executor sequential --packages-select rf2o_laser_odometry ugv_bringup
source install/setup.bash
python3 -m pytest -q src/ugv_main/ugv_bringup/test/test_odometry_inputs.py
python3 scripts/test_lidar_odometry_synthetic.py
python3 scripts/test_stationary_lidar_dropout.py
```

The synthetic script uses ROS domain 87, starts only estimation nodes, and sends
no motor commands. It tests stationary/forward/curved/reverse/lateral motion,
a 30% wheel-scale error, nonzero scanner extrinsics, both scan-angle conventions,
beam-count changes, invalid scans and missing-input diagnostics. It does not
replace physical validation. Confirm direction and approximate travel with a
short supervised run before depending on the estimate for autonomous driving.

For recorded-data tests, launch only `odometry_lidar.launch.py use_sim_time:=true`
with recorded sensors/TF and `/clock`; omit recorded `/odom` and its dynamic
odom-to-base transform to avoid competing publishers. Do not launch the rover
hardware bringup to replay a bag.

LiDAR fusion still accumulates drift. Reliable longer routes or return to a
physical starting spot require map localization/SLAM or another external reference.
