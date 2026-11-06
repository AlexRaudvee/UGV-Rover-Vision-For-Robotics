# obstacle_avoidance

Drives the rover forward, goes around black obstacles, and returns to the original path. It uses the LiDAR detector and gap list by default.

**This node moves the robot.** It is not started by `start_rover.sh`, so it only runs when you start it. Have someone next to the robot, and press Ctrl+C to stop.

## How it works

The robot's start pose defines the path.

| State | What the robot does | When it moves on |
| --- | --- | --- |
| `CRUISE` | Follows the path | An obstacle in the path is within 1 m |
| `TURN` | Turns on the spot to the chosen heading | The heading is reached |
| `PASS` | Drives to a lane beside the obstacle and follows it | It is 65 cm past the front of the obstacle |
| `RETURN` | Drives back to the path | It is within 5 cm of the path |
| `HOLD` | Waits, because there is no safe way around | A way opens up, or it gives up after 15 s |
| `DONE` | Stopped | The run is over |

The side is chosen once per obstacle and kept until the robot has passed it. This is the hysteresis the assignment asks for. The robot picks the safe lane closest to the path, using gaps from `polar_hist` and the free space beside each obstacle.

The robot stops if:

- detections, odometry or the LiDAR stop arriving
- the LiDAR sees something right in front of it, even if the detectors missed it
- it reaches `target_distance`, in any state

A black object without a measured distance does not stop the robot, because dark floor markings would stop it for good. It slows down instead, avoids aiming at that object, and relies on the LiDAR stop.

## Run it

Start the rover in one terminal:

```bash
bash start_rover.sh --build
```

Start the controller in the container. `target_distance` must fit inside the field:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=4
ros2 launch obstacle_avoidance avoidance.launch.py target_distance:=2.0
```

To test each side, add `preferred_side:=left` or `preferred_side:=right`. To watch the state:

```bash
ros2 topic echo /avoidance/status
```

Other settings are in `config/avoidance.yaml`.

## First test on the robot

1. Put the robot on a box so the wheels are off the ground, then check that the wheels stop with Ctrl+C.
2. Check the LiDAR stop: hold a box in front of the sensor. The status should say `LiDAR stop`.
3. With the robot on the ground and no obstacles, use `target_distance:=1.0`.
4. Add one obstacle, then two.

## Tests

```bash
colcon test --packages-select obstacle_avoidance
colcon test-result --verbose
```

The tests run on a simulated robot and do not move anything.
