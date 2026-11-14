# ugv_ws
## Workspace to use ROS2 Humble with Waveshare's UGV Rover

## Daily startup on rover-04

The existing `ugv_jetson_ros_humble` container mounts `/home/jetson` on the robot
at `/home/ws` inside the container. Editing `~/ugv_ws/src` on the host therefore
updates `/home/ws/ugv_ws/src` immediately. Building packages and launching their
nodes are separate steps; neither requires rebuilding this Docker image.

From an SSH terminal on the **robot host**, stop any previous ROS launch sessions
with Ctrl+C, then run:

```bash
cd ~/ugv_ws
bash start_rover.sh --build
```

This starts the existing container if necessary, builds the project packages,
and launches the motor interface, sensor feedback, odometry, IMU filter, LD19,
robot transforms, USB and OAK cameras, Vizanti, the improved depth/LiDAR
detectors, and their histograms together. The original detection stack is disabled.
Leave this terminal open. Ctrl+C stops this foreground ROS session; the container
remains running. This does not install a service that launches ROS at boot or on
every `docker start`.

Odometry now defaults to LiDAR/wheel/IMU fusion, with conservative wheel weighting
and no wheel-turn calibration required. Build RF2O once with `--build`. Use
`odometry:=legacy` for the previous estimator. See the
[odometry guide](src/ugv_main/ugv_bringup/ODOMETRY.md) for topics, diagnostics,
limitations, and isolated synthetic tests.

Vizanti now opens a **3D rover and depth** view. The floor grid is 1 m, green
axes mark the odometry starting point, and the rover outline follows `/odom`.
Drag to orbit, scroll to zoom, or select **Follow rover**. **Back to 2D** and the
**3D** toolbar button switch views. The blue trail records motion since opening
the view; numeric x/y/yaw remain relative to the odometry origin. The rover is a
schematic body/wheel outline, not a loaded CAD mesh.

The shared WLS metric depth is projected using `/wls_filtered/camera_info` and transformed
into `odom` at each image timestamp, then published on `/vizanti/depth/points`.
The view shows live height-coloured points (0.15–6 m), capped to roughly 5,000
points at 5 Hz. Projection uses the existing compressedDepth transport to avoid
raw-image backlogs. Up to 3 s of capture delay is accepted, with the original
acquisition-time transform; the 3D status line reports that delay.
It uses the existing camera TF/extrinsics; floor alignment depends
on that mounting geometry being correct. This is a live depth view, not an
accumulated 3D map. Missing/stale depth is hidden and reported in the view.
Use `depth_3d:=false` to disable projection. Restart the rover session and reload
Vizanti after building; existing saved dashboards receive the new widget.

Advanced avoidance is available as an opt-in package. See the
[controller guide](src/obstacle_avoidance_adv/README.md) for preview/motor modes,
start/stop services, configuration and validation limits. It follows an 8 m
mission route, stops before an obstacle, makes two gyro-controlled 90 degree
turns around a sidestep, passes 1 m, then drives diagonally to the original goal.
`avoidance:=true` starts it in IDLE with
private preview commands; `avoidance_drive:=true` explicitly selects motor output.

On later runs without source changes:

```bash
bash start_rover.sh
```

Build without starting nodes using `bash start_rover.sh --build-only`. The build
assumes the original workspace dependencies are already installed and built.
Rebuild after changing package metadata, launch/configuration files, or source
code, then restart the affected nodes to load the new version. This is an
incremental build and does not clear the workspace. Docker image changes are
only needed when changing the image's installed environment; persist those
dependency changes in the Docker image definition if the container is recreated.

Choose which components run in
`src/ugv_main/ugv_bringup/launch/rover.launch.py`. For example, use the USB detector
and skip the OAK driver with:

```bash
bash start_rover.sh improved_detection:=false obstacles:=true obstacle_camera:=usb oak_camera:=false
```

The launch arguments `base`, `usb_camera`, `oak_camera`, `vizanti`, and
`improved_detection` default to `true`; `objects`, `obstacles`, and `polar_hist`
default to `false`. Improved detection includes its own histogram node. Its depth
detector requires the OAK camera. The live OAK profiles apply the same 5 degree
downward pitch correction as the recorded bag. Keep the detector's selected camera enabled. The base
launch supplies the robot transforms, so this session does not additionally
launch `lab1_view.launch.py`.

On your **laptop**, open `http://10.0.80.4:5100` in a browser while connected to
the robot's network. This address was checked on rover-04; it can change. Vizanti
also needs TCP port 5001 for its WebSocket connection. The container uses host
networking, so no Docker port mappings are needed. Alternatively, keep this
command running in a laptop terminal and open `http://localhost:5100`:

```bash
ssh -N -L 5100:127.0.0.1:5100 -L 5001:127.0.0.1:5001 jetson@10.0.80.4
```

To verify actual running nodes and data, use a second SSH terminal on the robot:

```bash
docker exec -it ugv_jetson_ros_humble bash
source /opt/ros/humble/setup.bash
source /home/ws/ugv_ws/install/setup.bash
export ROS_DOMAIN_ID=4
ros2 pkg executables detection_improved
ros2 node list
ros2 topic hz /detection_improved/polar_hist/obstacle/histogram/compressed
```

`ros2 pkg executables` confirms installation; `/detection_improved/obstacle/detector` in
the node list confirms it is running. Histogram messages confirm output, but
can also report stale/missing camera input. Inspect
`ros2 topic echo /detection_improved/obstacle/detections --once` and the debug image to
check the input's freshness. Add Compressed Image widgets in Vizanti for
`/detection_improved/obstacle/debug/compressed` and
`/detection_improved/polar_hist/obstacle/histogram/compressed`.

`detection_improved` measures objects using registered OAK depth and projected
LiDAR ranges, with floor checks for depth and merged object pieces. Its histogram
node plots each detector and marks measured openings of at least 35 cm.
The rover launch supplies a six-widget Vizanti layout for new browser sessions.
Saved dashboards migrate their managed perception topics on reload while keeping
widget positions. You can also import `src/polar_hist/config/vizanti_layout.json`
in Vizanti Settings. RGB uses `/oak/rgb/image_rect/compressed`;
the metric depth preview is `/detection_improved/object/depth/compressed`. The depth
histogram is `/detection_improved/polar_hist/object/histogram/compressed`. Read the
[perception guide](src/polar_hist/README.md) for input calibration, equations,
parameters, gap semantics, and tests. No autonomous controller is started;
these packages publish perception output without driving motors.

The dashboard also includes `/wls_filtered/compressed` and a colorized metric
depth view. When Vizanti is enabled, `wls_depth_preview` converts
`/wls_filtered/compressedDepth` (PNG transport) into
`/wls_filtered/depth_preview/compressed`. The rover starts `ugv_vision/wls_depth` by default (`wls:=false` disables it).
It applies RGB-guided WLS smoothing to inverse metric depth at up to 640 pixels
wide and 10 Hz. Both improved detectors and the 3D view use
`/wls_filtered/compressedDepth`, with `/wls_filtered/camera_info` scaled to match.
The preview renders those same measurements; it is not itself a metric input.
The obstacle branch defaults to depth too; `obstacle_range_source:=lidar` restores
independent LiDAR ranging without changing LiDAR odometry.

Edit **`src/ugv_main/ugv_vision/config/wls_depth.yaml`** for smoothing strength
(`lambda`), RGB edge sensitivity (`sigma_color`), hole interpolation (`fill_holes`),
shared depth EMA (`ema_tau_s`, `ema_max_delta_m`),
resolution/rate, and preview colour limits. Edit **`config/depth_cloud.yaml`** in
that package for 3D clipping range and point density. OAK sensor filters remain
in **`config/oak_d_lite.yaml`**. See the [depth configuration guide](src/ugv_main/ugv_vision/DEPTH.md)
for units, detector settings, and how to apply changes.


This is a fork from https://github.com/waveshareteam/ugv_ws.git, created for the course 'Vision for Autonomous Robots' at the University of Amsterdam.

In the original code of Waveshare, there is explicit reference to the home-directory '/home/ws', while on the UGV itself the home-directory can be found at '/home/jetson'.
At your own machine, you will have own configuration.

Note that the situation is different once you have started the docker image (at the UGV or at your laptop). Note that most of the [Waveshare documentation](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2) assumes that you are inside the docker image. The Intelligent Robotics Lab also reversed engineered a version of the docker image, also available on [github](https://github.com/IntelligentRoboticsLab/ugv_rover_docker).

## 1. Environment

- pc software：Ubuntu 22.04, ROS2 Humble
- ugv Version：UGV Rover

The workspace should also work for the UGV Beast, although this is not tested.

## 2. Installation

- The history of the this workspace can be summarized：https://github.com/DUDULRX/ugv_ws/tree/ros2-humble -> https://github.com/waveshareteam/ugv_ws.git -> https://github.com/IntelligentRoboticsLab/ugv_ws.git

- Start with creating a clone of this repository:

    ```jsx
    cd
    git clone -b ros2-humble-develop https://github.com/IntelligentRoboticsLab/ugv_ws.git
    ```

- Before your first compilation, you should first install a number of dependencies

  ```jsx
  cd ~/ugv_ws
  source install_additional_ros_humble_packages.sh
  ```

- install_additional_ros_humble_packages.sh content:

   ```jsx
   sudo apt-get install ros-humble-nav2-msgs ros-humble-map-msgs
   sudo apt-get install ros-humble-nav2-costmap-2d
   sudo apt-get install ros-humble-rosbridge-suite
   sudo apt-get install ros-humble-nav2-bringup
   sudo apt-get install ros-humble-usb-cam ros-humble-depthai-*
   sudo apt-get install ros-humble-joint-state-publisher-*
   sudo apt-get install ros-humble-robot-localization
   sudo apt-get install ros-humble-imu-tools
   sudo apt-get install ros-humble-cartographer-ros
   sudo apt-get install ros-humble-apriltag ros-humble-apriltag-msgs ros-humble-apriltag-ros
   sudo apt-get install ros-humble-ros-gz
      ```
- First compilation (it starts cleans the workspace, so you start from scratch. It also defines a number of environment variables, which is needed only once):

  ```jsx
  cd ~/ugv_ws
  . build_first.sh
  ```

- build_first.sh content:

  ```jsx
  cd ~/ugv_ws/
  colcon clean workspace
  colcon build --cmake-args -Wno-dev --packages-select cartographer costmap_converter_msgs explore_lite --executor sequential
  colcon build --cmake-args -Wno-dev --packages-select openslam_gmapping slam_gmapping --executor sequential
  colcon build --cmake-args -Wno-dev --packages-select ldlidar rf2o_laser_odometry robot_pose_publisher teb_msgs --executor sequential
  colcon build --cmake-args -Wno-dev --packages-select ugv_base_node ugv_interface ugv_bringup ugv_chat_ai ugv_description ugv_gazebo ugv_nav ugv_slam ugv_tools ugv_vision ugv_web_app --executor sequential
  chmod +x ~/ugv_ws/src/ugv_else/vizanti/vizanti_server/scripts/*.py
  colcon build --cmake-args -Wno-dev --packages-select vizanti vizanti_cpp vizanti_demos vizanti_msgs vizanti_server --executor sequential

  # echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc ## already done in install_additional_ros_humble_packages.sh
  # echo ""export ROS_DOMAIN_ID=42" >> ~/.bashrc # change ROS_DOMAIN_ID to your group-number
  echo "source ~/ugv_ws/install/setup.bash" >> ~/.bashrc
  echo "export UGV_MODEL=ugv_rover" >> ~/.bashrc
  echo "export LDLIDAR_MODEL=ld19" >> ~/.bashrc
  source ~/.bashrc
  ```

- Daily compilation after first build:

  ```jsx
  cd ~/ugv_ws
  . build_common.sh
  ```

- build_common.sh content

  ```jsx
  cd ~/ugv_ws
  colcon build --cmake-args -Wno-dev --packages-select cartographer costmap_converter_msgs explore_lite --executor sequential
  colcon build --cmake-args -Wno-dev --packages-select openslam_gmapping slam_gmapping --executor sequential
  colcon build --cmake-args -Wno-dev --packages-select ldlidar rf2o_laser_odometry robot_pose_publisher teb_msgs  --executor sequential
  colcon build --cmake-args -Wno-dev --packages-select ugv_base_node ugv_interface ugv_bringup ugv_chat_ai ugv_description ugv_gazebo ugv_nav ugv_slam ugv_tools ugv_vision ugv_web_app --executor sequential
  colcon build --cmake-args -Wno-dev --packages-select vizanti vizanti_cpp vizanti_demos vizanti_msgs vizanti_server --executor sequential
  source install/setup.bash
        ```

- Compile apriltag

  ```jsx
  cd ~/ugv_ws
  . build_apriltag.sh
  ```

- build_apriltag.sh content

  ```jsx
  sudo apt install libopencv-dev=4.5.4+dfsg-9ubuntu4 -y
  cd ~/ugv_ws
  colcon build --cmake-args -Wno-dev --packages-select apriltag apriltag_msgs apriltag_ros --executor sequential
  ```

- Python3 Library：


  ```jsx
  cd ~/ugv_ws/
  python3 -m pip install -r requirements.txt
  echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc
  ```

- requirements.txt content

  ```jsx
  pyserial
  flask
  mediapipe
  requests
  pygame
  ```

## Modules inside the workspace

This are the alphabetic-list of the modules in the ugv_ws, which is not the logical build-up as given in [Waveshare documentation](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2).

### [ugv_main](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/): Main functions

- [ugv_base_node](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/ugv_base_node): filter to estimate the odometry

  Unfortunately no tutorial from WaveShare.

- [ugv_bringup](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/ugv_bringup): drive, control with two-wheel differential kinematics

  A tutorial is available from Waveshare: [Tutorial 3: Use Joystick or Keyboard Control](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2_3._Use_Joystick_or_Keyboard_Control)

- [ugv_chat_ai](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/ugv_chat_ai): web ai interaction

  A tutorial is available from Waveshare: [Tutorial 8: Web Natural Language Interaction](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2_8._Web_Natural_Language_Interaction)

- [ugv_description](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/ugv_description): Kinematic Model

  A tutorial is available from Waveshare: [Tutorial 2: RViz View Product Model](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2_2._RViz_View_Product_Model)

- [ugv_gazebo](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/ugv_gazebo): simulation

  A tutorial is available from Waveshare: [Tutorial 11: Gazebo Simulation Debugging](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2_11._Gazebo_Simulation_Debugging)

- [ugv_interface](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/ugv_interface): Information interface

  A tutorial is available from Waveshare: [Tutorial 10: Command Interaction](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2_10._Command_Interaction)

- [ugv_nav navigation](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/ugv_nav): Nav2 toolbox

  A tutorial is available from Waveshare: [Tutorial 6: Auto Navigation](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2_6._Auto_Navigation)

- [ugv_slam](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/ugv_slam): Simultaneous Localization and Mapping

  This module is described in three tutorials from Waveshare:
  - [Tutorial 4: 2D Mapping based on LiDAR](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2_4._2D_Mapping_Based_on_LiDAR)
  - [Tutorial 5: 3D Mapping based on Depth Camera](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2_5._3D_Mapping_Based_on_Depth_Camera)
  - [Tutorial 7: Navigation and SLAM Mapping](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2_7._Navigation_and_SLAM_Mapping)

- [ugv_tools](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/ugv_tools): Joystick tool

  This module is used in Waveshare's [Tutorial 3: Use Joystick or Keyboard Control](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2_3._Use_Joystick_or_Keyboard_Control)

- [ugv_vision](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/ugv_vision): visual interaction

  Actually, this module is already used in [Tutorial 4: 2D Mapping based on LiDAR](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2_4._2D_Mapping_Based_on_LiDAR)

- [ugv_web_app](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/ugv_web_app): web-based control tool

  A tutorial is available from Waveshare: [Tutorial 9: Web-based control tool](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2_9._Web-based_control_tool)

### ugv_else ( ugv_main dependencies)

- [apriltag_ros](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_else/apriltag_ros): ROS2 wrapper for apriltag detection

  dependence from [ugv_vision](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/ugv_vision)

- [cartographer][https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_else/cartographer]: real-time simultaneous localization, see [ROS Humble documentation][https://docs.ros.org/en/humble/p/cartographer/]

  dependence from [ugv_slam](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/ugv_slam)

- [costmap_converter](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_else/costmap_converter): converts occupied costmap2d cells to primitive types.

  dependence from [ugv_nav navigation](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/ugv_nav)

- [emcl_ros2](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_else/emcl2_ros2): Monte Carlo localization

  dependence from [ugv_slam](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/ugv_slam)

- [explore_lite](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_else/explore_lite)

  dependence from [ugv_nav navigation](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/ugv_nav)

- [gmapping](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_else/gmapping): ROS2 wrapper for OpenSlam's Gmapping.

  dependence from [ugv_slam](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/ugv_slam)

- [ldlidar](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_else/ldlidar): drivers for LiDAR products sold by Shenzhen LDROBOT Co.

  dependence from [ugv_bringup](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/ugv_bringup)

- [rf2o_laser_odometry](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_else/rf2o_laser_odometry): estimate the planar motion from lidar measurements

  dependence from [ugv_bringup](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/ugv_bringup)

- [robot_pose_publisher](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_else/robot_pose_publisher): ROS2 wrapper which publishes  the transform between the /base_link frame and the /map frame as a pose message

  dependence from [ugv_nav navigation](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/ugv_nav)

- [teb_local_planner](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_else/teb_local_planner): optimizes the robot's trajectory after planning

  dependence from [ugv_nav navigation](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/ugv_nav)

- [vizanti](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_else/vizanti): web-based visualization and control tool for outdoor robots


## 3. Usage

The ugv-packages have several parameters, including the hardware configuration, specified as environment variable:

- car model optional rasp_rover, ugv_rover, ugv_beast

- lidar model optional ld06, ld19 (default), stl27l

You have also parameters given as ros-argument, such as:

- use_rviz optional true, false (default)

Graphical tools like rviz only work when you have a display. So, run those tools on your laptop, and the drivers on the UGV Rover


### View model joints

See also [Tutorial 2: RViz View Product Model](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2_2._RViz_View_Product_Model)

- Specify your UGV-model

  ```jsx
     export UGV_MODEL=ugv_rover
  ```

- start up on (executed on your laptop)

   ```jsx
   ros2 launch ugv_description display.launch.py use_rviz:=true
   ```

  ![image.png](images/Ugv_rover.png)

### Drive the car

See also [Tutorial 3: Use Joystick or Keyboard Control](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2_3._Use_Joystick_or_Keyboard_Control)

- Start the car (executed at the UGV Rover)

   ```jsx
   ros2 run ugv_bringup ugv_driver
   ```

- Make sure that your UGV Rover is on the ground. Alternatively lift the wheels from the table by placing the base on a small box.

  ```jsx
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear : {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
  ```

  Control the light data 0-255 data[0] control the light IO4 near the oak camera data[1] control the light IO5 near the usb camera

        ```jsx
        ros2 topic pub /ugv/led_ctrl std_msgs/msg/Float32MultiArray "{data: [0, 0]}" -1
        ```

- Start LiDAR  (executed at the UGV Rover)

  ```jsx
  export LDLIDAR_MODEL=LD19
  ```

  ```jsx
  ros2 launch ugv_bringup bringup_lidar.launch.py use_rviz:=false
  ```


- Joystick, keyboard control

    - Joystick control (can be executed at the UGV Rover or your own laptop, as long as you are in the same ROS_DOMAIN_ID)

        ```jsx
        ros2 launch ugv_tools teleop_twist_joy.launch.py
        ```

    - keyboard control

        ```jsx
        ros2 run ugv_tools keyboard_ctrl
        ```


The other functionalities are explained in the documention of the each of the [ugv packages](https://github.com/IntelligentRoboticsLab/ugv_ws/tree/ros2-humble-develop/src/ugv_main/) and the [Waveshare documentation](https://www.waveshare.com/wiki/UGV_Rover_Jetson_Orin_ROS2).


instant command

setup
docker exec -it -e ROS_DOMAIN_ID=4 -e ROS_LOCALHOST_ONLY=0 ugv_jetson_ros_humble bash
source /opt/ros/humble/setup.bash && source /home/ws/ugv_ws/install/setup.bash

host (laptop)

cd ~/ugv_ws && bash start_rover.sh avoidance:=true avoidance_drive:=true goal_distance:=10.0 2>&1 | tee ~/run10h_launch.log

container (rover)

logs:-


timeout 5 ros2 topic hz /scan
ros2 topic echo /obstacle_avoidance_adv/status --once --full-length | grep -oE '"(readiness|command_topic)": "[^"]*"'
ros2 topic echo /obstacle_avoidance_adv/status --full-length | tee /home/ws/run10h_status.txt | grep --line-buffered -oE '"(state|reason)": "[^"]*"'

run command (different terminal on rover)

ros2 service call /obstacle_avoidance_adv/start std_srvs/srv/Trigger '{}'


stop command

ros2 service call /obstacle_avoidance_adv/stop std_srvs/srv/Trigger '{}'
