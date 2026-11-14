**Live rover input:** both dashboard branches now default to the shared WLS
metric stream `/wls_filtered/compressedDepth`. Its calibration is published on
`/wls_filtered/camera_info`. Select `obstacle_range_source:=lidar` to use the
independent LiDAR branch described below. See the
[depth configuration guide](../ugv_main/ugv_vision/DEPTH.md) for filter settings.

# detection_improved

The improved detectors from the `detection-improved` branch, as an **alternative** to
`object_detection`, `obstacle_detection` and `polar_hist`. The rover launch uses
this package by default; the original packages remain available through explicit
launch arguments.

What is different from the default detectors:
- Depth: a floor plane is fitted to the depth image, and only objects standing on the
  floor are kept (`ground_check`); grey boxes without dark colour are found from depth.
- Both: pieces of one object are merged into one detection; the distance jump gate
  grows with distance; a blob cut off by the image border keeps its distance.
- Every object with a measured distance is in `detections` (and, for older consumers,
  also in `blockers`); `accepted` only says whether its width is in the target band.
- Gaps can be bounded by any measured object, not only target-width ones.

## Topics

| | Default packages | detection_improved |
|---|---|---|
| depth detections | `/object_detection/detections` | `/detection_improved/object/detections` |
| LiDAR detections | `/obstacle_detection/detections` | `/detection_improved/obstacle/detections` |
| depth gaps | `/polar_hist/object/gaps` | `/detection_improved/polar_hist/object/gaps` |
| LiDAR gaps | `/polar_hist/obstacle/gaps` | `/detection_improved/polar_hist/obstacle/gaps` |

Debug images, histograms and scans follow the same pattern.

## Running

    ros2 launch detection_improved detection.launch.py   # depth:=false / lidar:=false
    bash start_rover.sh --build                        # default rover detection stack

To drive `obstacle_avoidance` with it, set its `detections_topic` and `gaps_topic`
parameters to the topics above.

Bag replay: `DETECTORS=improved bash run_bag_eval.sh <run> <bag>` runs these nodes
under the default topic names, so the evaluation scripts work unchanged.

The live OAK profiles apply a 5 degree downward pitch correction, matching the
recorded bag and the current mounting (TF models the camera as level). Remove
this correction if the mounting pitch is incorporated into TF later.

Live profiles set `check_acquisition_time: false`: synchronized frames are not
rejected because their header timestamps differ from the host ROS clock. Live detector profiles also set
`synchronize_inputs: false`: the newest RGB is paired immediately with the latest
received range frame, regardless of header timestamp differences. Stopped streams
expire using local monotonic receipt time and `stale_after_s`. Set
`synchronize_inputs: true` to restore matching within `sync_tolerance_s`. TF uses the latest available pose in this mode.
Set `check_acquisition_time: true` to restore absolute acquisition-age checks.
