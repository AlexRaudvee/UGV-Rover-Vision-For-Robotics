# detection_improved

The improved detectors from the `detection-improved` branch, as an **alternative** to
`object_detection`, `obstacle_detection` and `polar_hist`. Those packages keep their
my-ugv behaviour, and `obstacle_avoidance` uses them by default.

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
    bash start_rover.sh improved_detection:=true        # on the rover, next to the defaults

To drive `obstacle_avoidance` with it, set its `detections_topic` and `gaps_topic`
parameters to the topics above.

Bag replay: `DETECTORS=improved bash run_bag_eval.sh <run> <bag>` runs these nodes
under the default topic names, so the evaluation scripts work unchanged.
