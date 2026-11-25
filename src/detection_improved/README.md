**Live rover input:** one depth detector now supplies both dashboard branches from
the shared WLS metric stream `/wls_filtered/compressedDepth`. Its calibration is published on
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
| obstacle detections (depth by default) | `/obstacle_detection/detections` | `/detection_improved/obstacle/detections` |
| depth gaps | `/polar_hist/object/gaps` | `/detection_improved/polar_hist/object/gaps` |
| obstacle gaps | `/polar_hist/obstacle/gaps` | `/detection_improved/polar_hist/obstacle/gaps` |

Debug images, histograms and scans follow the same pattern.
The default launch runs `/detection_improved/object/detector` once and republishes
the same result and images under `/detection_improved/obstacle/*`. The histogram
retains both sets of topics for existing displays. `single_depth:=false` restores
the two independent depth nodes; `obstacle_range_source:=lidar` selects the
independent LiDAR node. The Session 3 map uses only the improved object/depth stream.

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

## Navigation evidence

Detection JSON additionally contains `navigation` (schema 1), consumed by the
[Session 3 preview map](../l2s3_curling/README.md#obstacle-map-preview).
Its validity is independent of the legacy display: it requires both acquisitions
within 1 s of the ROS clock, RGB/depth skew at most 0.15 s, a fitted/held floor,
and a timestamp/frame-matched WLS measured-depth support mask. A fresh preview
can therefore have unusable navigation evidence; the reason is explicit in JSON.
Clock offsets or missing masks must be resolved before using the map for planning.

`free_points` are sparse measured floor patches, never whole image sectors or
rays. WLS-filled holes and values differing by over 5 cm from the current source
measurement cannot certify free floor. Unmeasured object silhouettes also mask
free evidence. `occupied_points` preserve above-floor depth hits, including objects
outside the colour detector's target class. `blocker_faces` include every measured
blocker regardless of `accepted`, plus range/width uncertainty; only the visible
front face is projected, with no invented box depth. Empty lists do not prove
free space. Points are returned to the nominal optical TF frame after the existing
mount-pitch correction, and carry the depth acquisition timestamp.

The 0.04–0.45 m above-floor occupancy band is for this initial rover/box preview;
it does not establish clearance for every possible shape or obstacle height.
