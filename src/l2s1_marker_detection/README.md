# Lab 2 Session 1: marker detection

Provides detection presets, the detector ROS node and offline evaluation. The default marker family is tagStandard41h12. All live entry points (rover, standalone detector and Session 2 scanner) default to `sharpen`: one sharpened detection pass at the input resolution. Launch with `ros2 launch l2s1_marker_detection marker_detection.launch.py`. Select `preset:=baseline` or `preset:=improved` explicitly for comparisons; the rover equivalent is `marker_preset:=baseline` or `marker_preset:=improved`.

Capture with `/marker_detector/capture`; compare saved frames with `python3 -m l2s1_marker_detection.evaluate --help`. Build the bundled apriltag library with its Python wrapper before building this package.

Both baseline and improved use `/image_rect` by default. Select `/image_raw` with `image_topic:=/image_raw` when using the `undistort` preset; do not undistort an already rectified image. The Session 2 scan launch retains `/image_raw` for its raw-camera geometry model.
