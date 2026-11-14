# Lab 2 Session 1: marker detection

Provides baseline and improved detection presets, the detector ROS node and offline evaluation. The default marker family is tagStandard41h12. Start with `ros2 launch l2s1_marker_detection marker_detection.launch.py preset:=baseline` or use `preset:=improved`. The rover launch starts baseline detection by default.

Capture with `/marker_detector/capture`; compare saved frames with `python3 -m l2s1_marker_detection.evaluate --help`. Build the bundled apriltag library with its Python wrapper before building this package.
