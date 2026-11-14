# Lab 2 Session 2: localisation

Provides pan-tilt scanning, marker geometry, marker maps and basic and weighted localisation. Depends on l2s1_marker_detection. Enter measured marker positions in config/marker_map.yaml and measure the marker inner border size before use. Launch with `ros2 launch l2s2_localisation marker_scan.launch.py marker_size_m:=0.15`. The scanner assumes the rover stays stationary during a scan.
