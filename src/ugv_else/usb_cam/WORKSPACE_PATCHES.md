# Workspace camera driver

Pinned source: https://github.com/ros-drivers/usb_cam/tree/0.8.1

Archive: https://codeload.github.com/ros-drivers/usb_cam/tar.gz/refs/tags/0.8.1

SHA256: `c2492808bf7b7f7df65f15b361f9b5707f80e40c9eea035d20961244d86e6875`

This is the same 0.8.1 version previously supplied by `/opt/ros/humble`. The
workspace overlay changes only the epoch/monotonic timestamp conversion in
`include/usb_cam/utils.hpp`: `tv_usec` already contains microseconds, so dividing
it by 1000 discards almost an entire fractional second. That caused valid
AprilTag frames to fail the rover's one-second freshness check.

The correction matches upstream:
https://github.com/ros-drivers/usb_cam/issues/327
https://github.com/ros-drivers/usb_cam/blob/main/include/usb_cam/utils.hpp

`test/test_usb_cam_timestamps.cpp` injects fixed wall/monotonic clock values and
checks fractional seconds without opening any device or changing system time.
The binary system installation is left untouched. `start_rover.sh --build`
builds this overlay before starting the camera supervisor.
