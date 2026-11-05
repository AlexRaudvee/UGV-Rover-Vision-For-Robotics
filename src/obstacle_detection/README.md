# obstacle_detection

Detects black/near-black blobs from OAK RGB (or a calibrated USB camera), projects synchronized LiDAR returns into their contours using TF, measures their physical width, and uses a nominal **23–30 cm** size band with bounded measurement uncertainty. Missing range measurements do not fall back to an assumed object width.

```bash
ros2 launch obstacle_detection detection.launch.py camera:=oak
# Or use the existing calibrated USB camera and LiDAR:
ros2 launch obstacle_detection detection.launch.py camera:=usb
```

Inputs and thresholds live in `config/detector.yaml`, `config/oak.yaml`, and `config/usb.yaml`. Outputs are `/obstacle_detection/detections` (versioned JSON) and `/obstacle_detection/debug/compressed` (annotated JPEG). Histograms and measured 35 cm passage candidates are now provided by `polar_hist` on `/polar_hist/obstacle/...`.

See [the complete perception guide](../polar_hist/README.md) for building all three packages, input topics, calibration, width/gap equations, Vizanti layout, parameters, output schema, limitations, and tests. The depth-based detector is the separate `object_detection` package.
