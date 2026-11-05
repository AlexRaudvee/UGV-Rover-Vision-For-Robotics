# object_detection

Detects black/near-black blobs in `/oak/rgb/image_rect/compressed`, uses RGB-registered losslessly compressed depth from `/oak/stereo/image_raw/compressedDepth` to calculate their physical width, and uses a nominal **23–30 cm** size band with bounded measurement uncertainty. Both raw metric depth and PNG compressedDepth transport are supported. RGB transport names alone do not establish which topic contains depth.

```bash
ros2 launch object_detection detection.launch.py
```

Outputs are `/object_detection/detections`, `/object_detection/debug/compressed`, and `/object_detection/depth/compressed` (a normal colorized JPEG for Vizanti). `polar_hist` produces this detector's live histogram and measured 35 cm gap candidates on `/polar_hist/object/...`.

See [the complete perception guide](../polar_hist/README.md) for setup, geometry, calibration, Vizanti, parameters, output schema, and tests.

Depth ranges use a per-blob, time-based EMA, with `depth_ema_tau_s: 0.35` by default. Set it live to a larger value for more smoothing, or to `0.0` to disable EMA. Missing depth stays unknown. See the perception guide for the formula and tuning commands.
