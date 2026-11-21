# Lab 2 field notes — 18 Nov 2026 (rover 4)

Data (not in git): images in `runs/marker_captures`, `runs/marker_eval`; rosbags in `runs/bags`.

Field frame: origin = centre spot, +x towards the target goal, +y left. Units m.

## Tags (tagStandard41h12, white inner square 14.0–14.1 cm → marker_size_m 0.14)

Map on the rover (`src/l2s2_localisation/config/marker_map.yaml`, evenly spaced layout):

| ID | x | y | Note |
|---|---|---|---|
| 0 | −0.55 | 0.0 | measured: 2.67 m in front of start (−3.25,0), 0.5 m behind centre |
| 1 | −1.5 | −3.0 | Zamo-consistent; its camera bearing looked off (camera issue, not map) |
| 2 | −1.5 | 3.0 | |
| 3 | 1.55 | −3.0 | right side seen from centre facing +x (confirmed) |
| 4 | 1.55 | 3.0 | left side |
| 5 | 4.5 | 0.0 | in the goal (Zamo 4.57 from centre) |

## Session 1 — detection robustness (runs/marker_captures, runs/marker_eval)

Tag 0 only, 61 test images (1/3/5/7 m × 0/30/60°), hits per preset:
baseline 31 (51 %, 27 ms) · clahe 31 (28 ms) · sharpen 35 (57 %, 31 ms) · upscale 33 (109 ms) · improved 33 (142 ms).
Key cases: 3 m/60° baseline 0/5 → sharpen 4/5, upscale 3/5; 5 m/30° only improved 2/7; 5 m/60° and 7 m all 0.
Tag ≈ 8 px at 5 m, ≈ 5 px at 7 m. fail_occluded 0/5, fail_edge 2/5 (all presets).
`tuned` = baseline (OpenCV-only params), `undistort` = baseline (input already /image_rect).
"Unexpected" detections = real background tags (IDs 1, 2), not false positives.

## Session 2 — distance (bag `center_sweep2_221736`, rover at (0,0) facing +x, true heading ≈ −9.2°)

| Tag | Zamo | Camera (0.14 m) | Error | Default size 1 m |
|---|---|---|---|---|
| 1 | 3.43 | 3.42 | −0.01 | 24.4 |
| 2 | 3.30 | 3.51 | +0.21 | 25.1 |
| 3 | 3.42 | 3.57 | +0.15 | 25.5 |
| 4 | 3.45 | 3.60 | +0.15 | 25.7 |
| 5 | 4.57 | 5.16 | +0.59 | 36.9 |

Tags ≈ 8 px wide; error grows with distance (σr ∝ d²).

## Session 2 — triangulation from the centre (true (0,0))

| Markers | Basic (ranges) error | Advanced (range+bearing) error |
|---|---|---|
| 3,4 | 0.41 | 0.03 |
| 2,3 | 1.08 | 0.02 |
| 3,4,5 | 0.56 | 0.09 |
| 2,3,4,5 | 0.55 | 0.05 |
| 1–5 | 0.45 | 0.36 (tag 1 bearing off) |

Advanced heading estimate −9.2° (rover not exactly facing +x).

## Session 2 — driving bags (runs/bags)

- `drive_straight_223727`: start (0,0) heading ≈ −9.2°, 2.0 m straight at 0.15 m/s.
  End by Zamo (tag 1 4.47, tag 5 2.46) → (2.06, −0.29); odometry predicted (1.97, −0.32).
- `drive_arc2_225442`: start (−3.25, 0) facing +x; commanded R = 4 m arc, rover went ~straight
  (5.6° turn); passed tag 0's box within 2–5 mm (not touched, per observer). Odom 4.6 m.
- `drive_arc_sweep_225831`: from the end of arc2; R = 1.5 m commanded, turned only 30° in 2.6 m;
  camera swept continuously ±120° at 30°/s. Final position by Zamo (tag 4 3.20, tag 2 2.30)
  → (−0.79, 0.81). Odometry over arc2 + this run implies ~(3.2, 1.9): ~4 m discrepancy, unexplained
  (wheel slip / start heading?) — analyse offline.
- `center_sweep_220250`: OLD tag layout, do not use.

Rover turns at ≈ 1/3 of the commanded angular rate (0.04 rad/s → none, 0.1 rad/s → ~0.03) —
matters for Session 3.

## Scripts (scripts/lab2)
sweep_pan.py (stepped sweep), sweep_continuous.py, drive_straight.py, drive_arc.py,
bag_tags.py (tags per pan angle), bag_ranges.py (per-tag distance/bearing from a bag).
Pan tracker must be paused while a sweep script runs (it owns /localization/pan_tilt_command);
restart: `ros2 run l2s2_localisation pan_tracker --ros-args -r __node:=marker_pan_tracker --params-file .../pan_tracking.yaml`.

## Triangulation tested on the bags (21 Nov, scripts/lab2/test_triangulation_bags.py)

Settled frames only (pan unchanged ≥ 0.8 s), tag size 0.14 m, map above with tag 0 at (−0.55, 0).
"Scan fix" = per tag the median over the frames nearest the image centre, solved once.

| Test (ground truth) | Tags | Line (yours) | Trig (advanced) | Trig + rejection | Live (team) |
|---|---|---|---|---|---|
| Centre spot (0, 0), stepped sweep | 1–5 | 0.47 m | **0.07 m** | 0.07 m (nothing dropped) | 0.47 m |
| Start of 2nd drive (−3.25, 0) | 0, 1 | 0.17 m | 0.21 m | 0.21 m | 0.17 m |
| End of curve (−0.79, 0.81), continuous sweep | 1, 2 | 0.61 m | **0.14 m** | 0.14 m | fails |
| End of straight drive (2.06, −0.29) | 5 only | — | — | — | — (1 tag: needs EKF) |

Per single image (2+ tags in one frame): centre 33 images, median line 0.26 m / trig 0.35 m;
start of 2nd drive 21 images, line 0.17 m / trig 0.21 m.
Correction: the earlier "tag 1 bearing 26° off" came from frames taken while the pan servo was
still moving; with settled frames tag 1 agrees with the map (−106° measured, −107° expected).

## Still open
- Analyse drive bags with the EKF (rover_additions/ekf*.py; rover build lacks /marker_ranges/observations).
- Course rosbag (apriltag_localization.zip): rosbag evaluation + pan-tilt on it.
- Report writing (Session 1 improvements/comparison, Session 2 sections). Fix report: penalty mark x = 3.2.
- Session 3 (curling).
