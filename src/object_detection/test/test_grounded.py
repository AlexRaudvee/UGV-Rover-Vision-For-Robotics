"""Ground check: surfaces, merging, floor contact and fallbacks on synthetic depth scenes."""

import json

import numpy as np
from object_detection.floor import prior_plane
from object_detection.grounded import (CONTACT_UNKNOWN, FLAT, ground_objects, NO_FLOOR,
                                       NOT_ON_FLOOR, working_points)
from object_detection.perception import Blob, CameraModel, DetectorSettings, measure_blob
from test_floor import camera_pose, floor_depth, HEIGHT, MATRIX, WIDTH

CAMERA = CameraModel(WIDTH, HEIGHT, MATRIX, np.zeros(5))
SETTINGS = DetectorSettings(ground_check=True)
GREY = np.full((HEIGHT, WIDTH, 3), 90, np.uint8)
RED = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
RED[:, :, 2] = 200


def blob(x, y, w, h):
    """Rectangular colour region."""
    contour = np.array([[[x, y]], [[x + w - 1, y]], [[x + w - 1, y + h - 1]],
                        [[x, y + h - 1]]], np.int32)
    return Blob((x, y, w, h), contour, 0.0, -0.1, 0.1, 0.2, float(w * h), False)


def heights(depth):
    """Height above the true floor at the working resolution."""
    return prior_plane(*camera_pose(0.20, 5.0)).height(working_points(depth, MATRIX))


def run(blobs, depth, bgr=GREY, settings=SETTINGS):
    return ground_objects(blobs, bgr, depth, heights(depth), CAMERA, settings)


def standing_box(depth, x0, x1, distance, top_row):
    """Paint a vertical face at `distance` from top_row down to where it meets the floor."""
    floor = floor_depth(0.20, 5.0)
    for row in range(top_row, HEIGHT):
        if np.isfinite(floor[row, x0]) and floor[row, x0] < distance:
            break
        depth[row, x0:x1] = distance
    return depth


def test_box_on_floor_is_kept_beyond_its_dark_part():
    depth = standing_box(floor_depth(0.20, 5.0), 140, 180, 1.5, 60)
    # Only the upper half is dark; the object is the whole surface.
    objects, rejected, _ = run([blob(140, 60, 40, 25)], depth)
    assert len(objects) == 1 and not rejected and objects[0].seed == 'colour'
    x, y, w, h = objects[0].blob.bbox
    assert abs(x - 140) <= 2 and abs(w - 40) <= 2 and y + h > 85 + 10
    assert abs(objects[0].estimate.axial_m - 1.5) < 0.01


def test_floating_object_is_not_on_floor():
    depth = floor_depth(0.20, 5.0)
    depth[95:110, 140:180] = 1.5   # hangs ~20 cm above the floor, floor visible below
    _, rejected, _ = run([blob(140, 95, 40, 15)], depth)
    assert [reason for _, reason in rejected] == [NOT_ON_FLOOR]


def test_shadow_on_floor_is_flat():
    depth = floor_depth(0.20, 5.0)
    objects, rejected, _ = run([blob(100, 180, 60, 30)], depth)
    assert not objects and [reason for _, reason in rejected] == [FLAT]


def test_box_in_front_of_dark_background_is_split_off():
    depth = standing_box(floor_depth(0.20, 5.0), 60, 260, 4.5, 20)   # far "curtain"
    depth = standing_box(depth, 140, 180, 1.5, 60)                    # near box
    objects, _, _ = run([blob(60, 20, 200, 90)], depth)
    near = [item for item in objects if item.estimate.axial_m < 2]
    assert len(near) == 1
    x, _, w, _ = near[0].blob.bbox
    assert abs(x - 140) <= 2 and abs(w - 40) <= 2


def test_no_depth_below_object_is_contact_unknown():
    depth = standing_box(floor_depth(0.20, 5.0), 140, 180, 1.5, 60)
    rows = np.flatnonzero(depth[:, 160] == 1.5)
    depth[rows[-1] - 30:, 130:190] = np.nan   # lower part and floor below: no depth
    _, rejected, _ = run([blob(140, 60, 40, 25)], depth)
    assert [reason for _, reason in rejected] == [CONTACT_UNKNOWN]


def test_pieces_of_one_surface_become_one_object():
    depth = standing_box(floor_depth(0.20, 5.0), 140, 180, 1.5, 60)
    objects, _, _ = run([blob(140, 60, 40, 12), blob(140, 80, 40, 12),
                         blob(140, 100, 40, 12)], depth)
    assert len(objects) == 1


def test_surfaces_split_by_a_small_step_are_merged():
    depth = standing_box(floor_depth(0.20, 5.0), 140, 180, 1.5, 60)
    depth[60:80, 140:180] = 1.6   # upper part 10 cm further: a depth jump, same object
    objects, _, _ = run([blob(140, 60, 40, 40)], depth)
    assert len(objects) == 1 and objects[0].blob.bbox[1] <= 62


def test_no_depth_below_but_object_reaches_floor_is_kept():
    depth = standing_box(floor_depth(0.20, 5.0), 140, 180, 1.5, 60)
    rows = np.flatnonzero(depth[:, 160] == 1.5)
    depth[rows[-1] + 1:, 130:190] = np.nan   # only the floor below has no depth
    objects, rejected, _ = run([blob(140, 60, 40, 25)], depth)
    assert len(objects) == 1 and not rejected


def test_sparse_depth_falls_back_to_the_colour_region():
    depth = standing_box(floor_depth(0.20, 5.0), 120, 200, 0.8, 40)
    face = depth == 0.8
    streaks = np.zeros_like(face)
    streaks[::6, :] = True   # only thin streaks of depth survive on the dark face
    depth[face & ~streaks] = np.nan
    objects, _, _ = run([blob(120, 40, 80, 130)], depth)
    assert len(objects) == 1
    assert objects[0].blob.bbox == (120, 40, 80, 130)
    assert abs(objects[0].estimate.axial_m - 0.8) < 0.02


def test_small_pieces_count_as_dark_and_alone_stay_silent():
    depth = standing_box(floor_depth(0.20, 5.0), 140, 180, 1.5, 60)
    pieces = [blob(140, row, 40, 3) for row in range(60, 100, 4)]
    for piece in pieces:
        piece.small = True   # each fails the size filters on its own
    objects, rejected, unmeasured = run(pieces, depth, RED)   # red: depth alone would fail
    assert len(objects) == 1 and objects[0].seed == 'colour'
    assert not rejected and not unmeasured
    speck = blob(300, 20, 5, 5)
    speck.small = True       # no depth: silently dropped, not 'no distance'
    nothing = np.full(depth.shape, np.nan)
    objects, rejected, unmeasured = ground_objects([speck], GREY, nothing, heights(depth),
                                                   CAMERA, SETTINGS)
    assert not objects and not rejected and not unmeasured


def test_depth_alone_finds_a_grey_box_but_not_a_red_one():
    depth = standing_box(floor_depth(0.20, 5.0), 140, 180, 1.5, 60)
    objects, rejected, _ = run([], depth)
    assert len(objects) == 1 and objects[0].seed == 'depth' and not rejected
    assert abs(objects[0].estimate.axial_m - 1.5) < 0.01
    assert run([], depth, RED)[0] == []


def test_depth_alone_ignores_tall_walls():
    depth = standing_box(floor_depth(0.20, 5.0), 0, WIDTH, 2.5, 0)   # wall, top out of view
    assert run([], depth)[0] == []


def test_output_is_json_serializable():
    depth = standing_box(floor_depth(0.20, 5.0), 140, 180, 1.5, 60)
    item = run([], depth)[0][0]
    detection = measure_blob(item.blob, item.estimate.axial_m, 'depth', SETTINGS,
                             item.estimate)
    json.dumps({'d': detection.to_dict(), 'outline': item.outline}, allow_nan=False)


def test_small_piece_cannot_carry_a_wall_past_the_size_limits():
    depth = standing_box(floor_depth(0.20, 5.0), 20, 300, 2.0, 70)   # 2 m wide board
    piece = blob(150, 70, 6, 6)
    piece.small = True
    objects, rejected, _ = run([piece], depth)
    assert not objects and not rejected


def test_without_floor_everything_is_rejected():
    depth = standing_box(floor_depth(0.20, 5.0), 140, 180, 1.5, 60)
    objects, rejected, _ = ground_objects([blob(140, 60, 40, 25)], GREY, depth, None,
                                          CAMERA, SETTINGS)
    assert not objects and [reason for _, reason in rejected] == [NO_FLOOR]


def test_dark_piece_resting_on_an_object_joins_it():
    depth = standing_box(floor_depth(0.20, 5.0), 140, 180, 1.5, 100)   # "tracks" on floor
    depth[70:96, 130:190] = 1.5   # "body" above them, a 4 px gap of floor-depth between
    stacked = DetectorSettings(ground_check=True, merge_max_gap_px=15)
    objects, rejected, _ = run([blob(140, 100, 40, 15), blob(130, 70, 60, 26)], depth,
                               RED, stacked)
    assert len(objects) == 1 and not rejected
    x, y, w, h = objects[0].blob.bbox
    assert y <= 72 and x <= 132 and x + w >= 186
    # Without stacking (merge_max_gap_px = 0) the body is rejected as off the floor.
    objects, rejected, _ = run([blob(140, 100, 40, 15), blob(130, 70, 60, 26)], depth, RED)
    assert len(objects) == 1 and [reason for _, reason in rejected] in (
        [NOT_ON_FLOOR], [CONTACT_UNKNOWN])


def test_contact_limit_without_depth_below_grows_with_distance():
    depth = standing_box(floor_depth(0.20, 5.0), 140, 180, 3.0, 80)
    rows = np.flatnonzero(depth[:, 160] == 3.0)
    depth[rows[-1] - 5:, 130:190] = np.nan   # base and floor below it: no depth
    fixed = DetectorSettings(ground_check=True)
    scaled = DetectorSettings(ground_check=True, contact_height_per_m=0.02)
    assert [reason for _, reason in run([blob(140, 80, 40, 10)], depth, GREY, fixed)[1]] == [
        CONTACT_UNKNOWN]
    assert len(run([blob(140, 80, 40, 10)], depth, GREY, scaled)[0]) == 1
    depth[rows[-1] - 10:, 130:190] = np.nan   # more of it missing: still unknown
    assert [reason for _, reason in run([blob(140, 80, 40, 10)], depth, GREY, scaled)[1]] == [
        CONTACT_UNKNOWN]


def test_contact_by_height_ignores_turf_measured_high_below_the_base():
    from object_detection.grounded import floor_contact, Frame
    shape = (60, 40)
    depth = np.full(shape, 2.0)
    heights = np.zeros(shape)
    depth[30:40, :] = 1.6     # turf just below the object, measured nearer...
    heights[30:40, :] = 0.05  # ...and 5 cm above the floor plane
    valid = np.ones(shape, bool)
    frame = Frame(depth, heights, valid, valid & (heights <= 0.03), valid & (heights > 0.03))
    mask = np.ones((20, 10), bool)   # object rows 10-29, base 3 cm above the floor
    band = DetectorSettings(ground_check=True)
    height = DetectorSettings(ground_check=True, contact_by_height=True)
    assert floor_contact(mask, 15, 10, 0.03, frame, band, distance=2.0) == NOT_ON_FLOOR
    assert floor_contact(mask, 15, 10, 0.03, frame, height, distance=2.0) is None
    # Base clearly above the limit: still not on the floor.
    assert floor_contact(mask, 15, 10, 0.15, frame, height, distance=2.0) == NOT_ON_FLOOR
    # Beyond contact_by_height_max_m the band check applies again.
    far = DetectorSettings(ground_check=True, contact_by_height=True,
                           contact_by_height_max_m=1.5)
    assert floor_contact(mask, 15, 10, 0.03, frame, far, distance=2.0) == NOT_ON_FLOOR
