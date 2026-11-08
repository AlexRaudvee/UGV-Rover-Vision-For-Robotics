"""
Ground check: keep only objects that stand on the floor, using depth.

Works at half resolution (SCALE) in one pass over the whole frame:

1. split the depth image into floor and above-floor pixels (heights from the fitted
   floor plane);
2. cut the above-floor pixels at depth jumps and label the connected surfaces once;
3. merge surfaces whose boxes touch (within MERGE_GAP px) at about the same distance,
   so a rover's deck and tracks or a box's split faces become one object;
4. each merged surface is a colour object if enough of it is dark (colour regions),
   otherwise a depth-only object (grey / backlit boxes), which must also look like an
   obstacle: 10-80 cm wide, top 10-60 cm high, within range, and greyish;
5. keep an object only if it stands on the floor (lowest part near floor height and
   floor just below its bottom edge) and passes the size filters (too small / hollow /
   too thin);
6. a dark region the surfaces barely explain (very close or plain black surfaces with
   sparse depth) is measured as a colour region, with the same floor-contact test.

Rejected colour objects and regions are reported with a reason; depth-only failures
are silent.
"""

from dataclasses import dataclass

import cv2
import numpy as np

from .floor import classify, depth_points
from .perception import Blob, depth_estimate_for_blob, RangeEstimate

SCALE = 2       # work on every 2nd pixel in both directions (640x360 -> 320x180)
MERGE_GAP = 2   # working px: the cut at a depth jump leaves a ~2 px gap in one surface

NO_DISTANCE = 'no distance'
FLAT = 'flat on floor'
NOT_ON_FLOOR = 'not on floor'
CONTACT_UNKNOWN = 'contact unknown'
NO_FLOOR = 'no floor found'
TOO_SMALL, HOLLOW, TOO_THIN = 'too small', 'hollow', 'too thin'


@dataclass
class GroundedObject:
    """A floor-standing object ready to be measured."""

    blob: Blob
    estimate: RangeEstimate
    height_m: float           # top of the object above the floor (95th percentile)
    outline: list             # simplified polygon [[x, y], ...]
    seed: str = 'colour'      # 'colour' (dark pixels support it) or 'depth' alone


class Piece:
    """A boolean mask stored as a tight local array plus its image offset."""

    def __init__(self, local, x0, y0):
        ys, xs = np.nonzero(local)
        if len(ys):
            local = local[ys.min():ys.max() + 1, xs.min():xs.max() + 1]
            x0, y0 = x0 + int(xs.min()), y0 + int(ys.min())
        self.local, self.x0, self.y0 = local.astype(bool), int(x0), int(y0)

    @property
    def box(self):
        """(x0, y0, x1, y1), exclusive end."""
        return (self.x0, self.y0, self.x0 + self.local.shape[1],
                self.y0 + self.local.shape[0])

    def area(self):
        return int(self.local.sum())


def upscaled(mask, x0, y0):
    """Full-resolution Piece of a working-resolution mask with corner (x0, y0)."""
    # Each working pixel sampled the top-left full-resolution pixel of its
    # SCALE x SCALE block: drop the extra column/row the upscale adds at the end.
    return Piece(cv2.resize(mask.astype(np.uint8), None, fx=SCALE, fy=SCALE,
                            interpolation=cv2.INTER_NEAREST)[:1 - SCALE, :1 - SCALE],
                 x0 * SCALE, y0 * SCALE)


def region_piece(blob):
    """Full-resolution Piece of a colour region."""
    x, y, w, h = blob.bbox
    local = np.zeros((h, w), np.uint8)
    cv2.drawContours(local, [blob.contour], -1, 1, cv2.FILLED, offset=(-x, -y))
    return Piece(local, x, y)


def joined(first, second):
    """Union of two Pieces."""
    x0, y0 = min(first.x0, second.x0), min(first.y0, second.y0)
    x1, y1 = max(first.box[2], second.box[2]), max(first.box[3], second.box[3])
    local = np.zeros((y1 - y0, x1 - x0), bool)
    for item in (first, second):
        ax, ay, bx, by = item.box
        local[ay - y0:by - y0, ax - x0:bx - x0] |= item.local
    return Piece(local, x0, y0)


def piece_bbox(piece):
    """(x, y, w, h) of a Piece."""
    x0, y0, x1, y1 = piece.box
    return (x0, y0, x1 - x0, y1 - y0)


def rests_on(box, support, settings):
    """Whether box sits on top of support: overlapping sideways, at most a small gap."""
    x, y, w, h = box
    sx, sy, sw, sh = support
    overlap = min(x + w, sx + sw) - max(x, sx)
    return (settings.merge_max_gap_px > 0 and y < sy
            and overlap >= settings.merge_min_overlap * min(w, sw)
            and sy - (y + h) <= settings.merge_max_gap_px)


def working_points(depth, matrix):
    """3D points (optical frame) of the depth image at the working resolution."""
    scaled = np.array(matrix, float)
    scaled[:2] /= SCALE
    return depth_points(depth[::SCALE, ::SCALE], scaled)


def blob_from_piece(piece, camera):
    """Blob with the angles, area and clipping of a full-resolution piece."""
    x, y, x1, y1 = piece.box
    width, height = x1 - x, y1 - y
    rays = camera.horizontal_rays([(x, y + height / 2), (x + width, y + height / 2),
                                   (x + width / 2, y + height / 2)])
    angles = -np.arctan(rays)
    contours = cv2.findContours(piece.local.astype(np.uint8), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE, offset=(x, y))[-2]
    contour = max(contours, key=cv2.contourArea)
    return Blob((x, y, width, height), contour, float(angles[2]), float(min(angles[:2])),
                float(max(angles[:2])), abs(float(rays[1] - rays[0])), float(piece.area()),
                x <= 0 or x + width >= camera.width)


def depth_edges(depth, settings):
    """Mark pixels next to a depth jump; surfaces never continue across these."""
    filled = np.where(np.isfinite(depth), depth, 0.0)
    limit = settings.edge_step_m + settings.edge_step_per_m * filled
    edges = np.zeros(depth.shape, bool)
    jump_x = np.abs(np.diff(filled, axis=1)) > np.minimum(limit[:, 1:], limit[:, :-1])
    jump_y = np.abs(np.diff(filled, axis=0)) > np.minimum(limit[1:, :], limit[:-1, :])
    edges[:, 1:] |= jump_x
    edges[:, :-1] |= jump_x
    edges[1:, :] |= jump_y
    edges[:-1, :] |= jump_y
    return edges


def reach(distance, settings):
    """Depth band around an object's distance that still counts as the same object."""
    return max(settings.grow_range_m, settings.grow_range_fraction * distance)


def size_problem(piece, blob, settings):
    """Apply the colour-region size filters to a whole object; None if it passes."""
    _, _, width, height = blob.bbox
    area = piece.area()
    if area < settings.min_area or width < settings.min_width or height < settings.min_height:
        return TOO_SMALL
    if area / (width * height) < settings.min_fill:
        return HOLLOW
    if blob.ray_span * settings.range_max < settings.width_min_m - settings.width_tolerance_m:
        return TOO_THIN
    return None


@dataclass
class Frame:
    """Per-frame working-resolution images shared by all steps."""

    depth: np.ndarray
    heights: np.ndarray
    valid: np.ndarray
    floor: np.ndarray
    above: np.ndarray


def floor_contact(mask, x0, y0, lowest, frame, settings, distance=None, range_m=None):
    """
    Return None if the mask (local, corner x0, y0) stands on the floor, else a reason.

    lowest: height of the object's lowest part. With distance given, pixels just below
    the object that look like more of it (above the floor, at its distance: a base row
    split off by a depth jump) are skipped before the floor band is checked. Without depth
    below it, its lowest part may be contact_max_height_m + contact_height_per_m per metre
    of distance (distance, else range_m) above the floor: stereo noise grows with range.

    With contact_by_height, that height limit alone decides, whatever the band below
    shows (turf in front of a base often measures a few cm above the plane); the band
    then only picks the label ('not on floor' with depth below, else 'contact unknown').
    Only up to contact_by_height_max_m: further away depth is too noisy to trust alone.
    """
    rows = frame.depth.shape[0]
    if y0 + mask.shape[0] >= rows:
        return None   # reaches the image bottom: close, and its base is out of view
    has = mask.any(axis=0)
    columns = x0 + np.flatnonzero(has)
    bottoms = y0 + mask.shape[0] - 1 - np.argmax(mask[::-1], axis=0)[has]
    band = max(2, settings.contact_band_px // SCALE)
    skip = 0
    if distance is not None:
        steps = 4 * band
        below = np.minimum(bottoms[None, :] + np.arange(1, steps + 1)[:, None], rows - 1)
        cols = np.repeat(columns[None, :], steps, 0)
        same = frame.above[below, cols] & (
            np.abs(frame.depth[below, cols] - distance) <= reach(distance, settings))
        skip = np.where(same.all(axis=0), steps, np.argmin(same, axis=0))
    below = np.minimum(bottoms[None, :] + skip + np.arange(1, band + 1)[:, None], rows - 1)
    cols = np.repeat(columns[None, :], band, 0)
    valid = frame.valid[below, cols]
    reach_m = distance if distance is not None else range_m
    limit = settings.contact_max_height_m + settings.contact_height_per_m * (
        reach_m if reach_m is not None and np.isfinite(reach_m) else 0.0)
    if valid.mean() < settings.contact_min_valid:
        # No depth just below it: decide on the object's own lowest part alone.
        return None if lowest <= limit else CONTACT_UNKNOWN
    if settings.contact_by_height and (reach_m is None or not np.isfinite(reach_m)
                                       or reach_m <= settings.contact_by_height_max_m):
        return None if lowest <= limit else NOT_ON_FLOOR
    if frame.floor[below, cols][valid].mean() < settings.contact_floor_fraction:
        return NOT_ON_FLOOR
    if lowest > settings.contact_max_height_m:
        return NOT_ON_FLOOR   # floor below it in the image, but it hangs above it
    return None


def with_rim(mask, x0, y0, distance, frame, settings):
    """
    Give back the 1-px rim the depth-jump cut removed around a surface.

    Returns (mask, x0, y0) grown by one pixel into neighbouring above-floor pixels at
    the surface's distance, so the object's width is not measured short.
    """
    rows, cols = frame.depth.shape
    gx0, gy0 = max(0, x0 - 1), max(0, y0 - 1)
    gx1, gy1 = min(cols, x0 + mask.shape[1] + 1), min(rows, y0 + mask.shape[0] + 1)
    grown = np.zeros((gy1 - gy0, gx1 - gx0), np.uint8)
    grown[y0 - gy0:y0 - gy0 + mask.shape[0], x0 - gx0:x0 - gx0 + mask.shape[1]] = mask
    grown = cv2.dilate(grown, np.ones((3, 3), np.uint8)).astype(bool)
    box = (slice(gy0, gy1), slice(gx0, gx1))
    grown &= frame.above[box] & (np.abs(frame.depth[box] - distance)
                                 <= reach(distance, settings))
    grown[y0 - gy0:y0 - gy0 + mask.shape[0], x0 - gx0:x0 - gx0 + mask.shape[1]] |= mask
    return grown, gx0, gy0


def depth_only_problem(blob, top, distance, saturation, settings):
    """Obstacle-like limits for objects without dark-colour support; None if it passes."""
    if blob.bbox[1] <= 0:
        return 'top out of view'
    if not (settings.depth_seed_min_width_m <= distance * blob.ray_span
            <= settings.depth_seed_max_width_m):
        return 'depth seed width'
    if top > settings.depth_seed_max_height_m:
        return 'too tall'
    if top < settings.depth_seed_min_height_m:
        return 'too low'
    if saturation > settings.depth_seed_max_saturation:
        return 'coloured'   # e.g. the red barriers
    return None


def estimate_of(values):
    """Median distance and robust spread of depth samples."""
    median = float(np.median(values))
    spread = max(0.01, 1.4826 * float(np.median(np.abs(values - median))))
    return RangeEstimate(median, spread, len(values), 1.0)


def merged_surfaces(stats, mean_depth, labels, settings):
    """Group surface labels whose boxes touch at about the same distance."""
    labels = np.asarray(labels, int)
    x0, y0 = stats[labels, 0], stats[labels, 1]
    x1, y1 = x0 + stats[labels, 2], y0 + stats[labels, 3]
    depth = mean_depth[labels]
    near = np.minimum(depth[:, None], depth[None, :])
    band = np.maximum(settings.grow_range_m, settings.grow_range_fraction * near)
    touch = ((x0[None, :] <= x1[:, None] + MERGE_GAP) & (x0[:, None] <= x1[None, :] + MERGE_GAP)
             & (y0[None, :] <= y1[:, None] + MERGE_GAP) & (y0[:, None] <= y1[None, :] + MERGE_GAP)
             & (np.abs(depth[:, None] - depth[None, :]) <= band))
    parent = list(range(len(labels)))

    def root(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    for first, second in zip(*np.nonzero(np.triu(touch, 1))):
        parent[root(first)] = root(second)
    groups = {}
    for index, label in enumerate(labels):
        groups.setdefault(root(index), []).append(int(label))
    return list(groups.values())


def ground_objects(blobs, bgr, depth, heights, camera, settings, floor_settings=None):
    """
    Return (objects, rejected, unmeasured_blobs) for one frame.

    blobs may include small pieces (blob.small) that failed the colour size filters:
    they count as dark-colour evidence but are never reported on their own. bgr and
    depth are full resolution; heights is the height above the floor at the working
    resolution (see working_points), or None when no floor was found.
    """
    full_size = [blob for blob in blobs if not blob.small]
    if heights is None:
        return [], [(blob.bbox, NO_FLOOR) for blob in full_size], []
    d = depth[::SCALE, ::SCALE]
    rows, cols = d.shape
    valid = np.isfinite(d) & (d >= settings.range_min) & (d <= settings.range_max)
    floor, above = classify(heights) if floor_settings is None else classify(heights,
                                                                             floor_settings)
    frame = Frame(d, heights, valid, floor & valid, above & valid)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        (frame.above & ~depth_edges(d, settings)).astype(np.uint8), connectivity=4)
    dark = np.zeros((rows, cols), np.uint8)
    cv2.drawContours(dark, [blob.contour // SCALE for blob in blobs], -1, 1, cv2.FILLED)
    flat_labels = labels.ravel()
    dark_count = np.bincount(flat_labels[dark.ravel() > 0], minlength=count)
    mean_depth = (np.bincount(flat_labels, weights=np.nan_to_num(d).ravel(), minlength=count)
                  / np.maximum(stats[:, 4], 1))
    min_pixels = max(5, settings.seed_min_pixels // (SCALE * SCALE))
    saturation = None   # computed on first use: only depth-only candidates need it

    objects, failed, rejected, unmeasured = [], [], [], []
    pieces = []     # full-resolution Piece of each kept object
    hanging = []    # (piece, distance, top, failed entry): dark, off the floor
    covered = np.zeros((rows, cols), bool)     # pixels of kept objects
    explained = np.zeros((rows, cols), bool)   # pixels of colour-supported surfaces
    usable = [label for label in range(1, count) if stats[label, 4] >= min_pixels]
    for members in merged_surfaces(stats, mean_depth, usable, settings):
        x0, y0 = stats[members, 0].min(), stats[members, 1].min()
        x1 = (stats[members, 0] + stats[members, 2]).max()
        y1 = (stats[members, 1] + stats[members, 3]).max()
        surface = np.isin(labels[y0:y1, x0:x1], members)
        surface_box = (slice(y0, y1), slice(x0, x1))
        dark_pixels = int(dark_count[members].sum())
        colour = (dark_pixels >= min_pixels
                  and dark_pixels >= settings.dark_min_share * surface.sum())
        if colour:
            explained[y0:y1, x0:x1] |= surface
        depths = d[y0:y1, x0:x1][surface]
        distance = float(np.median(depths))
        if not colour and distance > settings.depth_seed_max_range_m:
            continue
        lowest, top = np.percentile(heights[y0:y1, x0:x1][surface], [5, 95])
        reason = floor_contact(surface, x0, y0, lowest, frame, settings, distance)
        if reason is None:
            mask, x0, y0 = with_rim(surface, x0, y0, distance, frame, settings)
            x1, y1 = x0 + mask.shape[1], y0 + mask.shape[0]
            piece = upscaled(mask, x0, y0)
            blob = blob_from_piece(piece, camera)
            reason = size_problem(piece, blob, settings)
        if reason is None and not colour:
            if saturation is None:
                saturation = cv2.cvtColor(bgr[::SCALE, ::SCALE], cv2.COLOR_BGR2HSV)[:, :, 1]
            reason = depth_only_problem(
                blob, top, distance, float(np.median(saturation[surface_box][surface])),
                settings)
        if reason is not None:
            if colour:   # depth-only failures are weak evidence: never reported
                entry = (tuple(int(v) * SCALE for v in (x0, y0, x1 - x0, y1 - y0)), reason)
                failed.append(entry)
                if reason in (NOT_ON_FLOOR, CONTACT_UNKNOWN):
                    hanging.append((upscaled(surface, x0, y0), distance, float(top), entry))
            continue
        covered[y0:y1, x0:x1] |= mask
        pieces.append(piece)
        objects.append(GroundedObject(
            blob, estimate_of(depths), float(top),
            cv2.approxPolyDP(blob.contour, 2.0, True)[:, 0, :].tolist(),
            'colour' if colour else 'depth'))

    # Sparse depth: a dark region the colour-supported surfaces barely explain is
    # measured as the colour region itself (very close or plain black surfaces).
    for blob in full_size:
        bx, by, bw, bh = blob.bbox
        x0, y0 = bx // SCALE, by // SCALE
        x1, y1 = min(cols, (bx + bw) // SCALE + 1), min(rows, (by + bh) // SCALE + 1)
        region = np.zeros((y1 - y0, x1 - x0), np.uint8)
        cv2.drawContours(region, [blob.contour // SCALE], -1, 1, cv2.FILLED,
                         offset=(-x0, -y0))
        region = region.astype(bool)
        box = (slice(y0, y1), slice(x0, x1))
        region_valid = region & frame.valid[box]
        standing = region_valid & ~frame.floor[box]
        if not standing.any():
            standing = region   # (almost) no depth: judge the whole region
        if (covered[box][standing].mean() > 0.5
                or explained[box][standing].mean() >= settings.sparse_coverage):
            continue   # already an object, or the surfaces above decided it
        if region_valid.sum() < max(4, settings.min_depth_pixels // (SCALE * SCALE)):
            unmeasured.append(blob)
            rejected.append((blob.bbox, 'clipped, ' + NO_DISTANCE if blob.clipped
                             else NO_DISTANCE))
            continue
        if (region & frame.above[box]).sum() < max(4, settings.min_depth_pixels
                                                   // (SCALE * SCALE)):
            rejected.append((blob.bbox, FLAT))   # shadow or dark turf
            continue
        # Lowest part over all its depth, floor-height pixels included: the region
        # reaches down to the floor even where its own base has no depth.
        lowest = float(np.percentile(heights[box][region_valid], 5))
        reason = floor_contact(region, x0, y0, lowest, frame, settings,
                               range_m=float(np.median(d[box][region_valid])))
        if reason is None:
            reason = size_problem(Piece(np.ones((bh, bw), bool), bx, by), blob, settings)
        estimate = depth_estimate_for_blob(blob, depth, settings)
        if reason is None and not np.isfinite(estimate.axial_m):
            # The nearest sizeable surface inside the region gives the distance.
            inside_labels = labels[box][region & frame.above[box]]
            sizes = np.bincount(inside_labels, minlength=count)
            sizes[0] = 0
            sizeable = np.flatnonzero(sizes >= max(min_pixels, 0.1 * sizes.sum()))
            if len(sizeable):
                nearest = sizeable[np.argmin(mean_depth[sizeable])]
                estimate = estimate_of(d[box][region & (labels[box] == nearest)])
        if reason is None and not np.isfinite(estimate.axial_m):
            unmeasured.append(blob)
            reason = NO_DISTANCE
        top = float(np.percentile(heights[box][region & frame.above[box]], 95))
        if reason is not None:
            failed.append((blob.bbox, reason))
            if reason in (NOT_ON_FLOOR, CONTACT_UNKNOWN) and np.isfinite(estimate.axial_m):
                hanging.append((region_piece(blob), estimate.axial_m, top, failed[-1]))
            continue
        covered[box] |= region
        pieces.append(region_piece(blob))
        objects.append(GroundedObject(
            blob, estimate, top,
            cv2.approxPolyDP(blob.contour, 2.0, True)[:, 0, :].tolist(), 'colour'))

    # A dark piece that is off the floor only because it rests on a kept object at the
    # same distance (a rover's body on its tracks, a box on a box) is part of it.
    for piece, distance, top, entry in hanging:
        for index, item in enumerate(objects):
            if (rests_on(piece_bbox(piece), item.blob.bbox, settings)
                    and abs(distance - item.estimate.axial_m)
                    <= settings.merge_max_range_diff_m):
                pieces[index] = joined(pieces[index], piece)
                blob = blob_from_piece(pieces[index], camera)
                objects[index] = GroundedObject(
                    blob, item.estimate, max(item.height_m, top),
                    cv2.approxPolyDP(blob.contour, 2.0, True)[:, 0, :].tolist(), item.seed)
                failed.remove(entry)
                break

    # Pieces lying inside a kept object's box are part of it, not separate rejections.
    for box, reason in failed:
        if not any(inside(box, item.blob.bbox) for item in objects):
            rejected.append((box, reason))
    return objects, rejected, unmeasured


def inside(box, outer, share=0.8):
    """Return whether at least `share` of box's area lies within outer."""
    x, y, w, h = box
    ox, oy, ow, oh = outer
    overlap = (max(0, min(x + w, ox + ow) - max(x, ox))
               * max(0, min(y + h, oy + oh) - max(y, oy)))
    return overlap >= share * w * h
