"""Small JPEG-friendly views for Vizanti's existing Compressed Image widget."""

import math

import cv2
import numpy as np


BACKGROUND = (34, 28, 23)
WHITE = (235, 235, 235)
MUTED = (150, 150, 150)
UNKNOWN = (85, 85, 85)


def text(image, message, point, scale=0.48, color=WHITE):
    cv2.putText(image, str(message), point, cv2.FONT_HERSHEY_SIMPLEX,
                scale, color, 1, cv2.LINE_AA)


def histogram_image(histogram, detections, label, status, fresh):
    """Draw a camera-relative polar score plot and a front-view bar histogram."""
    image = np.full((580, 1000, 3), BACKGROUND, dtype=np.uint8)
    text(image, 'BLACK-OBSTACLE POLAR HISTOGRAM', (22, 30), 0.72)
    text(image, label, (22, 55), 0.50, MUTED)
    text(image, status[:110], (22, 80), 0.48,
         (110, 220, 160) if fresh else (90, 150, 255))
    center = (265, 315)
    radius = 190

    def point(angle, length):
        return (round(center[0] - length * math.sin(angle)),
                round(center[1] - length * math.cos(angle)))

    for index, angle in enumerate(histogram.angles):
        if not histogram.observed[index]:
            cv2.line(image, point(angle, 168), point(angle, 188), UNKNOWN, 3)
        else:
            cv2.line(image, point(angle, 168), point(angle, 188), (75, 145, 95), 3)
            score = float(histogram.scores[index])
            if score > 0.02:
                half = histogram.step * 0.40
                polygon = np.array([center, point(angle - half, score * radius),
                                    point(angle + half, score * radius)], dtype=np.int32)
                cv2.fillConvexPoly(image, polygon, (45, 65, 235), cv2.LINE_AA)
    for fraction in (0.25, 0.5, 0.75, 1.0):
        cv2.circle(image, center, round(radius * fraction), (80, 75, 65), 1, cv2.LINE_AA)
    cv2.arrowedLine(image, center, (center[0], center[1] - radius - 8), WHITE, 1)
    cv2.circle(image, center, 5, WHITE, -1)
    text(image, 'FRONT 0 deg', (205, 108), 0.48)
    text(image, 'LEFT (+)', (22, 315), 0.45)
    text(image, 'RIGHT (-)', (425, 315), 0.45)
    text(image, 'REAR', (245, 530), 0.45, MUTED)
    text(image, 'Radius = obstacle score, not distance', (65, 555), 0.43, MUTED)

    x0, x1, y0, y1 = 570, 970, 165, 425
    text(image, 'FRONT VIEW: angular-bin scores', (565, 125), 0.48)
    front = np.flatnonzero(np.abs(histogram.angles) <= math.pi / 2)[::-1]
    width = (x1 - x0) / len(front)
    for position, index in enumerate(front):
        x = round(x0 + position * width)
        right = round(x0 + (position + 1) * width) - 1
        if histogram.observed[index]:
            score = float(histogram.scores[index])
            top = round(y1 - score * (y1 - y0))
            color = (45, 65, 235) if score > 0.02 else (75, 145, 95)
            cv2.rectangle(image, (x, min(top, y1 - 4)), (right, y1), color, -1)
        else:
            cv2.rectangle(image, (x, y0), (right, y1), (53, 49, 45), -1)
            cv2.line(image, (x, y1 - 7), (right, y1 - 7), UNKNOWN, 2)
    for fraction in (0, 0.5, 1.0):
        y = round(y1 - fraction * (y1 - y0))
        cv2.line(image, (x0, y), (x1, y), (100, 95, 85), 1)
        text(image, f'{fraction:.1f}', (535, y + 4), 0.40, MUTED)
    for degrees in (90, 45, 0, -45, -90):
        x = round(x0 + (90 - degrees) / 180 * (x1 - x0))
        text(image, f'{degrees:+d}', (x - 12, y1 + 20), 0.40, MUTED)
    text(image, 'Bearing (degrees); positive = camera left', (580, 465), 0.43, MUTED)
    measured = sum(item.distance_source == 'depth' for item in detections)
    text(image, f'Candidates: {len(detections)} | depth ranges: {measured}',
         (570, 505), 0.49)
    text(image, 'Red: obstacle | Green: searched | Gray: unknown',
         (535, 535), 0.40, MUTED)
    text(image, 'Color-based baseline: searched is not guaranteed free space',
         (535, 555), 0.37, MUTED)
    return image


def annotated_image(bgr, detections, mask, polygon, threshold, status):
    """Show which dark regions caused the histogram peaks."""
    image = bgr.copy()
    selected = mask != 0
    image[selected] = (image[selected].astype(np.float32) * 0.5
                       + np.array([0, 0, 120])).clip(0, 255).astype(np.uint8)
    cv2.polylines(image, [polygon], True, (60, 220, 80), 1)
    for index, item in enumerate(detections, 1):
        x, y, width, height = item.bbox
        color = (50, 230, 240)
        cv2.rectangle(image, (x, y), (x + width, y + height), color, 2)
        source = 'D' if item.distance_source == 'depth' else '~'
        message = f'{index}: {math.degrees(item.bearing):+.0f}deg {source}{item.distance_m:.2f}m'
        text(image, message, (x, max(42, y - 5)), 0.43, color)
    cv2.rectangle(image, (0, 0), (image.shape[1], 31), BACKGROUND, -1)
    text(image, f'V <= {threshold:.0f} | D=depth, ~=width estimate | {status[:55]}',
         (6, 20), 0.40)
    return image


def waiting_image(message):
    image = np.full((360, 640, 3), BACKGROUND, dtype=np.uint8)
    text(image, 'OBSTACLE DETECTION', (24, 70), 0.8)
    text(image, message[:85], (24, 125), 0.48, (90, 150, 255))
    text(image, 'Histogram is unknown until fresh calibrated images arrive.',
         (24, 175), 0.45, MUTED)
    return image
