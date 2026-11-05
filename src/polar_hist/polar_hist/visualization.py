"""Metric distance histograms with zero degrees at the robot's right side."""

import math

import cv2
import numpy as np

from object_detection.image_io import label

BACKGROUND = (30, 27, 24)
UNKNOWN = (65, 65, 65)
GREEN = (65, 210, 65)
RED = (65, 75, 240)


def display_degrees(camera_bearing, robot_yaw=0.0):
    """Map ROS bearings to 0 right, 90 front, 180 left, and 270 rear."""
    return (math.degrees(camera_bearing + robot_yaw) + 90) % 360


def histogram_image(histogram, gaps, title, status, display_range_m=3.0, robot_yaw=0.0):
    """Draw measured distance bars and passage intervals, preserving unknown bins."""
    image = np.full((620, 1100, 3), BACKGROUND, np.uint8)
    label(image, title, (20, 32), scale=0.75)
    label(image, status[:125], (20, 60), scale=0.43)
    center, radius = (245, 315), 180

    def point(bearing, length):
        bearing += robot_yaw
        return (round(center[0] - length * math.sin(bearing)),
                round(center[1] - length * math.cos(bearing)))

    def distance_fraction(index):
        return min(float(histogram.ranges[index]) / display_range_m, 1.0)

    for index, angle in enumerate(histogram.angles):
        color = UNKNOWN if not histogram.observed[index] else (115, 105, 90)
        cv2.line(image, point(angle, radius), point(angle, radius + 10), color, 3)
        if histogram.observed[index] and np.isfinite(histogram.ranges[index]):
            color = RED if histogram.ranges[index] <= display_range_m else (0, 170, 255)
            cv2.line(image, center, point(angle, radius * distance_fraction(index)), color, 3)
    for gap in gaps:
        for angle in np.linspace(gap.angle_min, gap.angle_max, max(
                2, round((gap.angle_max - gap.angle_min) / 0.01))):
            cv2.line(image, point(angle, radius + 15), point(angle, radius + 23), GREEN, 2)
    for fraction in (0.25, 0.5, 0.75, 1.0):
        cv2.circle(image, center, round(radius * fraction), (90, 80, 70), 1, cv2.LINE_AA)
        label(image, f'{fraction * display_range_m:g}m',
              (center[0] + 5, center[1] + round(radius * fraction) - 4), scale=0.35)
    cv2.arrowedLine(image, center, (center[0], center[1] - radius - 12), (235, 235, 235), 1)
    label(image, '90 FRONT', (205, 107))
    label(image, '180 LEFT', (15, 315))
    label(image, '0 RIGHT', (425, 315), scale=0.40)
    label(image, '270 REAR', (205, 535))
    label(image, 'Radius = measured distance (m)', (100, 565), scale=0.42)

    x0, x1, top, baseline = 565, 1065, 140, 430

    def x_at(angle):
        return round(x0 + display_degrees(angle, robot_yaw) / 360 * (x1 - x0))

    def angular_band(low, high, y, color, thickness):
        start, end = x_at(low), x_at(high)
        if end < start:
            cv2.line(image, (start, y), (x1, y), color, thickness)
            cv2.line(image, (x0, y), (end, y), color, thickness)
        else:
            cv2.line(image, (start, y), (end, y), color, thickness)

    label(image, 'OBSTACLE DISTANCE (m)', (565, 108), scale=0.57)
    bar_width = max(1, round(histogram.step / (2 * math.pi) * (x1 - x0)) - 2)
    for index, angle in enumerate(histogram.angles):
        x = x_at(angle)
        if not histogram.observed[index]:
            cv2.rectangle(image, (x, top), (min(x1, x + bar_width), baseline), (45, 43, 41), -1)
        elif np.isfinite(histogram.ranges[index]):
            color = RED if histogram.ranges[index] <= display_range_m else (0, 170, 255)
            height = round(baseline - distance_fraction(index) * (baseline - top))
            cv2.rectangle(image, (x, height),
                          (min(x1, x + bar_width), baseline), color, -1)
    for distance in np.linspace(0, display_range_m, 7):
        y = round(baseline - distance / display_range_m * (baseline - top))
        cv2.line(image, (x0, y), (x1, y), (100, 95, 85), 1)
        label(image, f'{distance:g}', (525, y + 4), scale=0.40)
    for degrees in (0, 90, 180, 270, 360):
        x = round(x0 + degrees / 360 * (x1 - x0))
        label(image, str(degrees), (x - 10, baseline + 22), scale=0.40)
    for gap in gaps:
        angular_band(gap.angle_min, gap.angle_max, baseline + 34, GREEN, 5)
        angular_band(gap.center_min, gap.center_max, baseline + 47, (190, 255, 190), 4)
    label(image, 'Robot angle (deg): 0 right, 90 front, 180 left', (545, 514), scale=0.40)
    label(image, 'Green: opening | Pale green: possible center headings',
          (535, 540), GREEN, 0.40)
    widths = ', '.join(f'{gap.clearance_m*100:.1f}cm' for gap in gaps) or 'none'
    label(image, 'Gaps after uncertainty allowance: ' + widths[:65], (535, 566), scale=0.40)
    label(image, f'Gray = unknown. Orange = beyond {display_range_m:g}m display range.',
          (535, 590), scale=0.39)
    return image
