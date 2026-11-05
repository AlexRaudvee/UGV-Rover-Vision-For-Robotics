import sys
# The container's pip OpenCV 4.10 is broken; prefer the system OpenCV 4.5.4 (matches cv_bridge)
sys.path.insert(0, '/usr/lib/python3/dist-packages')

import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

OPEN_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


def build_roi(h, w, top, bottom, side_top, side_bottom):
    """Trapezoid mask of the area the robot is about to drive into.

    top/bottom are fractions of image height; side_* are fractions of width trimmed
    from EACH side at the top and bottom edges. Trimming more at the bottom cuts out
    the dark bottom corners (vignetting / rover body).
    """
    poly = np.array([
        (int(side_top * w), int(top * h)),
        (int((1 - side_top) * w), int(top * h)),
        (int((1 - side_bottom) * w), int(bottom * h)),
        (int(side_bottom * w), int(bottom * h)),
    ], dtype=np.int32)
    roi = np.zeros((h, w), np.uint8)
    cv2.fillPoly(roi, [poly], 255)
    return roi, poly


def detect_obstacle(gray, roi, roi_area, threshold, fx, cx, min_area, obstacle_frac):
    """Find dark blobs inside the ROI of a greyscale frame.

    Returns (obstacle, blobs, dark_mask):
      obstacle - True if the kept blobs cover more than obstacle_frac of the ROI
      blobs    - list of (bearing_deg, area_px, contour)
                 ROS convention: 0 = straight ahead, positive = LEFT, negative = RIGHT
    """
    dark = np.where(gray < threshold, 255, 0).astype(np.uint8)
    dark = cv2.bitwise_and(dark, roi)
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, OPEN_KERNEL)   # remove speckle

    contours = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    blobs = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        M = cv2.moments(c)
        if M['m00'] <= 0:
            continue
        u = M['m10'] / M['m00']
        # pinhole model: angle = atan((u - cx) / fx); image x grows right, ROS angles grow left
        bearing = -math.degrees(math.atan2(u - cx, fx))
        blobs.append((bearing, area, c))

    covered = sum(a for _, a, _ in blobs)
    obstacle = covered > obstacle_frac * roi_area
    return obstacle, blobs, dark


def main_blob(blobs):
    return max(blobs, key=lambda b: b[1]) if blobs else None


def turn_away_from_obstacle(blobs, turn_speed=0.5):
    """Rotate in place away from the largest blob (stray small blobs can't flip the direction)."""
    cmd = Twist()
    blob = main_blob(blobs)
    if blob is None:
        cmd.angular.z = turn_speed                # obstacle seen but no usable blob: default to turning left
        return cmd
    # obstacle on the left (positive) -> turn right (negative angular.z), and vice versa
    cmd.angular.z = -turn_speed if blob[0] >= 0 else turn_speed
    return cmd


class DriveForward(Node):
    def __init__(self):
        super().__init__('drive_forward')
        # Run-time settings: ros2 run my_package my_node --ros-args -p distance:=1.0 -p speed:=0.15
        p = self.declare_parameter
        p('distance', 0.5)
        p('speed', 0.2)
        p('min_speed', 0.05)
        p('slow_zone', 0.5)
        p('odom_wait', 5.0)
        p('image_every', 3)          # process every Nth camera frame to save CPU
        # detection
        p('dark_ratio', 0.65)        # dark = grey below this fraction of the turf brightness
        p('turf_percentile', 75.0)   # turf brightness = this percentile of grey in the ROI
        p('turf_adapt', 0.05)        # how fast the turf reference follows lighting (only while clear)
        p('obstacle_frac', 0.05)     # obstacle if kept blobs cover more than this fraction of the ROI
        p('min_blob_area', 200)      # pixels; smaller dark blobs are ignored
        # driving corridor (fractions of image size)
        p('roi_top', 0.55)
        p('roi_bottom', 0.95)
        p('roi_side_top', 0.05)
        p('roi_side_bottom', 0.15)
        # camera calibration (pixels, at the resolution published on /image_raw)
        p('cam_fx', 289.0)
        p('cam_cx', 347.0)
        # behaviour
        p('avoid', False)            # turn away instead of stopping
        p('turn_speed', 0.5)         # rad/s while turning away
        p('debug_image', False)      # publish /drive_forward/debug for rqt_image_view

        g = lambda name: self.get_parameter(name).value
        self.target = float(g('distance'))
        self.speed = float(g('speed'))
        self.min_speed = float(g('min_speed'))
        self.slow_zone = float(g('slow_zone'))
        self.odom_wait = float(g('odom_wait'))
        self.image_every = max(1, int(g('image_every')))
        self.dark_ratio = float(g('dark_ratio'))
        self.turf_percentile = float(g('turf_percentile'))
        self.turf_adapt = float(g('turf_adapt'))
        self.obstacle_frac = float(g('obstacle_frac'))
        self.min_blob_area = int(g('min_blob_area'))
        self.roi_params = (float(g('roi_top')), float(g('roi_bottom')),
                           float(g('roi_side_top')), float(g('roi_side_bottom')))
        self.fx = float(g('cam_fx'))
        self.cx = float(g('cam_cx'))
        self.avoid = bool(g('avoid'))
        self.turn_speed = float(g('turn_speed'))
        self.debug_image = bool(g('debug_image'))

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/drive_forward/debug', 1) if self.debug_image else None
        # /odom/odom_raw: data = [left_wheel_m, right_wheel_m], cumulative since the base started
        self.create_subscription(Float32MultiArray, '/odom/odom_raw', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/image_raw', self.image_cb, qos_profile_sensor_data)
        self.bridge = CvBridge()

        self.start_dist = None           # set from the first odom message
        self.travelled = 0.0
        self.obstacle = False            # set by image_cb, read by tick
        self.blobs = []                  # (bearing_deg, area, contour), set by image_cb
        self.roi = None                  # built on the first frame, once the size is known
        self.turf_ref = None             # running turf brightness
        self.frames = 0
        self.done = False

        # safety nets: give up if odometry never arrives, or the drive takes far longer than expected
        self.timeout = 2.0 * self.target / self.speed
        self.t_created = time.monotonic()
        self.t0 = None                   # drive clock starts at the first odom message
        self.create_timer(0.1, self.tick)
        mode = 'turning away from' if self.avoid else 'stopping at'
        self.get_logger().info(f'Driving {self.target} m at {self.speed} m/s; {mode} obstacles')

    def odom_cb(self, msg):
        if len(msg.data) < 2:
            return
        # average of both sides = distance of the centre; turning in place adds ~0
        dist = (msg.data[0] + msg.data[1]) / 2.0
        if self.start_dist is None:
            self.start_dist = dist
            self.t0 = time.monotonic()
            self.get_logger().info(f'Start odometry: left={msg.data[0]:.2f} m, right={msg.data[1]:.2f} m')
        self.travelled = abs(dist - self.start_dist)

    def image_cb(self, msg):
        self.frames += 1
        if self.frames % self.image_every:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = frame.shape[:2]
        if self.roi is None:
            self.roi, self.roi_poly = build_roi(h, w, *self.roi_params)
            self.roi_area = cv2.countNonZero(self.roi)
            self.get_logger().info(f'Image {w}x{h}, ROI {self.roi_area} px')
            if not 0 < self.cx < w:
                self.get_logger().warn(f'cam_cx={self.cx:.0f} is outside the {w}px image - '
                                       'calibration was probably done at another resolution')

        # greyscale weights blue lightly, so the blue-tinted box still reads as dark
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        turf_now = float(np.percentile(gray[self.roi > 0], self.turf_percentile))
        if self.turf_ref is None:
            self.turf_ref = turf_now
        threshold = self.dark_ratio * self.turf_ref

        self.obstacle, self.blobs, dark = detect_obstacle(
            gray, self.roi, self.roi_area, threshold, self.fx, self.cx,
            self.min_blob_area, self.obstacle_frac)

        # follow slow lighting changes, but never learn from a frame with an obstacle in it
        if not self.obstacle:
            self.turf_ref += self.turf_adapt * (turf_now - self.turf_ref)

        main = main_blob(self.blobs)
        blobs_txt = ', '.join(f'{b:+.0f}deg/{a:.0f}px' for b, a, _ in self.blobs) or 'none'
        main_txt = f' main {main[0]:+.0f}deg ({"L" if main[0] > 0 else "R"})' if main else ''
        self.get_logger().info(f'turf {self.turf_ref:.0f} thr {threshold:.0f} '
                               f'obstacle={self.obstacle} blobs: {blobs_txt}{main_txt}',
                               throttle_duration_sec=1.0)

        if self.debug_pub is not None:
            self.publish_debug(frame, dark, threshold)

    def publish_debug(self, frame, dark, threshold):
        dbg = frame.copy()
        dbg[dark > 0] = (0, 0, 255)                                   # dark pixels in red
        cv2.polylines(dbg, [self.roi_poly], True, (0, 255, 0), 2)     # corridor in green
        main = main_blob(self.blobs)
        for blob in self.blobs:
            bearing, area, c = blob
            thick = 3 if blob is main else 1
            cv2.drawContours(dbg, [c], -1, (0, 255, 255), thick)
            x, y, _, _ = cv2.boundingRect(c)
            cv2.putText(dbg, f'{bearing:+.0f}', (x, max(15, y - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(dbg, f'turf {self.turf_ref:.0f} thr {threshold:.0f} obstacle={self.obstacle}',
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(dbg, 'bgr8'))

    def tick(self):
        if self.start_dist is None:
            if time.monotonic() - self.t_created > self.odom_wait:
                self.get_logger().error(f'No /odom/odom_raw within {self.odom_wait:.0f} s - not driving. '
                                        'Is ugv_bringup running, and is ROS_DOMAIN_ID set?')
                self.done = True
            else:
                self.get_logger().info('Waiting for /odom/odom_raw...', throttle_duration_sec=2.0)
            return

        if time.monotonic() - self.t0 > self.timeout:
            self.get_logger().warn(f'Timeout after {self.travelled:.2f} m')
            self.done = True
            return

        if self.obstacle:
            if not self.avoid:
                self.get_logger().info(f'Obstacle detected after {self.travelled:.2f} m - stopping')
                self.done = True
                return
            cmd = turn_away_from_obstacle(self.blobs, self.turn_speed)
            self.pub.publish(cmd)
            side = 'right' if cmd.angular.z < 0 else 'left'
            self.get_logger().info(f'Obstacle at {self.travelled:.2f} m - turning {side}',
                                   throttle_duration_sec=1.0)
            return

        remaining = self.target - self.travelled
        if remaining <= 0.0:
            self.get_logger().info(f'Reached target: travelled {self.travelled:.3f} m')
            self.done = True
            return

        # full speed until the last slow_zone metres, then ramp down linearly (never below min_speed)
        speed = max(self.min_speed, self.speed * min(1.0, remaining / self.slow_zone))
        cmd = Twist()
        cmd.linear.x = speed
        self.pub.publish(cmd)
        self.get_logger().info(f'{self.travelled:.2f} / {self.target:.2f} m  speed {speed:.2f} m/s',
                               throttle_duration_sec=1.0)

    def stop(self):
        for _ in range(5):
            self.pub.publish(Twist())
            time.sleep(0.05)
        self.get_logger().info('Stopped')


def main():
    rclpy.init()
    node = DriveForward()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()