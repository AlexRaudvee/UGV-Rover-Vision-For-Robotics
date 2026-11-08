"""Consume both independent detectors and publish histograms plus physical gaps."""

from dataclasses import dataclass, replace
import json
import math
import time

from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Float32MultiArray, Header, String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from detection_improved.image_io import publish_jpeg
from detection_improved.perception import Detection
from detection_improved.detector import transform_arrays
from .gaps import find_gaps
from .histogram import PolarHistogram
from .visualization import histogram_image

# Raw (distorted) images: the FOV is measured on the principal row, so silhouettes
# touching the image edge in other rows can overshoot it by ~1e-3 rad. Angles within
# this tolerance are clipped to the FOV; anything further out is still rejected.
FOV_EDGE_TOLERANCE_RAD = 0.01


def clip_to_fov(low, high, fov):
    """Clip an angular interval that overshoots the FOV by at most the tolerance."""
    if not (np.isfinite([low, high]).all() and low <= high
            and fov[0] - FOV_EDGE_TOLERANCE_RAD <= low
            and high <= fov[1] + FOV_EDGE_TOLERANCE_RAD):
        return None
    return max(low, fov[0]), min(high, fov[1])


@dataclass
class Source:
    """Independent state; losing one sensor never refreshes the other."""

    histogram: PolarHistogram
    plot_pub: object
    scan_pub: object
    values_pub: object
    gaps_pub: object
    received: float = -math.inf
    frame: str = ''
    gaps: tuple = ()
    status: str = 'Waiting for detections'
    header: object = None
    fresh: bool = False


class PolarHistogramNode(Node):
    """Maintain separate depth and LiDAR histograms in the camera-facing frame."""

    def __init__(self):
        super().__init__('histogram', namespace='detection_improved/polar_hist')
        defaults = {
            'object_topic': '/detection_improved/object/detections',
            'obstacle_topic': '/detection_improved/obstacle/detections',
            'bin_degrees': 2.0, 'rate_hz': 5.0, 'stale_after_s': 0.8,
            'gap_min_width_m': 0.35, 'side_margin_m': 0.0, 'display_range_m': 3.0,
            'frame_prefix': 'detection_improved_hist', 'plot_frame_id': 'base_link',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.value = lambda name: self.get_parameter(name).value
        if not (0 < self.value('rate_hz') <= 30 and self.value('stale_after_s') > 0
                and self.value('gap_min_width_m') > 0 and self.value('side_margin_m') >= 0
                and math.isfinite(self.value('display_range_m'))
                and self.value('display_range_m') > 0 and self.value('frame_prefix')
                and self.value('plot_frame_id')):
            raise ValueError('Invalid histogram, gap, rate, or timeout parameters')
        self.broadcaster = StaticTransformBroadcaster(self)
        self.transforms = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sources, self.subscriptions_by_source = {}, []
        for key, topic_parameter in (('object', 'object_topic'), ('obstacle', 'obstacle_topic')):
            self.sources[key] = Source(
                PolarHistogram(self.value('bin_degrees'), smoothing_s=0, release_s=1e-6),
                self.create_publisher(CompressedImage, key + '/histogram/compressed', 1),
                self.create_publisher(LaserScan, key + '/scan', 1),
                self.create_publisher(Float32MultiArray, key + '/values', 1),
                self.create_publisher(String, key + '/gaps', 1))
            self.subscriptions_by_source.append(self.create_subscription(
                String, self.value(topic_parameter),
                lambda message, name=key: self.receive(name, message), 1))
        self.timer = self.create_timer(1.0 / self.value('rate_hz'), self.tick)
        self.get_logger().info('Measured histograms ready: depth and LiDAR; plot range 0-3 m')

    def receive(self, key, message):
        """Validate detector output and compute gaps only from fresh measured geometry."""
        state = self.sources[key]
        now = time.monotonic()
        try:
            payload = json.loads(message.data)
            if payload['schema_version'] != 1:
                raise ValueError('Unsupported detection schema')
            if not payload['fresh']:
                raise ValueError(payload['status'])
            stamp = payload['stamp']
            acquisition_ns = stamp['sec'] * 1_000_000_000 + stamp['nanosec']
            age = (self.get_clock().now().nanoseconds - acquisition_ns) / 1e9
            if acquisition_ns <= 0 or not -0.15 <= age <= self.value('stale_after_s'):
                raise ValueError(f'Detection acquisition stale/future: age {age:+.2f}s')
            fov = payload['field_of_view']
            if (not payload['frame_id'] or len(fov) != 2 or not np.isfinite(fov).all()
                    or not -math.pi/2 < fov[0] < fov[1] < math.pi/2):
                raise ValueError('Invalid camera frame or field of view')
            blockers = []
            for item in (Detection(**entry) for entry in payload['detections']):
                clipped = clip_to_fov(item.angle_min, item.angle_max, fov)
                if clipped is None or not item.angle_min <= item.bearing <= item.angle_max:
                    raise ValueError('Invalid measured obstacle geometry')
                low, high = clipped
                item = replace(item, angle_min=low, angle_max=high,
                               bearing=min(max(item.bearing, low), high))
                blockers.append(item)
                if not (np.isfinite([item.angle_min, item.angle_max, item.bearing,
                                     item.distance_m, item.width_m, item.confidence,
                                     item.width_std_m, item.range_std_m]).all()
                        and item.angle_min < item.angle_max
                        and item.distance_m > 0 and item.width_m > 0
                        and item.width_std_m >= 0 and item.range_std_m >= 0
                        and 0 <= item.confidence <= 1):
                    raise ValueError('Invalid measured obstacle geometry')
            unknown = []
            for interval in payload['unknown_intervals']:
                clipped = clip_to_fov(interval['angle_min'], interval['angle_max'], fov)
                if clipped is None:
                    raise ValueError('Invalid unknown silhouette interval')
                unknown.append(dict(interval, angle_min=clipped[0], angle_max=clipped[1]))
            low_range, high_range = payload['range_min'], payload['range_max']
            if not np.isfinite([low_range, high_range]).all() or not 0 < low_range < high_range:
                raise ValueError('Invalid sensor range')
            if state.frame and state.frame != payload['frame_id']:
                raise ValueError(
                    'Camera frame changed; restart the histogram node to update static TF')
            state.histogram.range_min, state.histogram.range_max = low_range, high_range
            state.histogram.clear_unknown()
            state.histogram.update(blockers, fov, now)
            for interval in unknown:
                edges_low = state.histogram.angles - state.histogram.step / 2
                edges_high = state.histogram.angles + state.histogram.step / 2
                hit = ((edges_high >= interval['angle_min'])
                       & (edges_low <= interval['angle_max']))
                state.histogram.observed[hit] = False
                state.histogram.ranges[hit] = np.nan
                state.histogram.scores[hit] = 0
            state.gaps = find_gaps(blockers, unknown, self.value('gap_min_width_m'),
                                   self.value('side_margin_m'))
            state.header = Header()
            state.header.stamp.sec, state.header.stamp.nanosec = stamp['sec'], stamp['nanosec']
            if not state.frame:
                self.attach_frame(key, payload['frame_id'])
            state.frame = payload['frame_id']
            state.status = payload['status']
            # Receipt of a delayed message cannot extend the acquisition lifetime.
            state.received = now - max(0, age)
            state.fresh = True
        except (ValueError, TypeError, KeyError, AttributeError) as error:
            self.invalidate(state, str(error))
        self.publish(key)

    def attach_frame(self, key, parent):
        """Attach forward/left/up scan coordinates to the RGB optical frame."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent
        transform.child_frame_id = self.value('frame_prefix') + '_' + key
        transform.transform.rotation.x = 0.5
        transform.transform.rotation.y = -0.5
        transform.transform.rotation.z = 0.5
        transform.transform.rotation.w = 0.5
        # Humble's broadcaster does not retain prior children automatically.
        # Send both children together so a late Vizanti client receives both.
        self.transforms[key] = transform
        self.broadcaster.sendTransform(list(self.transforms.values()))

    @staticmethod
    def invalidate(state, status):
        """Remove every green passage and invalidate observed directions."""
        state.histogram.clear_unknown()
        state.gaps, state.fresh, state.status = (), False, status

    def tick(self):
        """Publish plots and clear stopped streams even if no more callbacks arrive."""
        for key, state in self.sources.items():
            if time.monotonic() - state.received > self.value('stale_after_s'):
                self.invalidate(state, 'Input stale or unavailable: gaps unknown')
                self.publish(key)
            if state.plot_pub.get_subscription_count():
                self.publish_plot(key)

    def publish(self, key):
        """Publish the native scan, normalized values, and passage geometry."""
        state = self.sources[key]
        histogram = state.histogram
        scan = LaserScan()
        scan.header.stamp = (state.header.stamp if state.fresh and state.header
                             else self.get_clock().now().to_msg())
        scan.header.frame_id = self.value('frame_prefix') + '_' + key
        scan.angle_min, scan.angle_max = float(histogram.angles[0]), float(histogram.angles[-1])
        scan.angle_increment = histogram.step
        scan.scan_time = 1.0 / self.value('rate_hz')
        scan.range_min, scan.range_max = histogram.range_min, histogram.range_max
        scan.ranges = histogram.ranges.tolist()
        scan.intensities = histogram.values().tolist()
        state.scan_pub.publish(scan)
        state.values_pub.publish(Float32MultiArray(data=histogram.values().tolist()))
        state.gaps_pub.publish(String(data=json.dumps({
            'schema_version': 1, 'fresh': state.fresh, 'status': state.status,
            'frame_id': scan.header.frame_id,
            'stamp': {'sec': scan.header.stamp.sec, 'nanosec': scan.header.stamp.nanosec},
            'required_width_m': self.value('gap_min_width_m'),
            'side_margin_m': self.value('side_margin_m'),
            'gaps': [gap.to_dict() for gap in state.gaps],
        }, allow_nan=False)))

    def publish_plot(self, key, header=None):
        """Render only when Vizanti or another client subscribes."""
        state = self.sources[key]
        if header is None:
            header = Header()
            header.frame_id = self.value('frame_prefix') + '_' + key
            header.stamp = state.header.stamp if state.header else self.get_clock().now().to_msg()
        title = 'RGB + DEPTH' if key == 'object' else 'RGB + LIDAR'
        yaw, histogram, gaps, status = 0.0, state.histogram, state.gaps, state.status
        if state.fresh:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.value('plot_frame_id'), state.frame, Time.from_msg(header.stamp))
                rotation, _ = transform_arrays(transform.transform)
                forward = rotation[:, 2]
                if np.linalg.norm(forward[:2]) < 0.1:
                    raise ValueError('Camera points vertically; horizontal robot angle undefined')
                yaw = math.atan2(forward[1], forward[0])
            except (ValueError, TransformException):
                histogram = PolarHistogram(self.value('bin_degrees'))
                gaps = []
                status = 'Waiting for camera pose in ' + self.value('plot_frame_id')
        image = histogram_image(histogram, gaps, title, status,
                                self.value('display_range_m'), yaw)
        publish_jpeg(state.plot_pub, image, header)


def main(args=None):
    """Run the histogram and gap visualization node."""
    rclpy.init(args=args)
    node = PolarHistogramNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
