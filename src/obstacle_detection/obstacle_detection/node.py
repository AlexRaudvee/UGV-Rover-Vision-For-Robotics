"""RGB/LiDAR detection using calibrated projection and measured physical width."""

import rclpy

from object_detection.node import MeasuredDetector


class ObstacleDetector(MeasuredDetector):
    """Range black silhouettes with LaserScan instead of camera depth."""

    def __init__(self):
        super().__init__('lidar')


def main(args=None):
    """Run the RGB/LiDAR detector."""
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
