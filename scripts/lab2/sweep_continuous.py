"""Continuous back-and-forth pan sweep (triangle wave) for a fixed duration.

Publishes on /localization/pan_tilt_command (driver priority). Run only while
the automatic pan tracker is stopped. Left positive; joint = PAN_SIGN * angle.
Usage: python3 sweep_continuous.py SECONDS
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

PAN_SIGN = -1.0
LIMIT_DEG = 120.0
SPEED_DEG_S = 30.0
RATE_HZ = 20.0


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rclpy.init()
    node = Node('pan_sweep_continuous')
    pub = node.create_publisher(JointState, '/localization/pan_tilt_command', 1)
    time.sleep(0.5)
    period = 4 * LIMIT_DEG / SPEED_DEG_S           # 0 -> +L -> 0 -> -L -> 0
    t0 = time.monotonic()
    try:
        while (now := time.monotonic() - t0) < duration:
            phase = (now % period) / period
            # triangle wave starting at 0 deg, going left first
            angle = LIMIT_DEG * (4 * phase if phase < .25 else 2 - 4 * phase if phase < .75
                                 else 4 * phase - 4)
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = ['pt_base_link_to_pt_link1', 'pt_link1_to_pt_link2']
            msg.position = [PAN_SIGN * math.radians(angle), 0.0]
            pub.publish(msg)
            time.sleep(1.0 / RATE_HZ)
    finally:
        print('sweep finished', flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
