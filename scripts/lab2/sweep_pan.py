"""Step the pan-tilt camera through fixed angles (for a recorded rosbag).

Publishes on /localization/pan_tilt_command, which the driver gives priority
over /ugv/joint_states. Run only while the automatic pan tracker is stopped.
Angles: left positive, as in the rest of the code; joint = pan_sign * angle.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

PAN_SIGN = -1.0                      # pan_tracking.yaml / marker_ranges.yaml
STOPS_DEG = [0, 30, 60, 90, 120, 150, 180, -150, -120, -90, -60, -30, 0]
DWELL_S = 3.0                        # per stop: ~1 s to settle + ~2 s of sharp frames
RATE_HZ = 20.0


def main():
    rclpy.init()
    node = Node('pan_sweep_recorder')
    pub = node.create_publisher(JointState, '/localization/pan_tilt_command', 1)

    def send(angle_deg):
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = ['pt_base_link_to_pt_link1', 'pt_link1_to_pt_link2']
        msg.position = [PAN_SIGN * math.radians(angle_deg), 0.0]
        pub.publish(msg)

    time.sleep(1.0)                  # let the publisher connect
    for angle in STOPS_DEG:
        print(f'pan {angle:+4d} deg', flush=True)
        end = time.monotonic() + DWELL_S
        while time.monotonic() < end:
            send(angle)
            time.sleep(1.0 / RATE_HZ)
    print('sweep done', flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
