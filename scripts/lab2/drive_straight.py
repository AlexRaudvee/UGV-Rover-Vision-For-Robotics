"""Drive straight forward a fixed distance (by /odom), slowly, then stop.

Safety: no odometry -> no motion; hard time limit; zero velocity is always
sent at the end (also on errors / Ctrl+C).
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

DISTANCE_M = 2.0
SPEED = 0.15
TIME_LIMIT_S = 20.0
RATE_HZ = 20.0


def main():
    rclpy.init()
    node = Node('drive_straight_recorder')
    pub = node.create_publisher(Twist, '/cmd_vel', 1)
    state = {'pose': None}

    def on_odom(msg):
        p = msg.pose.pose.position
        state['pose'] = (p.x, p.y)

    node.create_subscription(Odometry, '/odom', on_odom, qos_profile_sensor_data)

    def send(v):
        t = Twist()
        t.linear.x = float(v)
        pub.publish(t)

    try:
        deadline = time.monotonic() + 3.0
        while state['pose'] is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if state['pose'] is None:
            print('NO ODOMETRY - not driving', flush=True)
            return 1
        start = state['pose']
        print(f'start odom ({start[0]:.3f}, {start[1]:.3f}); driving {DISTANCE_M} m at {SPEED} m/s',
              flush=True)
        t0 = time.monotonic()
        travelled = 0.0
        while travelled < DISTANCE_M and time.monotonic() - t0 < TIME_LIMIT_S:
            send(SPEED)
            rclpy.spin_once(node, timeout_sec=1.0 / RATE_HZ)
            x, y = state['pose']
            travelled = math.hypot(x - start[0], y - start[1])
        reason = 'distance reached' if travelled >= DISTANCE_M else 'TIME LIMIT'
        print(f'stop: {reason}, odometry says {travelled:.3f} m in {time.monotonic() - t0:.1f} s',
              flush=True)
    finally:
        for _ in range(20):                 # 1 s of explicit zeros
            send(0.0)
            time.sleep(0.05)
    time.sleep(1.0)
    rclpy.spin_once(node, timeout_sec=0.2)
    end = state['pose']
    if end is not None:
        print(f'end odom ({end[0]:.3f}, {end[1]:.3f})', flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
