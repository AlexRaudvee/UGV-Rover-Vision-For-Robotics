"""Drive a slow left arc (U-turn) by /odom, then stop.

Stops when the heading has turned TURN_DEG, or after MAX_DISTANCE_M, or after
TIME_LIMIT_S. No odometry -> no motion. Zero velocity is always sent at the end.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

SPEED = 0.15                 # m/s
RADIUS = 1.5                 # m, left
TURN_DEG = 90.0
MAX_DISTANCE_M = 2.6
TIME_LIMIT_S = 25.0
RATE_HZ = 20.0
CHECK_AFTER_M = 1.0          # safety: by now the heading must have changed
MIN_TURN_AT_CHECK_DEG = 15.0 # (expected ~38 deg at R = 1.5 m)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    node = Node('drive_arc_recorder')
    pub = node.create_publisher(Twist, '/cmd_vel', 1)
    state = {'pose': None}

    def on_odom(msg):
        p = msg.pose.pose.position
        state['pose'] = (p.x, p.y, yaw_of(msg.pose.pose.orientation))

    node.create_subscription(Odometry, '/odom', on_odom, qos_profile_sensor_data)

    def send(v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        pub.publish(t)

    try:
        deadline = time.monotonic() + 3.0
        while state['pose'] is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if state['pose'] is None:
            print('NO ODOMETRY - not driving', flush=True)
            return 1
        x0, y0, h0 = state['pose']
        print(f'start odom ({x0:.3f}, {y0:.3f}, {math.degrees(h0):.1f} deg); arc R={RADIUS} m left',
              flush=True)
        t0 = time.monotonic()
        turned, travelled, last = 0.0, 0.0, state['pose']
        not_turning = False
        while (turned < math.radians(TURN_DEG) and travelled < MAX_DISTANCE_M
               and time.monotonic() - t0 < TIME_LIMIT_S):
            send(SPEED, SPEED / RADIUS)
            rclpy.spin_once(node, timeout_sec=1.0 / RATE_HZ)
            x, y, h = state['pose']
            travelled += math.hypot(x - last[0], y - last[1])
            turned += math.atan2(math.sin(h - last[2]), math.cos(h - last[2]))
            last = state['pose']
            if travelled >= CHECK_AFTER_M and turned < math.radians(MIN_TURN_AT_CHECK_DEG):
                not_turning = True       # driving straight instead of curving: abort
                break
        reason = ('NOT TURNING - aborted' if not_turning else
                  'turn reached' if turned >= math.radians(TURN_DEG) else
                  'distance limit' if travelled >= MAX_DISTANCE_M else 'TIME LIMIT')
        print(f'stop: {reason}; turned {math.degrees(turned):.1f} deg, {travelled:.2f} m, '
              f'{time.monotonic() - t0:.1f} s', flush=True)
    finally:
        for _ in range(20):
            send(0.0, 0.0)
            time.sleep(0.05)
    time.sleep(1.0)
    rclpy.spin_once(node, timeout_sec=0.2)
    if state['pose'] is not None:
        x, y, h = state['pose']
        print(f'end odom ({x:.3f}, {y:.3f}, {math.degrees(h):.1f} deg)', flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
