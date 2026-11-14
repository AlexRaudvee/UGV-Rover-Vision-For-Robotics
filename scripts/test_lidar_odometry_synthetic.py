#!/usr/bin/env python3
"""Exercise installed fusion on isolated ROS domain 87. Starts no hardware/drivers.

Run inside the sourced ROS container: python3 scripts/test_lidar_odometry_synthetic.py
"""
import math
import os
import signal
import subprocess
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = '87'

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32MultiArray
from tf2_ros import StaticTransformBroadcaster


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def ranges(x, y, heading, count, angle_min):
    """Ray trace an asymmetric room plus two fixed box obstacles."""
    segments = [(-4, -3, 6, -3), (6, -3, 6, 5), (6, 5, -4, 5), (-4, 5, -4, -3),
                (2, 1, 2, 2), (2, 2, 3, 2), (3, 2, 3, 1), (3, 1, 2, 1),
                (-2, -1, -2, 0), (-2, 0, -1, 0), (-1, 0, -1, -1), (-1, -1, -2, -1)]
    # Nonzero scanner translation and yaw test virtual-scan geometry and lever arm.
    ox = x + math.cos(heading)*0.14 + math.sin(heading)*0.05
    oy = y + math.sin(heading)*0.14 - math.cos(heading)*0.05
    result = []
    for index in range(count):
        angle = heading + 0.23 + angle_min + index * 2*math.pi/(count-1)
        dx, dy = math.cos(angle), math.sin(angle)
        distance = 20.0
        for ax, ay, bx, by in segments:
            sx, sy = bx-ax, by-ay
            cross = dx*sy - dy*sx
            if abs(cross) < 1e-9:
                continue
            t = ((ax-ox)*sy - (ay-oy)*sx)/cross
            u = ((ax-ox)*dy - (ay-oy)*dx)/cross
            if t > 0 and 0 <= u <= 1:
                distance = min(distance, t)
        result.append(distance)
    return result


def main():
    rclpy.init()
    node = rclpy.create_node('synthetic_odometry_test')
    scan_pub = node.create_publisher(LaserScan, '/scan', 10)
    wheel_pub = node.create_publisher(Float32MultiArray, '/odom/odom_raw', 10)
    imu_pub = node.create_publisher(Imu, '/imu/data', 10)
    received = {'lidar': [], 'fused': [], 'diagnostics': []}
    node.create_subscription(Odometry, '/odom_rf2o', lambda m: received['lidar'].append(m), 100)
    node.create_subscription(Odometry, '/odom', lambda m: received['fused'].append(m), 100)
    node.create_subscription(DiagnosticArray, '/diagnostics',
                             lambda m: received['diagnostics'].append(m), 100)
    broadcaster = StaticTransformBroadcaster(node)
    transforms = []
    for frame, tx, ty, theta in [('base_lidar_link', .14, -.05, .23),
                                  ('base_imu_link', 0., 0., 0.)]:
        tf = TransformStamped()
        tf.header.stamp = node.get_clock().now().to_msg()
        tf.header.frame_id = 'base_footprint'
        tf.child_frame_id = frame
        tf.transform.translation.x, tf.transform.translation.y = tx, ty
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(theta/2), math.cos(theta/2)
        transforms.append(tf)
    broadcaster.sendTransform(transforms)
    logfile = tempfile.NamedTemporaryFile(prefix='synthetic-odometry-', suffix='.log', delete=False)
    process = subprocess.Popen(['ros2', 'launch', 'ugv_bringup', 'odometry_lidar.launch.py'],
                               stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True)
    x = y = heading = wheel_distance = 0.0
    angle_min = 0.0
    count = 720

    def step(vx=0., vy=0., rate=0., send_scan=True, invalid=False):
        nonlocal x, y, heading, wheel_distance
        dt = 0.1
        x += (math.cos(heading)*vx - math.sin(heading)*vy)*dt
        y += (math.sin(heading)*vx + math.cos(heading)*vy)*dt
        heading += rate*dt
        wheel_distance += vx*1.30*dt  # Deliberate 30% wheel scale error.
        stamp = node.get_clock().now().to_msg()
        wheel_pub.publish(Float32MultiArray(data=[wheel_distance, wheel_distance]))
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, 'base_imu_link'
        imu.orientation.w = 1.0
        imu.angular_velocity.z = rate
        imu_pub.publish(imu)
        if send_scan:
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, 'base_lidar_link'
            scan.angle_min, scan.angle_max = angle_min, angle_min+2*math.pi
            scan.angle_increment = 2*math.pi/(count-1)
            scan.range_min, scan.range_max = .05, 20.
            scan.ranges = ([float('nan')]*count if invalid
                           else ranges(x, y, heading, count, angle_min))
            scan_pub.publish(scan)
        deadline = time.monotonic() + dt
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.005)
        assert process.poll() is None, f'Launch exited; see {logfile.name}'

    def check_pose(label, tolerance=.10):
        assert received['lidar'] and received['fused'], f'No odometry; see {logfile.name}'
        for source in ('lidar', 'fused'):
            msg = received[source][-1]
            error = math.hypot(msg.pose.pose.position.x-x, msg.pose.pose.position.y-y)
            angle_error = abs(math.atan2(math.sin(yaw(msg.pose.pose.orientation)-heading),
                                         math.cos(yaw(msg.pose.pose.orientation)-heading)))
            print(f'{label}: {source} position error={error:.4f} m, yaw error={angle_error:.4f} rad')
            assert error < tolerance and angle_error < .10, (label, source, error, angle_error)
            assert all(math.isfinite(v) for v in msg.twist.covariance)
        assert received['lidar'][-1].twist.covariance[0] > 0

    try:
        for _ in range(40):
            step()
        check_pose('stationary', .03)
        assert node.count_publishers('/imu/data') == 1, 'RF2O must not impersonate the IMU'
        assert node.count_publishers('/odom') == 1, 'Competing odometry publishers'
        for _ in range(35):
            step(.15)
        for _ in range(10):
            step()
        check_pose('forward with 30% wheel error')
        for _ in range(30):
            step(.10, rate=.15)
        for _ in range(10):
            step()
        check_pose('curved motion')
        for _ in range(20):
            step(-.12, .06, -.10)
        for _ in range(10):
            step()
        check_pose('reverse and lateral motion')
        angle_min = -math.pi
        count = 1080
        for _ in range(10):
            step()
        for _ in range(20):
            step(.12)
        for _ in range(10):
            step()
        check_pose('centered scan and changed beam count')
        # Real LD19 revolutions fluctuate by a few rays. Every scan must still
        # contribute odometry rather than reinitializing on each count change.
        for _ in range(10):
            count = 501
            step()
        before = len(received['lidar'])
        for index in range(30):
            count = 500 + index % 4
            step(.10)
        assert len(received['lidar']) - before >= 24, 'Beam-count jitter reset RF2O scan pairs'
        check_pose('LD19 fluctuating beam count')
        before = len(received['lidar'])
        for _ in range(12):
            step(invalid=True)
        assert len(received['lidar']) <= before+1, 'Invalid scans generated odometry'
        statuses = [s for m in received['diagnostics'][-10:] for s in m.status
                    if s.name == 'odometry/input_freshness']
        assert any('lidar' in s.message for s in statuses), 'Missing LiDAR diagnostic absent'
        for _ in range(12):
            step()
        check_pose('recovered after scan outage')
        # Stop all inputs: adapters must not manufacture fresh sensor measurements.
        wheel_count = []
        node.create_subscription(Odometry, '/odom/wheels', lambda m: wheel_count.append(m), 10)
        end = time.monotonic()+1.
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        assert len(wheel_count) <= 1
        statuses = [s for m in received['diagnostics'][-4:] for s in m.status
                    if s.name == 'odometry/input_freshness']
        assert any(all(k in s.message for k in ('wheels', 'imu', 'lidar')) for s in statuses)
        print('PASS: isolated synthetic fusion, signs, geometry, outages and diagnostics')
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        node.destroy_node()
        rclpy.shutdown()
        logfile.close()
        print(f'ROS launch log: {logfile.name}')


if __name__ == '__main__':
    main()
