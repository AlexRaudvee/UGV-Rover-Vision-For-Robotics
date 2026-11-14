#!/usr/bin/env python3
"""Isolated domain 88: stationary rover must not coast sideways after RF2O loss."""
import os
import signal
import subprocess
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = '88'
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray


def main():
    rclpy.init()
    node = rclpy.create_node('stationary_dropout_test')
    wheel = node.create_publisher(Float32MultiArray, '/odom/odom_raw', 10)
    imu = node.create_publisher(Imu, '/imu/data', 10)
    lidar = node.create_publisher(Odometry, '/odom_rf2o', 10)
    fused, wheel_messages = [], []
    node.create_subscription(Odometry, '/odom', fused.append, 100)
    node.create_subscription(Odometry, '/odom/wheels', wheel_messages.append, 10)
    log = tempfile.NamedTemporaryFile(prefix='stationary-dropout-', suffix='.log',delete=False)
    process = subprocess.Popen(['ros2', 'launch', 'ugv_bringup', 'odometry_lidar.launch.py'],
                               stdout=log, stderr=subprocess.STDOUT)

    def spin(seconds, sideways=0.):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            stamp = node.get_clock().now().to_msg()
            wheel.publish(Float32MultiArray(data=[0., 0.]))
            message = Imu()
            message.header.stamp, message.header.frame_id = stamp, 'base_footprint'
            message.orientation.w = 1.
            message.angular_velocity_covariance[8] = .001
            imu.publish(message)
            if sideways:
                message = Odometry()
                message.header.stamp, message.header.frame_id = stamp, 'odom'
                message.child_frame_id = 'base_footprint'
                message.pose.pose.orientation.w = 1.
                message.twist.twist.linear.y = sideways
                for index in (0,7,35):
                    message.twist.covariance[index] = .0001
                lidar.publish(message)
            until = time.monotonic()+.05
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.005)
            assert process.poll() is None

    try:
        deadline = time.monotonic()+12.
        while time.monotonic()<deadline and not (fused and wheel_messages):
            spin(.25)
        assert fused and wheel_messages, 'Fusion did not start'
        assert wheel_messages[-1].twist.covariance[7] < .01
        spin(1., sideways=.12)  # Plausible biased scan matching, followed by total loss.
        assert abs(fused[-1].twist.twist.linear.y) > .03, 'Fault injection did not reach the EKF'
        spin(.7)  # LiDAR is now completely absent; only real stationary wheel/IMU feedback.
        start = fused[-1].pose.pose.position
        spin(1.5)
        end = fused[-1].pose.pose.position
        drift = ((end.x-start.x)**2+(end.y-start.y)**2)**.5
        lateral = abs(fused[-1].twist.twist.linear.y)
        print(f'After LiDAR dropout: stationary drift={drift:.4f} m, lateral velocity={lateral:.4f} m/s')
        assert drift < .03 and lateral < .02
        start = fused[-1].pose.pose.position
        spin(1., sideways=.8)  # An implausible jump must fail the statistical gate.
        end = fused[-1].pose.pose.position
        assert ((end.x-start.x)**2+(end.y-start.y)**2)**.5 < .05
        print('PASS: stationary lateral velocity is constrained after LiDAR loss; no hardware used')
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=10)
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        print('ROS log:',log.name)


if __name__ == '__main__':
    main()
