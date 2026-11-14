#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial  
import json  
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray
import subprocess
import time
import math
import os

serial_port = os.environ.get('UGV_SERIAL_PORT', '/dev/ttyTHS1' if os.path.exists('/dev/ttyTHS1') else '/dev/ttyAMA0')

class UgvDriver(Node):
    def __init__(self, name='ugv_driver', serial_connection=None):
        super().__init__(name)
        self.declare_parameter('command_timeout', 0.5)
        self.command_timeout = self.get_parameter('command_timeout').value
        if not math.isfinite(self.command_timeout) or self.command_timeout <= 0:
            raise ValueError('command_timeout must be finite and positive')
        self.ser = serial_connection if serial_connection is not None else serial.Serial(
            serial_port, 115200, timeout=1, write_timeout=.2)
        self.last_command, self.motion_active = None, False
        self.last_voltage_alarm = -math.inf
        self.create_timer(.05, self.check_command_timeout)

        # Subscribe to velocity commands (cmd_vel topic)
        self.cmd_vel_sub_ = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

        # Subscribe to joint states (ugv/joint_states topic)
        self.joint_states_sub = self.create_subscription(JointState, 'ugv/joint_states', self.joint_states_callback, 10)

        # Subscribe to LED control data (ugv/led_ctrl topic)
        self.led_ctrl_sub = self.create_subscription(Float32MultiArray, 'ugv/led_ctrl', self.led_ctrl_callback, 10)

        # Subscribe to voltage data (voltage topic)
        self.voltage_sub = self.create_subscription(Float32, 'voltage', self.voltage_callback, 10)

    # Callback for processing velocity commands
    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        if not math.isfinite(linear_velocity) or not math.isfinite(angular_velocity):
            self.stop_motion()
            return
        # Send the velocity data to the UGV as a JSON string
        data = json.dumps({'T': '13', 'X': linear_velocity, 'Z': angular_velocity}) + "\n"
        self.ser.write(data.encode())
        self.last_command = time.monotonic()
        self.motion_active = linear_velocity != 0 or angular_velocity != 0

    def stop_motion(self):
        self.ser.write((json.dumps({'T': '13', 'X': 0.0, 'Z': 0.0}) + '\n').encode())
        self.motion_active = False

    def check_command_timeout(self):
        if self.motion_active and time.monotonic() - self.last_command > self.command_timeout:
            self.stop_motion()
            self.get_logger().warning('Velocity command timed out; stopped motors')

    # Callback for processing joint state updates
    def joint_states_callback(self, msg):
        header = {
            'stamp': {
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec,
            },
            'frame_id': msg.header.frame_id,
        }

        # Extract joint positions and convert to degrees
        name = msg.name
        position = msg.position

        x_rad = position[name.index('pt_base_link_to_pt_link1')]
        y_rad = position[name.index('pt_link1_to_pt_link2')]

        x_degree = (180 * x_rad) / 3.1415926
        y_degree = (180 * y_rad) / 3.1415926

        # Send the joint data as a JSON string to the UGV
        joint_data = json.dumps({
            'T': 134, 
            'X': x_degree, 
            'Y': y_degree, 
            "SX": 600,
            "SY": 600,
        }) + "\n"
                
        self.ser.write(joint_data.encode())

    # Callback for processing LED control commands
    def led_ctrl_callback(self, msg):
        IO4 = msg.data[0]
        IO5 = msg.data[1]
        
        # Send LED control data as a JSON string to the UGV
        led_ctrl_data = json.dumps({
            'T': 132, 
            "IO4": IO4,
            "IO5": IO5,
        }) + "\n"
                
        self.ser.write(led_ctrl_data.encode())

    # Callback for processing voltage data
    def voltage_callback(self, msg):
        voltage_value = msg.data

        # If voltage drops below a threshold, play a low battery warning sound
        if 0.1 < voltage_value < 9 and time.monotonic() - self.last_voltage_alarm > 5:
            self.last_voltage_alarm = time.monotonic()
            # Keep this callback nonblocking so the command watchdog can run.
            try:
                subprocess.Popen(['aplay', '-D', 'plughw:3,0', os.path.expanduser('~/ugv_ws/src/ugv_main/ugv_bringup/ugv_bringup/low_battery.wav')])
            except OSError as error:
                self.get_logger().warning(f'Battery sound unavailable: {error}')

def main(args=None):
    rclpy.init(args=args)
    node = UgvDriver("ugv_driver")
    
    try:
        rclpy.spin(node)  # Keep the node running and handling callbacks
    except KeyboardInterrupt:
        pass  # Graceful shutdown on user interrupt
    finally:
        try:
            node.stop_motion()
        finally:
            node.ser.close()
            node.destroy_node()
            rclpy.try_shutdown()  # Close the serial connection

if __name__ == '__main__':
    main()
