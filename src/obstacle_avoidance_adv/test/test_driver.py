"""Fake UART only; never opens a device or uses the live ROS domain."""
import json
import rclpy
from geometry_msgs.msg import Twist


def test_driver_watchdog_stops_fake_uart_and_rejects_nan(monkeypatch):
    from ugv_bringup.ugv_driver import UgvDriver
    monkeypatch.setenv('ROS_DOMAIN_ID','91')
    class Serial:
        def __init__(self):self.commands=[]
        def write(self,data):self.commands.append(json.loads(data))
    rclpy.init();uart=Serial();node=UgvDriver(serial_connection=uart)
    try:
        cmd=Twist();cmd.linear.x=.1
        node.cmd_vel_callback(cmd)
        assert uart.commands[-1]['X']==.1
        node.last_command-=1.
        node.check_command_timeout()
        assert uart.commands[-1]['X']==uart.commands[-1]['Z']==0.
        cmd.linear.x=0.;cmd.angular.z=.05;node.cmd_vel_callback(cmd)
        assert uart.commands[-1]['Z']==.05  # Do not jump a ramped turn to .2.
        count=len(uart.commands);node.check_command_timeout();assert len(uart.commands)==count
        cmd.linear.x=float('nan');node.cmd_vel_callback(cmd)
        assert uart.commands[-1]['X']==uart.commands[-1]['Z']==0.
    finally:
        node.destroy_node();rclpy.try_shutdown()
