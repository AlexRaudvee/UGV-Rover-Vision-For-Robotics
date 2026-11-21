"""Drive the rover with a game controller plugged into the LAPTOP.

Sends /cmd_vel to the rover through Vizanti's rosbridge (port 5001), so the
laptop needs no ROS. The rover session (start_rover.sh) must be running.

    <venv python> gamepad_drive.py            drive
    <venv python> gamepad_drive.py --show     only print sticks/buttons (to check the mapping)

Xbox 360 layout:
    hold LB         enable driving (let go = stop)
    left stick      forward / backward
    right stick     turn left / right
    B               emergency stop (let go of LB, then hold it again to continue)
    Back            quit
"""
import math
import os
import sys
import time

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame
import roslibpy

ROVER = '10.0.80.4'
PORT = 5001
MAX_SPEED = 0.20           # m/s
MAX_TURN = 1.5             # rad/s commanded (the rover turns at about 1/3 of this)
DEADZONE = 0.12
RATE_HZ = 20

AXIS_FORWARD, AXIS_TURN = 1, 2          # left stick Y, right stick X
BUTTON_ENABLE, BUTTON_STOP, BUTTON_QUIT = 4, 1, 6   # LB, B, Back


def shaped(value):
    """Deadzone, then squared response for fine control near the centre."""
    if abs(value) < DEADZONE:
        return 0.0
    value = (abs(value) - DEADZONE) / (1 - DEADZONE) * math.copysign(1, value)
    return math.copysign(value * value, value)


def open_controller():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        sys.exit('No controller found. Plug it into the laptop and try again.')
    pad = pygame.joystick.Joystick(0)
    pad.init()
    print(f'Controller: {pad.get_name()}')
    return pad


def show(pad):
    print('Move sticks / press buttons. Ctrl+C to quit.')
    while True:
        pygame.event.pump()
        axes = ' '.join(f'{i}:{pad.get_axis(i):+.2f}' for i in range(pad.get_numaxes()))
        buttons = ' '.join(str(i) for i in range(pad.get_numbuttons()) if pad.get_button(i))
        print(f'\raxes {axes} | buttons pressed: {buttons or "-":10}', end='', flush=True)
        time.sleep(0.05)


def main():
    pad = open_controller()
    if '--show' in sys.argv:
        try:
            show(pad)
        except KeyboardInterrupt:
            return
    ros = roslibpy.Ros(host=ROVER, port=PORT)
    try:
        ros.run(timeout=5)
    except Exception:  # noqa: BLE001
        sys.exit(f'Cannot reach the rover at ws://{ROVER}:{PORT}. Is start_rover.sh running '
                 'and Proton VPN off?')
    cmd = roslibpy.Topic(ros, '/cmd_vel', 'geometry_msgs/Twist')
    cmd.advertise()

    def send(v, w):
        cmd.publish(roslibpy.Message({'linear': {'x': v, 'y': 0.0, 'z': 0.0},
                                      'angular': {'x': 0.0, 'y': 0.0, 'z': w}}))

    print('Connected. Hold LB to drive, B = stop, Back = quit.')
    stopped, was_driving = False, False
    try:
        while ros.is_connected:
            pygame.event.pump()
            if pad.get_button(BUTTON_QUIT):
                break
            enable = pad.get_button(BUTTON_ENABLE)
            if pad.get_button(BUTTON_STOP):
                stopped = True
            if not enable:
                stopped = False                       # releasing LB clears the emergency stop
            if enable and not stopped:
                v = -shaped(pad.get_axis(AXIS_FORWARD)) * MAX_SPEED
                w = -shaped(pad.get_axis(AXIS_TURN)) * MAX_TURN
                send(v, w)
                was_driving = True
                state = f'DRIVING  v={v:+.2f} m/s  w={w:+.2f} rad/s'
            else:
                if was_driving:
                    for _ in range(5):
                        send(0.0, 0.0)
                    was_driving = False
                state = 'STOPPED (B pressed - let go of LB)' if stopped else 'idle (hold LB to drive)'
            print(f'\r{state:60}', end='', flush=True)
            time.sleep(1.0 / RATE_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        if ros.is_connected:
            for _ in range(5):
                send(0.0, 0.0)
                time.sleep(0.02)
            cmd.unadvertise()
            ros.terminate()
        print('\nStopped and disconnected.')


if __name__ == '__main__':
    main()
