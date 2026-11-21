"""Drive the rover and move its camera with a game controller plugged into the LAPTOP.

Sends commands to the rover through Vizanti's rosbridge (port 5001), so the
laptop needs no ROS. The rover session (start_rover.sh) must be running.

    <venv python> gamepad_drive.py            drive
    <venv python> gamepad_drive.py --show     only print sticks/buttons (to check the mapping)

Xbox 360 layout:
    hold RT         enable driving (press more than halfway; let go = stop)
    left stick      up/down = forward/backward, left/right = turn
    right stick     move the camera: left/right = pan, up/down = tilt
                    (no RT needed; automatic camera tracking resumes 3 s after you let go)
    B               camera back to straight ahead
    Y               emergency stop (let go of RT, then press it again to continue)
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
PAN_RATE, TILT_RATE = 60.0, 40.0        # deg/s at full stick
PAN_LIMIT, TILT_MIN, TILT_MAX = 150.0, -20.0, 60.0
PAN_SIGN, TILT_SIGN = -1.0, 1.0         # same as pan_tracking.yaml
DEADZONE = 0.12
RATE_HZ = 20

AXIS_FORWARD, AXIS_TURN = 1, 0          # left stick Y, left stick X
AXIS_PAN, AXIS_TILT = 2, 3              # right stick X, right stick Y
AXIS_ENABLE = 5                         # right trigger RT: -1 released, +1 fully pressed
ENABLE_ABOVE = 0.0                      # RT pressed more than halfway
BUTTON_STOP, BUTTON_CENTER, BUTTON_QUIT = 3, 1, 6   # Y, B, Back
PAN_JOINT, TILT_JOINT = 'pt_base_link_to_pt_link1', 'pt_link1_to_pt_link2'


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
    camera_cmd = roslibpy.Topic(ros, '/localization/pan_tilt_manual', 'sensor_msgs/JointState')
    cmd.advertise()
    camera_cmd.advertise()

    # Current camera angles (deg, pan left positive), kept in sync with what the
    # rover actually sends to the servos, so manual moves start from there.
    camera = {'pan': 0.0, 'tilt': 0.0, 'manual_until': 0.0}

    def on_commanded(message):
        if time.monotonic() < camera['manual_until']:
            return                                   # we are steering: keep our own values
        try:
            names = message['name']
            camera['pan'] = math.degrees(message['position'][names.index(PAN_JOINT)]) / PAN_SIGN
            camera['tilt'] = math.degrees(message['position'][names.index(TILT_JOINT)]) / TILT_SIGN
        except (KeyError, ValueError, IndexError):
            pass

    roslibpy.Topic(ros, '/ugv/pan_tilt_commanded', 'sensor_msgs/JointState').subscribe(on_commanded)

    def send(v, w):
        cmd.publish(roslibpy.Message({'linear': {'x': v, 'y': 0.0, 'z': 0.0},
                                      'angular': {'x': 0.0, 'y': 0.0, 'z': w}}))

    def send_camera():
        camera_cmd.publish(roslibpy.Message({
            'header': {'stamp': {'sec': 0, 'nanosec': 0}, 'frame_id': ''},
            'name': [PAN_JOINT, TILT_JOINT],
            'position': [PAN_SIGN * math.radians(camera['pan']), TILT_SIGN * math.radians(camera['tilt'])],
            'velocity': [], 'effort': []}))

    print('Connected. Hold RT to drive, right stick = camera, B = camera centre, Y = stop, Back = quit.')
    stopped, was_driving, armed = False, False, False
    dt = 1.0 / RATE_HZ
    try:
        while ros.is_connected:
            pygame.event.pump()
            if pad.get_button(BUTTON_QUIT):
                break

            # ---- wheels
            trigger = pad.get_axis(AXIS_ENABLE)
            if trigger < -0.5:
                armed = True                          # RT seen released at least once
            enable = armed and trigger > ENABLE_ABOVE
            if pad.get_button(BUTTON_STOP):
                stopped = True
            if not enable:
                stopped = False                       # releasing RT clears the emergency stop
            if enable and not stopped:
                v = -shaped(pad.get_axis(AXIS_FORWARD)) * MAX_SPEED
                w = -shaped(pad.get_axis(AXIS_TURN)) * MAX_TURN
                send(v, w)
                was_driving = True
                wheels = f'DRIVING v={v:+.2f} m/s w={w:+.2f} rad/s'
            else:
                if was_driving:
                    for _ in range(5):
                        send(0.0, 0.0)
                    was_driving = False
                wheels = ('STOPPED (let go of RT)' if stopped else 'idle (hold RT)' if armed
                          else 'press and release RT once')

            # ---- camera
            pan_in, tilt_in = shaped(pad.get_axis(AXIS_PAN)), shaped(pad.get_axis(AXIS_TILT))
            if pad.get_button(BUTTON_CENTER):
                camera['pan'], camera['tilt'] = 0.0, 0.0
                camera['manual_until'] = time.monotonic() + 0.5
                send_camera()
            elif pan_in or tilt_in:
                camera['pan'] = max(-PAN_LIMIT, min(PAN_LIMIT, camera['pan'] - pan_in * PAN_RATE * dt))
                camera['tilt'] = max(TILT_MIN, min(TILT_MAX, camera['tilt'] - tilt_in * TILT_RATE * dt))
                camera['manual_until'] = time.monotonic() + 0.5
                send_camera()
            manual = time.monotonic() < camera['manual_until']
            cam = f'camera pan {camera["pan"]:+5.0f} tilt {camera["tilt"]:+4.0f}' + (' (manual)' if manual else '')

            print(f'\r{wheels:36} | {cam:40}', end='', flush=True)
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        if ros.is_connected:
            for _ in range(5):
                send(0.0, 0.0)
                time.sleep(0.02)
            cmd.unadvertise()
            camera_cmd.unadvertise()
            ros.terminate()
        print('\nStopped and disconnected.')


if __name__ == '__main__':
    main()
