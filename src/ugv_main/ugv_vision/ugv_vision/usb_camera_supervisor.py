"""Recover usb_cam after device disconnects and re-enumeration."""

import argparse
import os
import signal
import subprocess
import threading

from ament_index_python.packages import get_package_prefix


def camera_command(driver, ros_arguments, device):
    """Resolve the stable device identifier on every capture-process restart."""
    arguments = list(ros_arguments)
    if '--ros-args' not in arguments:
        arguments.append('--ros-args')
    return [driver, *arguments, '-p', 'video_device:=' + os.path.realpath(device)]


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', required=True)
    options, ros_arguments = parser.parse_known_args(args)
    driver = os.path.join(get_package_prefix('usb_cam'), 'lib', 'usb_cam', 'usb_cam_node_exe')
    stopping = threading.Event()
    child = None

    def stop(signum, frame):
        stopping.set()
        if child is not None and child.poll() is None:
            try:
                child.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    waiting_logged = False
    while not stopping.is_set():
        if not os.path.exists(options.device):
            if not waiting_logged:
                print('USB camera disconnected; waiting for ' + options.device, flush=True)
                waiting_logged = True
            stopping.wait(3.0)
            continue
        waiting_logged = False
        command = camera_command(driver, ros_arguments, options.device)
        print('Starting USB capture on ' + os.path.realpath(options.device), flush=True)
        try:
            child = subprocess.Popen(command)
        except OSError as error:
            print('USB capture could not start: ' + str(error), flush=True)
            stopping.wait(3.0)
            continue
        while child.poll() is None and not stopping.is_set():
            stopping.wait(0.2)
        if stopping.is_set():
            try:
                child.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                child.terminate()
                try:
                    child.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
            break
        print(f'USB capture exited ({child.returncode}); retrying in 3 s', flush=True)
        stopping.wait(3.0)
