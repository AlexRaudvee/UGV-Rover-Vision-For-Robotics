#!/usr/bin/env python3
"""Hand over temporary visualization/camera helpers to the daily rover launch."""

import argparse
import os
from pathlib import Path
import signal
import time


def matches_helper(kind, arguments):
    if kind == 'usb_camera':
        return any(arg.endswith('/ugv_vision/usb_camera_supervisor') for arg in arguments)
    executable = {'marker_ranges': '/l2s2_localisation/ranges',
                  'marker_detector': '/l2s1_marker_detection/detector'}[kind]
    if any(arg.endswith(executable) for arg in arguments):
        return True
    package, filename = {
        'marker_ranges': ('l2s2_localisation', 'marker_ranges.launch.py'),
        'marker_detector': ('l2s1_marker_detection', 'marker_detection.launch.py'),
    }[kind]
    return any(arguments[i:i+3] == ['launch', package, filename]
               for i in range(len(arguments)))


def running(pid):
    try:
        status = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[0]
        return status != 'Z'
    except FileNotFoundError:
        return False


def stop_helpers(domain, directory=Path('/tmp')):
    for kind in ('marker_ranges', 'marker_detector', 'usb_camera'):
        path = directory / f'ugv_{kind}_{domain}.pid'
        if not path.exists():
            continue
        try:
            pid = int(path.read_text().strip())
            if pid <= 1:
                raise ValueError('invalid helper PID')
            arguments = Path(f'/proc/{pid}/cmdline').read_bytes().decode().strip('\0').split('\0')
            if not matches_helper(kind, arguments):
                print(f'Ignoring stale {kind} PID file: process does not match')
                path.unlink()
                continue
            os.kill(pid, signal.SIGINT)
            deadline = time.monotonic()+6
            while running(pid) and time.monotonic() < deadline:
                time.sleep(.1)
            if running(pid):
                raise RuntimeError(f'{kind} helper is still stopping; retry startup')
            print(f'Stopped standalone {kind} helper')
        except (FileNotFoundError, ProcessLookupError, ValueError):
            pass
        path.unlink(missing_ok=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=4)
    options = parser.parse_args()
    stop_helpers(options.domain)
