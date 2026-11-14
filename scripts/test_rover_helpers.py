"""Process handover tests using disposable sleep processes, never robot nodes."""

import subprocess
from pathlib import Path

from stop_rover_helpers import matches_helper, stop_helpers


def test_only_recognized_camera_helper_is_stopped(tmp_path):
    process = subprocess.Popen(['/workspace/install/ugv_vision/lib/ugv_vision/usb_camera_supervisor', '60'],
                               executable='/usr/bin/sleep')
    path = tmp_path / 'ugv_usb_camera_4.pid'
    path.write_text(str(process.pid))
    try:
        stop_helpers(4, tmp_path)
        assert process.wait(timeout=2) != 0
        assert not path.exists()
    finally:
        if process.poll() is None:
            process.terminate()
        process.wait()


def test_reused_pid_is_not_signalled(tmp_path):
    process = subprocess.Popen(['/usr/bin/sleep', '60'])
    (tmp_path / 'ugv_usb_camera_4.pid').write_text(str(process.pid))
    try:
        stop_helpers(4, tmp_path)
        assert process.poll() is None
    finally:
        process.terminate()
        process.wait()


def test_launch_match_checks_argument_tokens():
    assert matches_helper('marker_ranges', ['python3', 'ros2', 'launch', 'l2s2_localisation', 'marker_ranges.launch.py'])
    assert not matches_helper('marker_ranges', ['bash', '-c', 'ros2 launch l2s2_localisation marker_ranges.launch.py'])
    assert matches_helper('marker_ranges', ['python3', '/ws/install/l2s2_localisation/lib/l2s2_localisation/ranges'])
    assert matches_helper('marker_detector', ['python3', '/ws/install/l2s1_marker_detection/lib/l2s1_marker_detection/detector'])
