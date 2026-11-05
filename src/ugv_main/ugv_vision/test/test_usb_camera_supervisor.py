"""Regression checks for USB device re-enumeration."""

from ugv_vision.usb_camera_supervisor import camera_command


def test_restart_resolves_the_current_device(tmp_path):
    first = tmp_path / 'video0'
    second = tmp_path / 'video2'
    first.touch()
    second.touch()
    stable = tmp_path / 'by-id' / 'camera'
    stable.parent.mkdir()
    stable.symlink_to('../video0')
    args = ['--ros-args', '-r', '__node:=usb_cam']
    assert camera_command('driver', args, str(stable))[-1] == 'video_device:=' + str(first)
    stable.unlink()
    stable.symlink_to('../video2')
    assert camera_command('driver', args, str(stable))[-1] == 'video_device:=' + str(second)
    assert args == ['--ros-args', '-r', '__node:=usb_cam']


def test_manual_invocation_adds_ros_argument_marker(tmp_path):
    command = camera_command('driver', [], str(tmp_path / 'video0'))
    assert command[1:3] == ['--ros-args', '-p']
