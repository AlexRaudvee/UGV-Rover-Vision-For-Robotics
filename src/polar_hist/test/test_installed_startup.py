"""Exercise installed entry-point wrappers, including metadata resolution."""

import os
from pathlib import Path
import signal
import subprocess

from ament_index_python.packages import get_package_prefix
import pytest


@pytest.mark.parametrize('package,executable,ready', [
    ('object_detection', 'detector', 'object_detection:'),
    ('obstacle_detection', 'detector', 'obstacle_detection:'),
    ('polar_hist', 'histogram', 'Measured histograms ready'),
])
def test_installed_executable_reaches_node_startup(package, executable, ready):
    path = Path(get_package_prefix(package)) / 'lib' / package / executable
    environment = dict(os.environ, ROS_DOMAIN_ID='178',
                       ROS_LOG_DIR='/tmp/ugv-installed-perception-test')
    process = subprocess.Popen([str(path)], env=environment,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        try:
            output, _ = process.communicate(timeout=2.)
            pytest.fail(f'{package} exited before startup: {process.returncode}\n{output}')
        except subprocess.TimeoutExpired:
            process.send_signal(signal.SIGINT)
            output, _ = process.communicate(timeout=5.)
        assert ready in output, output
        assert 'Traceback' not in output, output
        assert process.returncode == 0, output
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
