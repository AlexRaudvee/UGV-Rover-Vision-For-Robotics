"""Metric projection and acquisition-time transforms for the 3D viewer."""

from types import SimpleNamespace
from io import BytesIO
import struct

from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from PIL import Image as PillowImage
from sensor_msgs_py.point_cloud2 import read_points_numpy

from ugv_vision.depth_cloud import DepthCloud, project_depth


P = [2., 0., 1., 0., 0., 2., 1., 0., 0., 0., 1., 0.]


def test_projection_filters_invalid_depth_and_uses_calibration():
    depth = np.array([[1., 0., float('nan')], [float('inf'), 2., 7.]], dtype=np.float32)
    points = project_depth(depth, P, 1, .15, 6.)
    np.testing.assert_allclose(points, [[-.5, -.5, 1.], [0., 0., 2.]])
    np.testing.assert_allclose(project_depth(np.ones((3, 3)), P, 2, .15, 6.),
                               [[-.5, -.5, 1.], [.5, -.5, 1.], [-.5, .5, 1.], [.5, .5, 1.]])


def test_cloud_uses_image_time_not_latest_robot_pose():
    rclpy.init()
    node = DepthCloud()
    output = []
    node.publisher = SimpleNamespace(publish=output.append)
    now = node.get_clock().now().nanoseconds
    # The real OAK stream arrives ~1.1 s late. Retain acquisition-time geometry.
    old = Time(nanoseconds=now-1_400_000_000).to_msg()
    new = Time(nanoseconds=now-100_000_000).to_msg()
    try:
        # Same camera at x=1 at acquisition, x=2 later. Rotation maps optical
        # forward/right/down into vehicle forward/left/up.
        for stamp, x in [(old, 1.), (new, 2.)]:
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'odom', 'camera_optical'
            tf.transform.translation.x, tf.transform.translation.z = x, .2
            tf.transform.rotation.x, tf.transform.rotation.y = -.5, .5
            tf.transform.rotation.z, tf.transform.rotation.w = -.5, .5
            node.buffer.set_transform(tf, 'test')
        info = CameraInfo(width=3, height=3, p=P)
        info.header.frame_id = 'camera_optical'
        node.on_info(info)
        for dtype, encoding, value in [(np.uint16, '16UC1', 1000), (np.float32, '32FC1', 1.)]:
            image = CvBridge().cv2_to_imgmsg(np.full((3,3), value, dtype=dtype), encoding=encoding)
            image.header.stamp, image.header.frame_id = old, 'camera_optical'
            node.on_image(image)
            node.publish()
            assert output[-1].header.frame_id == 'odom'
            assert output[-1].header.stamp == old
            points = read_points_numpy(output[-1], field_names=('x','y','z'))
            np.testing.assert_allclose(points[4], [2., 0., .2], atol=1e-6)
            count = len(output)
            node.publish()
            assert len(output) == count, 'An old image was republished as new data'
        png = BytesIO()
        PillowImage.fromarray(np.full((3, 3), 1000, np.uint16)).save(png, format='PNG')
        compressed = CompressedImage(format='16UC1; compressedDepth png',
                                     data=struct.pack('<iff', 0, 0., 0.) + png.getvalue())
        compressed.header = image.header
        node.on_image(compressed)
        node.publish()
        np.testing.assert_allclose(read_points_numpy(output[-1])[4], [2., 0., .2], atol=1e-6)
        assert len(output) == count + 1
        count = len(output)
        image.header.frame_id = 'unknown_camera'
        info.header.frame_id = 'unknown_camera'
        node.on_image(image)
        node.publish()
        assert len(output) == count, 'A cloud without sensor TF was published'
        image.header.stamp = Time(nanoseconds=now-4_000_000_000).to_msg()
        node.publish()
        assert len(output) == count, 'A stale cloud was published'
    finally:
        node.destroy_node()
        rclpy.shutdown()
