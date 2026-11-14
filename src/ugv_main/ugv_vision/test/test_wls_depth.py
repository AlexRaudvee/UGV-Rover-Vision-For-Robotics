"""Verify metric scale, invalid pixels, and smoothing of registered depth."""

import numpy as np

from ugv_vision.wls_depth import smooth_depth


def test_constant_depth_keeps_metric_scale_and_missing_pixels():
    depth = np.full((64, 80), 2.5, np.float32)
    depth[20:25, 20:25] = np.nan
    guide = np.full((64, 80, 3), 100, np.uint8)
    result = smooth_depth(depth, guide)
    np.testing.assert_allclose(result[np.isfinite(depth)], 2.5, atol=1e-4)
    assert not result[20:25, 20:25].any()
    filled = smooth_depth(depth, guide, fill_holes=True)
    np.testing.assert_allclose(filled, 2.5, atol=1e-4)


def test_missing_scene_stays_missing_and_smoothing_reduces_surface_noise():
    guide = np.full((64, 80, 3), 100, np.uint8)
    assert not smooth_depth(np.zeros((64, 80), np.float32), guide).any()
    depth = np.random.default_rng(7).normal(2., .1, (64, 80)).astype(np.float32)
    result = smooth_depth(depth, guide)
    assert np.std(result) < np.std(depth) / 2
    assert abs(np.mean(result) - 2.) < .03


def test_subsampled_calibration_keeps_pixel_rays_and_source_unchanged():
    from sensor_msgs.msg import CameraInfo
    from std_msgs.msg import Header
    from ugv_vision.wls_depth import filtered_camera_info
    info = CameraInfo(width=1280, height=720,
                      k=[1000., 0., 640., 0., 1000., 360., 0., 0., 1.],
                      p=[1000., 0., 640., 0., 0., 1000., 360., 0., 0., 0., 1., 0.])
    scaled = filtered_camera_info(info, Header(frame_id='camera'), (360, 640), 2)
    assert (scaled.width, scaled.height) == (640, 360)
    np.testing.assert_allclose(scaled.p[:8], [500., 0., 320., 0., 0., 500., 180., 0.])
    assert info.p[0] == 1000. and info.width == 1280


def test_wls_calibration_metric_preview_and_cloud_share_depth(monkeypatch):
    from types import SimpleNamespace
    import cv2
    from cv_bridge import CvBridge
    from geometry_msgs.msg import TransformStamped
    import rclpy
    from sensor_msgs.msg import CameraInfo, CompressedImage
    from sensor_msgs_py.point_cloud2 import read_points_numpy
    from ugv_vision.depth_cloud import DepthCloud
    from ugv_vision.depth_preview import depth_in_meters
    from ugv_vision.wls_depth import WLSDepth
    monkeypatch.setenv('ROS_DOMAIN_ID', '88')
    rclpy.init(args=['--ros-args', '-p', 'max_width:=160'])
    wls, cloud = WLSDepth(), DepthCloud()
    messages = {key: [] for key in ('raw', 'metric', 'info', 'preview', 'cloud')}
    for key in ('raw', 'metric', 'info', 'preview'):
        setattr(wls, key + '_pub', SimpleNamespace(publish=messages[key].append))
    cloud.publisher = SimpleNamespace(publish=messages['cloud'].append)
    try:
        from rclpy.time import Time
        from copy import deepcopy
        stamp = Time(nanoseconds=wls.get_clock().now().nanoseconds - 200_000_000).to_msg()
        raw = CvBridge().cv2_to_imgmsg(np.full((240, 320), 2500, np.uint16), encoding='16UC1')
        raw.header.frame_id, raw.header.stamp = 'camera', stamp
        rgb = CompressedImage(format='jpeg', data=cv2.imencode('.jpg', np.full((240,320,3), 100, np.uint8))[1].tobytes())
        rgb.header = raw.header
        info = CameraInfo(width=320, height=240,
                          k=[200.,0.,160.,0.,200.,120.,0.,0.,1.],
                          p=[200.,0.,160.,0.,0.,200.,120.,0.,0.,0.,1.,0.])
        info.header = raw.header
        wls.receive_depth(raw)
        wls.receive_rgb(rgb)
        wls.receive_info(info)
        wls.tick()
        assert all(len(messages[k]) == 1 for k in ('raw', 'metric', 'info', 'preview'))
        filtered = depth_in_meters(messages['metric'][0])
        assert filtered.shape == (120, 160)
        np.testing.assert_allclose(filtered, 2.5, atol=.001)
        assert messages['info'][0].p[0] == 100.
        assert messages['metric'][0].header == messages['preview'][0].header
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'odom', 'camera'
        tf.transform.rotation.w = 1.
        cloud.buffer.set_transform_static(tf, 'test')
        cloud.on_info(messages['info'][0])
        cloud.on_image(messages['metric'][0])
        cloud.publish()
        assert len(messages['cloud']) == 1
        np.testing.assert_allclose(read_points_numpy(messages['cloud'][0])[:,2], 2.5, atol=.001)
        # Every metric output (and therefore the external preview) sees EMA depth.
        changed = CvBridge().cv2_to_imgmsg(np.full((240, 320), 2900, np.uint16), encoding='16UC1')
        changed.header = deepcopy(raw.header)
        changed.header.stamp = Time(nanoseconds=Time.from_msg(stamp).nanoseconds + 100_000_000).to_msg()
        rgb.header = changed.header
        wls.receive_depth(changed)
        wls.receive_rgb(rgb)
        wls.tick()
        expected = 2.5 + (1. - np.exp(-.1 / .35)) * .4
        raw_depth = depth_in_meters(messages['raw'][-1])
        metric_depth = depth_in_meters(messages['metric'][-1])
        np.testing.assert_allclose(raw_depth, expected, atol=.0006)
        np.testing.assert_array_equal(raw_depth, metric_depth)
        assert messages['metric'][-1].header == messages['preview'][-1].header
        cloud.on_info(messages['info'][-1])
        cloud.on_image(messages['metric'][-1])
        cloud.publish()
        np.testing.assert_allclose(read_points_numpy(messages['cloud'][-1])[:,2], expected, atol=.0006)
    finally:
        cloud.destroy_node()
        wls.destroy_node()
        rclpy.shutdown()


def test_depth_ema_response_is_independent_of_frame_rate():
    from ugv_vision.wls_depth import DepthEMA
    for dt in (.05, .1, .2):
        ema = DepthEMA(.4, 0., .8)
        ema.update(np.array([[2.]], np.float32), 0., 'camera')
        for i in range(1, round(1. / dt) + 1):
            result = ema.update(np.array([[3.]], np.float32), i * dt, 'camera')
        np.testing.assert_allclose(result, 3. - np.exp(-1. / .4), atol=1e-6)


def test_depth_ema_invalid_pixels_edges_and_history_resets():
    from ugv_vision.wls_depth import DepthEMA
    ema = DepthEMA(.35, .5, .8)
    ema.update(np.array([[2., 2., 0.]], np.float32), 1., 'camera')
    result = ema.update(np.array([[2.4, np.nan, 3.]], np.float32), 1.1, 'camera')
    assert 2. < result[0, 0] < 2.4
    assert result[0, 1] == 0. and result[0, 2] == 3.
    result = ema.update(np.array([[1., 2.3, 3.]], np.float32), 1.2, 'camera')
    assert result[0, 0] == 1. and result[0, 1] == np.float32(2.3)
    for stamp, key in [(3., 'camera'), (3.1, 'new-calibration'), (2., 'new-calibration')]:
        expected = np.full((1, 3), 2.4, np.float32)
        np.testing.assert_array_equal(ema.update(expected, stamp, key), expected)
    ema.reset()
    np.testing.assert_array_equal(ema.update(np.ones((2, 2)), 4., 'camera'), np.ones((2, 2)))
    disabled = DepthEMA(0., .5, .8)
    disabled.update(np.array([[2.]]), 1., 'camera')
    np.testing.assert_array_equal(disabled.update(np.array([[2.4]]), 1.1, 'camera'),
                                  np.array([[2.4]], np.float32))
