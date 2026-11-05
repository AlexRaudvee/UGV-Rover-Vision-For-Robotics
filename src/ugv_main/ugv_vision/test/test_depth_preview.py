"""Check metric decoding and the meaning of the depth visualization colors."""

from types import SimpleNamespace

import numpy as np
import pytest

from ugv_vision.depth_preview import colorize_depth, depth_in_meters


@pytest.mark.parametrize('big_endian', [False, True])
def test_millimeters_with_padded_rows(big_endian):
    dtype = '>u2' if big_endian else '<u2'
    rows = [np.array(row, dtype=dtype).tobytes() + b'\x00\x00'
            for row in [[1000, 0], [2500, 5000]]]
    message = SimpleNamespace(encoding='16UC1', is_bigendian=big_endian,
                              width=2, height=2, step=6, data=b''.join(rows))
    original = message.data
    np.testing.assert_allclose(depth_in_meters(message), [[1, 0], [2.5, 5]])
    assert message.data == original


def test_float_depth_keeps_meters_and_invalid_measurements():
    values = np.array([[1.25, np.nan, np.inf, 0, -1]], dtype='>f4')
    message = SimpleNamespace(encoding='32FC1', is_bigendian=True,
                              width=5, height=1, step=20, data=values.tobytes())
    depth = depth_in_meters(message)
    assert depth[0, 0] == 1.25
    colors, valid = colorize_depth(depth, 0.2, 5.0)
    assert valid.tolist() == [[True, False, False, False, False]]
    assert not colors[0, 1:].any()


def test_near_is_red_and_far_is_blue():
    colors, _ = colorize_depth(np.array([[0.2, 5.0]], dtype=np.float32), 0.2, 5.0)
    assert colors[0, 0, 0] > colors[0, 0, 2]
    assert colors[0, 1, 2] > colors[0, 1, 0]


def test_preview_width_is_bounded_for_odd_image_sizes():
    values = np.full((2, 1281), 1500, dtype='<u2')
    message = SimpleNamespace(encoding='16UC1', is_bigendian=False,
                              width=1281, height=2, step=2562, data=values.tobytes())
    preview = depth_in_meters(message, max_width=640)
    assert preview.shape[1] <= 640
    np.testing.assert_allclose(preview, 1.5)


def test_color_image_cannot_be_interpreted_as_depth():
    with pytest.raises(ValueError, match='Unsupported depth encoding'):
        depth_in_meters(SimpleNamespace(encoding='rgb8'))
