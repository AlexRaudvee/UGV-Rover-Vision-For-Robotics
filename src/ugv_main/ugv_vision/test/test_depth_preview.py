"""Check metric decoding and the meaning of the depth visualization colors."""

from types import SimpleNamespace
from io import BytesIO
import struct

import numpy as np
import pytest
from PIL import Image

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


@pytest.mark.parametrize('encoding', ['16UC1', '32FC1'])
@pytest.mark.parametrize('codec_suffix', ['', ' png'])
def test_compressed_depth_decodes_metric_values_and_missing_pixels(encoding, codec_suffix):
    pixels = np.array([[1000, 0, 2000]], dtype=np.uint16)
    buffer = BytesIO()
    Image.fromarray(pixels).save(buffer, format='PNG')
    message = SimpleNamespace(format=encoding + '; compressedDepth' + codec_suffix,
                              data=struct.pack('<iff', 0, 1000., 0.) + buffer.getvalue())
    original = message.data
    depth = depth_in_meters(message)
    np.testing.assert_allclose(depth[0, [0, 2]], [1., 2.] if encoding == '16UC1' else [1., .5])
    _, valid = colorize_depth(depth, .2, 5.)
    assert valid.tolist() == [[True, False, True]]
    assert message.data == original


def test_compressed_depth_rejects_other_codecs_and_missing_header():
    for fmt, data in [('16UC1; compressedDepth rvl', b'x' * 30),
                      ('16UC1; compressedDepth png', b'\x89PNG\r\n\x1a\n')]:
        with pytest.raises(ValueError, match='compressedDepth PNG'):
            depth_in_meters(SimpleNamespace(format=fmt, data=data))
