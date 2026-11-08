"""Check metric compressedDepth encodings, raw depth, and invalid payloads."""

import struct
from types import SimpleNamespace

import cv2
from cv_bridge import CvBridge
import numpy as np
from detection_improved.image_io import decode_depth
import pytest
from sensor_msgs.msg import Image


def encoded(pixels, encoding, a=0, b=0):
    ok, png = cv2.imencode('.png', pixels)
    assert ok
    return SimpleNamespace(format=encoding + '; compressedDepth png',
                           data=struct.pack('<iff', 0, a, b) + png.tobytes())


def test_16bit_compressed_depth_is_mm_and_zero_is_unknown():
    message = encoded(np.array([[1500, 0, 2300]], np.uint16), '16UC1')
    depth = decode_depth(message, None, True)
    np.testing.assert_allclose(depth[0, [0, 2]], [1.5, 2.3])
    assert np.isnan(depth[0, 1])


def test_float_compressed_depth_uses_inverse_quantization_header():
    message = encoded(np.array([[1000, 500, 0]], np.uint16), '32FC1', 1000., 0.)
    depth = decode_depth(message, None, True)
    np.testing.assert_allclose(depth[0, :2], [1., 2.])
    assert np.isnan(depth[0, 2])


@pytest.mark.parametrize('image_format,data', [
    ('bgr8; jpeg compressed bgr8', b'bad'), ('16UC1; compressedDepth rvl', bytes(30)),
])
def test_rgb_and_unsupported_depth_codec_are_rejected(image_format, data):
    with pytest.raises(ValueError, match='metric compressedDepth PNG'):
        decode_depth(SimpleNamespace(format=image_format, data=data), None, True)


@pytest.mark.parametrize('encoding,bigendian', [
    ('16UC1', False), ('16UC1', True), ('32FC1', False),
])
def test_raw_depth_honors_units_and_endianness(encoding, bigendian):
    message = Image()
    message.height, message.width = 1, 2
    message.encoding, message.is_bigendian = encoding, int(bigendian)
    dtype = ('>u2' if bigendian else '<u2') if encoding == '16UC1' else '<f4'
    pixels = np.array([[1500, 0]] if encoding == '16UC1' else [[1.5, np.nan]], dtype=dtype)
    message.step, message.data = pixels.nbytes, pixels.tobytes()
    depth = decode_depth(message, CvBridge())
    assert depth[0, 0] == pytest.approx(1.5)
    assert np.isnan(depth[0, 1])
