import numpy as np

from gsplat_publisher.compact_encoder import COMPACT_DC_FP16_COV_RGBA_V1_STRIDE
from gsplat_publisher.compact_encoder import SH_C0
from gsplat_publisher.compact_encoder import encode_compact_dc_fp16_cov_rgba_v1


def _vertices():
    dtype = [
        ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
        ('f_dc_0', '<f4'), ('f_dc_1', '<f4'), ('f_dc_2', '<f4'),
        ('opacity', '<f4'),
        ('scale_0', '<f4'), ('scale_1', '<f4'), ('scale_2', '<f4'),
        ('rot_0', '<f4'), ('rot_1', '<f4'), ('rot_2', '<f4'), ('rot_3', '<f4'),
    ]
    vertices = np.zeros(2, dtype=dtype)
    vertices['x'] = [1.0, -2.0]
    vertices['y'] = [2.0, -4.0]
    vertices['z'] = [3.0, -6.0]
    vertices['f_dc_0'] = [0.0, (1.0 - 0.5) / SH_C0]
    vertices['f_dc_1'] = [0.0, (0.0 - 0.5) / SH_C0]
    vertices['f_dc_2'] = [0.0, 0.0]
    vertices['opacity'] = [0.0, 10.0]
    vertices['scale_0'] = [0.0, np.log(2.0)]
    vertices['scale_1'] = [0.0, np.log(3.0)]
    vertices['scale_2'] = [0.0, np.log(4.0)]
    vertices['rot_0'] = [1.0, 1.0]
    return vertices


def test_compact_encoder_stride_and_layout():
    encoded = encode_compact_dc_fp16_cov_rgba_v1(_vertices())
    assert len(encoded) == 2 * COMPACT_DC_FP16_COV_RGBA_V1_STRIDE

    dtype = np.dtype([
        ('position', '<f4', (3,)),
        ('covariance_upper', '<f2', (6,)),
        ('rgba', 'u1', (4,)),
        ('id_or_flags', '<u4'),
    ])
    decoded = np.frombuffer(encoded, dtype=dtype)

    np.testing.assert_allclose(decoded['position'][0], [1.0, 2.0, 3.0])
    np.testing.assert_allclose(decoded['position'][1], [-2.0, -4.0, -6.0])
    np.testing.assert_allclose(decoded['covariance_upper'][0], [1, 0, 0, 1, 0, 1])
    np.testing.assert_allclose(decoded['covariance_upper'][1], [4, 0, 0, 9, 0, 16])
    assert decoded['rgba'][0].tolist() == [127, 127, 127, 127]
    assert decoded['rgba'][1].tolist() == [255, 0, 127, 254]
    assert decoded['id_or_flags'].tolist() == [0, 0]
