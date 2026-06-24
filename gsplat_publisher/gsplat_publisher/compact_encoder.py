"""Compact binary encoders for streamed Gaussian splats."""

from __future__ import annotations

import numpy as np


COMPACT_DC_FP16_COV_RGBA_V1 = 'compact_dc_fp16_cov_rgba_v1'
COMPACT_DC_FP16_COV_RGBA_V1_STRIDE = 32
SH_C0 = 0.28209479177387814

_REQUIRED_FIELDS = (
    'x', 'y', 'z',
    'f_dc_0', 'f_dc_1', 'f_dc_2',
    'opacity',
    'scale_0', 'scale_1', 'scale_2',
    'rot_0', 'rot_1', 'rot_2', 'rot_3',
)


def sigmoid(values: np.ndarray) -> np.ndarray:
    """Return sigmoid(values) with numpy broadcasting."""
    return 1.0 / (1.0 + np.exp(-values))


def validate_3dgs_vertex_fields(vertices: np.ndarray) -> None:
    """Raise ValueError if *vertices* is missing fields needed by the encoder."""
    names = set(vertices.dtype.names or ())
    missing = [name for name in _REQUIRED_FIELDS if name not in names]
    if missing:
        raise ValueError('PLY vertex data missing required properties: ' + ', '.join(missing))


def covariance_upper_triangle_from_3dgs(vertices: np.ndarray) -> np.ndarray:
    """Return covariance upper triangles as float32 columns xx, xy, xz, yy, yz, zz."""
    validate_3dgs_vertex_fields(vertices)
    n = len(vertices)

    quats = np.stack([
        vertices['rot_0'].astype(np.float32),
        vertices['rot_1'].astype(np.float32),
        vertices['rot_2'].astype(np.float32),
        vertices['rot_3'].astype(np.float32),
    ], axis=1)
    qnorms = np.linalg.norm(quats, axis=1, keepdims=True)
    quats = np.where(qnorms > 1e-6, quats / qnorms, quats)
    qw, qx, qy, qz = quats[:, 0], quats[:, 1], quats[:, 2], quats[:, 3]

    rot = np.empty((n, 3, 3), dtype=np.float32)
    rot[:, 0, 0] = 1 - 2 * (qy * qy + qz * qz)
    rot[:, 0, 1] = 2 * (qx * qy - qw * qz)
    rot[:, 0, 2] = 2 * (qx * qz + qw * qy)
    rot[:, 1, 0] = 2 * (qx * qy + qw * qz)
    rot[:, 1, 1] = 1 - 2 * (qx * qx + qz * qz)
    rot[:, 1, 2] = 2 * (qy * qz - qw * qx)
    rot[:, 2, 0] = 2 * (qx * qz - qw * qy)
    rot[:, 2, 1] = 2 * (qy * qz + qw * qx)
    rot[:, 2, 2] = 1 - 2 * (qx * qx + qy * qy)

    scales = np.stack([
        np.exp(vertices['scale_0'].astype(np.float32)),
        np.exp(vertices['scale_1'].astype(np.float32)),
        np.exp(vertices['scale_2'].astype(np.float32)),
    ], axis=1)
    scaled_rot = rot * (scales ** 2)[:, np.newaxis, :]
    cov = np.einsum('nrk,nck->nrc', scaled_rot, rot).astype(np.float32)

    return np.stack([
        cov[:, 0, 0], cov[:, 0, 1], cov[:, 0, 2],
        cov[:, 1, 1], cov[:, 1, 2], cov[:, 2, 2],
    ], axis=1)


def dc_rgba8_from_3dgs(vertices: np.ndarray) -> np.ndarray:
    """Return RGBA uint8 values from 3DGS DC SH color and raw opacity."""
    validate_3dgs_vertex_fields(vertices)
    dc = np.stack([
        vertices['f_dc_0'].astype(np.float32),
        vertices['f_dc_1'].astype(np.float32),
        vertices['f_dc_2'].astype(np.float32),
    ], axis=1)
    rgb = np.clip((dc * SH_C0 + 0.5) * 255.0, 0.0, 255.0).astype(np.uint8)
    alpha = np.clip(sigmoid(vertices['opacity'].astype(np.float32)) * 255.0, 0.0, 255.0)
    return np.concatenate([rgb, alpha.astype(np.uint8)[:, np.newaxis]], axis=1)


def encode_compact_dc_fp16_cov_rgba_v1(vertices: np.ndarray) -> bytes:
    """Encode 3DGS PLY vertex data as compact_dc_fp16_cov_rgba_v1.

    Per-splat layout, little-endian, 32 bytes:
      float32 x, y, z
      float16 cov_xx, cov_xy, cov_xz, cov_yy, cov_yz, cov_zz
      uint8 r, g, b, a
      uint32 id_or_flags
    """
    validate_3dgs_vertex_fields(vertices)
    n = len(vertices)
    dtype = np.dtype([
        ('position', '<f4', (3,)),
        ('covariance_upper', '<f2', (6,)),
        ('rgba', 'u1', (4,)),
        ('id_or_flags', '<u4'),
    ], align=False)
    if dtype.itemsize != COMPACT_DC_FP16_COV_RGBA_V1_STRIDE:
        raise AssertionError(f'unexpected compact stride: {dtype.itemsize}')

    encoded = np.zeros(n, dtype=dtype)
    encoded['position'][:, 0] = vertices['x'].astype(np.float32)
    encoded['position'][:, 1] = vertices['y'].astype(np.float32)
    encoded['position'][:, 2] = vertices['z'].astype(np.float32)
    encoded['covariance_upper'] = covariance_upper_triangle_from_3dgs(vertices).astype(np.float16)
    encoded['rgba'] = dc_rgba8_from_3dgs(vertices)
    return encoded.tobytes()
