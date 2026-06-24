"""Integration tests for the PLY → compact encode → chunk pipeline.

Uses real PLY files from /home/bjoern/git/gs-icp-slam-ros2/.tmp_bench/.
All tests skip gracefully if the benchmark directory or specific files are absent.
"""

from __future__ import annotations

import glob
import os
from types import SimpleNamespace

import pytest

from gsplat_publisher.chunking import build_blob_chunks
from gsplat_publisher.chunking import crc64_ecma
from gsplat_publisher.chunking import estimate_chunk_count
from gsplat_publisher.compact_encoder import (
    COMPACT_DC_FP16_COV_RGBA_V1_STRIDE,
    encode_compact_dc_fp16_cov_rgba_v1,
    validate_3dgs_vertex_fields,
)
from gsplat_publisher.ply_reader import read_ply_vertex

_BENCH_DIR = '/home/bjoern/git/gs-icp-slam-ros2/.tmp_bench'
# Small 3DGS PLY from the splatograph regression suite (has all required fields)
_SMALL_PLY = '/home/bjoern/git/splatograph/tests/regression/golden/fixture.ply'
# Large 3DGS PLY for stress tests
_LARGE_PLY = '/home/bjoern/git/splatograph/output/floor3_orbbec_live/point_cloud/iteration_8000/point_cloud.ply'
_MAX_CHUNK = 1024 * 1024  # 1 MiB


class FakeBlobChunk:
    def __init__(self):
        self.header = SimpleNamespace(frame_id='', stamp=None)


@pytest.fixture
def bench_ply_dir():
    if not os.path.isdir(_BENCH_DIR):
        pytest.skip('benchmark PLY directory not found')
    return _BENCH_DIR


@pytest.fixture
def small_ply_path():
    if not os.path.isfile(_SMALL_PLY):
        pytest.skip(f'small 3DGS PLY not found: {_SMALL_PLY}')
    return _SMALL_PLY


@pytest.fixture
def large_ply_path():
    if not os.path.isfile(_LARGE_PLY):
        pytest.skip(f'large 3DGS PLY not found: {_LARGE_PLY}')
    return _LARGE_PLY


def test_read_and_encode_small_ply(small_ply_path):
    vertices = read_ply_vertex(small_ply_path)
    encoded = encode_compact_dc_fp16_cov_rgba_v1(vertices)
    assert isinstance(encoded, bytes)
    assert len(encoded) == len(vertices) * COMPACT_DC_FP16_COV_RGBA_V1_STRIDE


def test_compact_bytes_per_splat_is_32():
    assert COMPACT_DC_FP16_COV_RGBA_V1_STRIDE == 32


def test_size_reduction_vs_ply(small_ply_path):
    vertices = read_ply_vertex(small_ply_path)
    splat_count = len(vertices)
    ply_size = os.path.getsize(small_ply_path)
    ply_bytes_per_splat = ply_size / splat_count
    ratio = COMPACT_DC_FP16_COV_RGBA_V1_STRIDE / ply_bytes_per_splat
    # compact format must be less than 15% of PLY bytes/splat
    assert ratio < 0.15, f'compact/ply ratio={ratio:.3f}, expected <0.15'


def test_chunking_produces_correct_chunk_count(small_ply_path):
    vertices = read_ply_vertex(small_ply_path)
    payload = encode_compact_dc_fp16_cov_rgba_v1(vertices)
    chunks = build_blob_chunks(
        payload,
        session_id='test',
        version=1,
        format='compact_dc_fp16_cov_rgba_v1',
        max_chunk_bytes=_MAX_CHUNK,
        msg_factory=FakeBlobChunk,
    )
    expected = estimate_chunk_count(len(payload), _MAX_CHUNK)
    assert len(chunks) == expected


def test_all_chunks_have_valid_crc(small_ply_path):
    vertices = read_ply_vertex(small_ply_path)
    payload = encode_compact_dc_fp16_cov_rgba_v1(vertices)
    chunks = build_blob_chunks(
        payload,
        session_id='test',
        version=1,
        format='compact_dc_fp16_cov_rgba_v1',
        max_chunk_bytes=_MAX_CHUNK,
        msg_factory=FakeBlobChunk,
        enable_crc=True,
    )
    for chunk in chunks:
        assert chunk.chunk_crc64 != 0
        assert chunk.chunk_crc64 == crc64_ecma(bytes(chunk.data))


def _collect_bench_ply_files() -> list[str]:
    if not os.path.isdir(_BENCH_DIR):
        return []
    return sorted(glob.glob(os.path.join(_BENCH_DIR, '*.ply')))


@pytest.mark.parametrize('ply_path', _collect_bench_ply_files())
def test_all_bench_ply_files_encode_without_error(ply_path):
    try:
        vertices = read_ply_vertex(ply_path)
    except ValueError:
        pytest.skip(f'Unreadable PLY: {os.path.basename(ply_path)}')
    try:
        validate_3dgs_vertex_fields(vertices)
    except ValueError:
        pytest.skip(f'PLY missing required 3DGS fields: {os.path.basename(ply_path)}')
    encoded = encode_compact_dc_fp16_cov_rgba_v1(vertices)
    assert len(encoded) == len(vertices) * COMPACT_DC_FP16_COV_RGBA_V1_STRIDE
    assert len(encoded) % COMPACT_DC_FP16_COV_RGBA_V1_STRIDE == 0
