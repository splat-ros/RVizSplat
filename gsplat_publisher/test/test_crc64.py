"""Tests for CRC-64/ECMA-182 implementation in chunking.py.

Validates against the published check value so we can trust CRC integrity
before it is used in blob chunk assembly.
"""

from types import SimpleNamespace

from gsplat_publisher.chunking import build_blob_chunks
from gsplat_publisher.chunking import build_tile_chunks
from gsplat_publisher.chunking import crc64_ecma


class FakeBlobChunk:
    def __init__(self):
        self.header = SimpleNamespace(frame_id='', stamp=None)


class FakeTileChunk:
    UPSERT_TILE = 1
    COMMIT = 3

    def __init__(self):
        self.header = SimpleNamespace(frame_id='', stamp=None)


def test_known_check_vector():
    # CRC-64/ECMA-182 standard check value for b"123456789"
    assert crc64_ecma(b'123456789') == 0x6C40DF5F0B497347


def test_empty_input():
    assert crc64_ecma(b'') == 0


def test_blob_chunk_crc_per_chunk():
    chunks = build_blob_chunks(
        b'abcdefghij',
        session_id='s',
        version=1,
        format='compact_dc_fp16_cov_rgba_v1',
        max_chunk_bytes=6,
        msg_factory=FakeBlobChunk,
        enable_crc=True,
    )
    assert len(chunks) == 2
    for chunk in chunks:
        assert chunk.chunk_crc64 != 0
        assert chunk.chunk_crc64 == crc64_ecma(bytes(chunk.data))


def test_tile_chunk_crc_off_by_default():
    chunks = build_tile_chunks(
        b'0123456789',
        session_id='s',
        version=1,
        operation=FakeTileChunk.UPSERT_TILE,
        tile=(0, 0, 0),
        lod=0,
        encoding='compact_dc_fp16_cov_rgba_v1',
        stride=5,
        max_chunk_bytes=100,
        msg_factory=FakeTileChunk,
    )
    assert all(c.chunk_crc64 == 0 for c in chunks)


def test_tile_chunk_crc_opt_in():
    chunks = build_tile_chunks(
        b'0123456789',
        session_id='s',
        version=1,
        operation=FakeTileChunk.UPSERT_TILE,
        tile=(0, 0, 0),
        lod=0,
        encoding='compact_dc_fp16_cov_rgba_v1',
        stride=5,
        max_chunk_bytes=100,
        msg_factory=FakeTileChunk,
        enable_crc=True,
    )
    for chunk in chunks:
        assert chunk.chunk_crc64 != 0
        assert chunk.chunk_crc64 == crc64_ecma(bytes(chunk.data))
