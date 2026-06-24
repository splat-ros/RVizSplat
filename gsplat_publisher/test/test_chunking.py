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


def test_build_blob_chunks_sets_metadata_and_sizes():
    chunks = build_blob_chunks(
        b'abcdef',
        session_id='session',
        version=7,
        format='compact_dc_fp16_cov_rgba_v1',
        max_chunk_bytes=4,
        msg_factory=FakeBlobChunk,
        frame_id='map',
    )

    assert len(chunks) == 2
    assert [bytes(c.data) for c in chunks] == [b'abcd', b'ef']
    assert [c.chunk_index for c in chunks] == [0, 1]
    assert all(c.chunk_count == 2 for c in chunks)
    assert [c.chunk_size for c in chunks] == [4, 2]
    assert all(c.total_size == 6 for c in chunks)
    assert all(c.uncompressed_size == 6 for c in chunks)
    assert all(c.session_id == 'session' for c in chunks)
    assert all(c.version == 7 for c in chunks)
    assert all(c.format == 'compact_dc_fp16_cov_rgba_v1' for c in chunks)
    assert all(c.compression == 'none' for c in chunks)
    assert all(c.chunk_crc64 == crc64_ecma(bytes(c.data)) for c in chunks)
    assert all(c.replace_current for c in chunks)
    assert all(c.header.frame_id == 'map' for c in chunks)


def test_build_tile_chunks_sets_metadata_and_splat_count():
    chunks = build_tile_chunks(
        b'0123456789',
        session_id='session',
        version=8,
        operation=FakeTileChunk.UPSERT_TILE,
        tile=(-1, 2, 3),
        lod=1,
        encoding='compact_dc_fp16_cov_rgba_v1',
        stride=5,
        max_chunk_bytes=6,
        msg_factory=FakeTileChunk,
    )

    assert len(chunks) == 2
    assert [bytes(c.data) for c in chunks] == [b'012345', b'6789']
    assert [c.chunk_index for c in chunks] == [0, 1]
    assert all(c.chunk_count == 2 for c in chunks)
    assert all(c.session_id == 'session' for c in chunks)
    assert all(c.version == 8 for c in chunks)
    assert all(c.operation == FakeTileChunk.UPSERT_TILE for c in chunks)
    assert all((c.tile_x, c.tile_y, c.tile_z) == (-1, 2, 3) for c in chunks)
    assert all(c.lod == 1 for c in chunks)
    assert all(c.encoding == 'compact_dc_fp16_cov_rgba_v1' for c in chunks)
    assert all(c.compression == 'none' for c in chunks)
    assert all(c.splat_count == 2 for c in chunks)
    assert all(c.stride == 5 for c in chunks)
    assert all(c.uncompressed_size == 10 for c in chunks)
    assert all(c.chunk_crc64 == 0 for c in chunks)


def test_empty_tile_commit_is_one_chunk():
    chunks = build_tile_chunks(
        b'',
        session_id='session',
        version=9,
        operation=FakeTileChunk.COMMIT,
        tile=(0, 0, 0),
        lod=0,
        encoding='compact_dc_fp16_cov_rgba_v1',
        stride=32,
        max_chunk_bytes=4,
        splat_count=0,
        msg_factory=FakeTileChunk,
    )

    assert len(chunks) == 1
    assert chunks[0].operation == FakeTileChunk.COMMIT
    assert chunks[0].splat_count == 0
    assert bytes(chunks[0].data) == b''
