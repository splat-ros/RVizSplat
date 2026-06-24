"""Timing tests that document encoding and chunking overhead.

These are informational — they print metrics and assert reasonable bounds,
but are not strict performance SLAs. Run without ROS dependency.
"""

from __future__ import annotations

import array
import time

from gsplat_publisher.chunking import build_blob_chunks
from gsplat_publisher.chunking import crc64_ecma


class FakeBlobChunk:
    def __init__(self):
        pass


def _build_blob_chunks(payload: bytes, enable_crc: bool) -> None:
    build_blob_chunks(
        payload,
        session_id='bench',
        version=1,
        format='compact_dc_fp16_cov_rgba_v1',
        max_chunk_bytes=1024 * 1024,
        msg_factory=FakeBlobChunk,
        enable_crc=enable_crc,
    )


def test_crc_overhead_documentation():
    """Document CRC cost for a 1 MiB blob (one chunk). Informational, not a strict bound."""
    payload = bytes(range(256)) * (1024 * 4)  # ~1 MiB

    t0 = time.perf_counter()
    for _ in range(10):
        _build_blob_chunks(payload, enable_crc=False)
    baseline_ms = (time.perf_counter() - t0) / 10 * 1000

    t0 = time.perf_counter()
    for _ in range(10):
        _build_blob_chunks(payload, enable_crc=True)
    with_crc_ms = (time.perf_counter() - t0) / 10 * 1000

    crc_only_ms = with_crc_ms - baseline_ms
    print(
        f'\nCRC cost (1 MiB chunk): baseline={baseline_ms:.1f}ms, '
        f'with_crc={with_crc_ms:.1f}ms, crc_cost={crc_only_ms:.1f}ms'
    )
    # Sanity: CRC must actually complete in finite time
    assert with_crc_ms < 10_000, 'CRC took >10s on 1 MiB — something is very wrong'


def test_array_vs_list_overhead():
    part = bytes(range(256)) * (1024 * 4)  # ~1 MiB

    t0 = time.perf_counter()
    for _ in range(20):
        _ = list(part)
    list_ms = (time.perf_counter() - t0) / 20 * 1000

    t0 = time.perf_counter()
    for _ in range(20):
        _ = array.array('B', part)
    array_ms = (time.perf_counter() - t0) / 20 * 1000

    speedup = list_ms / max(array_ms, 0.001)
    print(
        f'\nData field: list={list_ms:.2f}ms, array={array_ms:.2f}ms, '
        f'speedup={speedup:.1f}x'
    )
    # array.array should be at least as fast as list (may be slower on CPython due to
    # copy semantics, but should not be more than 3x slower)
    assert array_ms < list_ms * 3, (
        f'array.array is unexpectedly slow vs list: {array_ms:.2f}ms vs {list_ms:.2f}ms'
    )
