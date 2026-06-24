"""Helpers for splitting splat payloads into ROS chunk messages."""

from __future__ import annotations

import array as _array
import math
from collections.abc import Callable, Sequence
from typing import Any

_CRC64_POLY = 0x42F0E1EBA9EA3693


def _build_crc64_table() -> list[int]:
    table = []
    for b in range(256):
        crc = b << 56
        for _ in range(8):
            if crc & 0x8000000000000000:
                crc = ((crc << 1) & 0xFFFFFFFFFFFFFFFF) ^ _CRC64_POLY
            else:
                crc = (crc << 1) & 0xFFFFFFFFFFFFFFFF
        table.append(crc)
    return table


_CRC64_TABLE = _build_crc64_table()


def _crc64_ecma_pure(data: bytes | bytearray) -> int:
    crc = 0
    for byte in data:
        idx = ((crc >> 56) ^ byte) & 0xFF
        crc = ((crc << 8) & 0xFFFFFFFFFFFFFFFF) ^ _CRC64_TABLE[idx]
    return crc


try:
    import crcmod as _crcmod
    # CRC-64/ECMA-182: poly=0x42F0E1EBA9EA3693, init=0, rev=False, xorOut=0
    # crcmod expects the poly with the implicit leading 1 bit included
    _crc64_fn = _crcmod.mkCrcFun(0x142F0E1EBA9EA3693, initCrc=0, rev=False, xorOut=0)

    def crc64_ecma(data: bytes | bytearray) -> int:
        """CRC-64/ECMA-182. Matches blob_assembler.cpp crc64Ecma()."""
        return _crc64_fn(data)

except Exception:
    def crc64_ecma(data: bytes | bytearray) -> int:  # type: ignore[misc]
        """CRC-64/ECMA-182. Matches blob_assembler.cpp crc64Ecma()."""
        return _crc64_ecma_pure(data)


def split_bytes(payload: bytes, max_chunk_bytes: int) -> list[bytes]:
    """Split *payload* into non-empty chunks no larger than *max_chunk_bytes*."""
    if max_chunk_bytes <= 0:
        raise ValueError('max_chunk_bytes must be positive')
    if not payload:
        return [b'']
    return [
        payload[start:start + max_chunk_bytes]
        for start in range(0, len(payload), max_chunk_bytes)
    ]


def _set_header(msg: Any, frame_id: str | None, stamp: Any | None) -> None:
    if not hasattr(msg, 'header'):
        return
    if frame_id is not None:
        msg.header.frame_id = frame_id
    if stamp is not None:
        msg.header.stamp = stamp


def build_blob_chunks(
    payload: bytes,
    *,
    session_id: str,
    version: int,
    format: str,
    max_chunk_bytes: int,
    msg_factory: Callable[[], Any] | None = None,
    compression: str = 'none',
    uncompressed_size: int | None = None,
    replace_current: bool = True,
    frame_id: str | None = None,
    stamp: Any | None = None,
    chunk_crc64: int = 0,
    enable_crc: bool = True,
    map_from_source: Any | None = None,
) -> list[Any]:
    """Return SplatBlobChunk messages for *payload*."""
    if msg_factory is None:
        from gsplat_msgs.msg import SplatBlobChunk
        msg_factory = SplatBlobChunk

    parts = split_bytes(payload, max_chunk_bytes)
    total_size = len(payload)
    raw_size = total_size if uncompressed_size is None else uncompressed_size
    messages = []
    for index, part in enumerate(parts):
        msg = msg_factory()
        _set_header(msg, frame_id, stamp)
        msg.session_id = session_id
        msg.version = int(version)
        msg.chunk_index = index
        msg.chunk_count = len(parts)
        msg.chunk_size = len(part)
        msg.total_size = total_size
        msg.uncompressed_size = raw_size
        msg.format = format
        msg.compression = compression
        msg.data = _array.array('B', part)
        msg.chunk_crc64 = crc64_ecma(part) if enable_crc else chunk_crc64
        msg.replace_current = replace_current
        if map_from_source is not None:
            msg.map_from_source = map_from_source
        messages.append(msg)
    return messages


def build_tile_chunks(
    payload: bytes,
    *,
    session_id: str,
    version: int,
    operation: int,
    tile: tuple[int, int, int],
    lod: int,
    encoding: str,
    stride: int,
    max_chunk_bytes: int,
    msg_factory: Callable[[], Any] | None = None,
    compression: str = 'none',
    splat_count: int | None = None,
    uncompressed_size: int | None = None,
    frame_id: str | None = None,
    stamp: Any | None = None,
    chunk_crc64: int = 0,
    enable_crc: bool = False,
) -> list[Any]:
    """Return SplatTileChunk messages for one tile payload."""
    if msg_factory is None:
        from gsplat_msgs.msg import SplatTileChunk
        msg_factory = SplatTileChunk

    if splat_count is None:
        if stride <= 0:
            raise ValueError('stride must be positive when splat_count is omitted')
        splat_count = len(payload) // stride
    raw_size = len(payload) if uncompressed_size is None else uncompressed_size
    parts = split_bytes(payload, max_chunk_bytes)
    messages = []
    for index, part in enumerate(parts):
        msg = msg_factory()
        _set_header(msg, frame_id, stamp)
        msg.session_id = session_id
        msg.version = int(version)
        msg.operation = int(operation)
        msg.tile_x = int(tile[0])
        msg.tile_y = int(tile[1])
        msg.tile_z = int(tile[2])
        msg.lod = int(lod)
        msg.chunk_index = index
        msg.chunk_count = len(parts)
        msg.encoding = encoding
        msg.compression = compression
        msg.splat_count = int(splat_count)
        msg.stride = int(stride)
        msg.uncompressed_size = int(raw_size)
        msg.data = _array.array('B', part)
        msg.chunk_crc64 = crc64_ecma(part) if enable_crc else chunk_crc64
        messages.append(msg)
    return messages


def flatten_chunks(chunks_by_tile: Sequence[Sequence[Any]]) -> list[Any]:
    """Return a flat list from tile chunk groups."""
    return [chunk for group in chunks_by_tile for chunk in group]


def estimate_chunk_count(payload_size: int, max_chunk_bytes: int) -> int:
    """Return the chunk count used by split_bytes without materializing chunks."""
    if max_chunk_bytes <= 0:
        raise ValueError('max_chunk_bytes must be positive')
    return max(1, int(math.ceil(payload_size / max_chunk_bytes)))
