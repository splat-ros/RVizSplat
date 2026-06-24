#!/usr/bin/env python3
"""Demo publisher for whole-tile compact splat updates."""

from __future__ import annotations

import os
import uuid
from collections import defaultdict

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from gsplat_msgs.msg import SplatTileChunk
from gsplat_publisher.chunking import build_tile_chunks
from gsplat_publisher.compact_encoder import COMPACT_DC_FP16_COV_RGBA_V1
from gsplat_publisher.compact_encoder import COMPACT_DC_FP16_COV_RGBA_V1_STRIDE
from gsplat_publisher.compact_encoder import encode_compact_dc_fp16_cov_rgba_v1
from gsplat_publisher.ply_reader import read_ply_vertex


def tile_keys_from_positions(vertices: np.ndarray, tile_size_m: float) -> np.ndarray:
    """Return int32 tile xyz keys from PLY vertex x/y/z positions."""
    if tile_size_m <= 0:
        raise ValueError('tile_size_m must be positive')
    positions = np.stack([
        vertices['x'].astype(np.float32),
        vertices['y'].astype(np.float32),
        vertices['z'].astype(np.float32),
    ], axis=1)
    return np.floor(positions / np.float32(tile_size_m)).astype(np.int32)


def compact_payloads_by_tile(vertices: np.ndarray, tile_size_m: float) -> dict[tuple[int, int, int], bytes]:
    """Encode vertices and group compact splat records by tile key."""
    keys = tile_keys_from_positions(vertices, tile_size_m)
    encoded = np.frombuffer(
        encode_compact_dc_fp16_cov_rgba_v1(vertices),
        dtype=np.uint8,
    ).reshape((len(vertices), COMPACT_DC_FP16_COV_RGBA_V1_STRIDE))

    indices_by_tile: dict[tuple[int, int, int], list[int]] = defaultdict(list)
    for index, key in enumerate(keys):
        indices_by_tile[(int(key[0]), int(key[1]), int(key[2]))].append(index)

    payloads = {}
    for tile, indices in indices_by_tile.items():
        payloads[tile] = encoded[np.asarray(indices, dtype=np.int64)].tobytes()
    return payloads


class TileDemoPublisher(Node):
    """Publish one whole-tile replacement pass followed by a COMMIT chunk."""

    def __init__(self):
        super().__init__('gsplat_tile_demo_publisher')

        self.declare_parameter('ply_path', '')
        self.declare_parameter('topic', 'gaussian_splat_tile_chunks')
        self.declare_parameter('session_id', '')
        self.declare_parameter('version', 1)
        self.declare_parameter('tile_size_m', 1.0)
        self.declare_parameter('max_chunk_bytes', 1024 * 1024)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('lod', 0)

        ply_path = self.get_parameter('ply_path').get_parameter_value().string_value
        topic = self.get_parameter('topic').get_parameter_value().string_value
        session_id = self.get_parameter('session_id').get_parameter_value().string_value
        version = self.get_parameter('version').get_parameter_value().integer_value
        tile_size_m = self.get_parameter('tile_size_m').get_parameter_value().double_value
        max_chunk_bytes = self.get_parameter('max_chunk_bytes').get_parameter_value().integer_value
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        lod = self.get_parameter('lod').get_parameter_value().integer_value

        if not ply_path or not os.path.isfile(ply_path):
            self.get_logger().fatal(f'PLY file not found: {ply_path}')
            raise FileNotFoundError(ply_path)
        if not session_id:
            session_id = str(uuid.uuid4())

        vertices = read_ply_vertex(ply_path)
        payloads_by_tile = compact_payloads_by_tile(vertices, tile_size_m)

        qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(SplatTileChunk, topic, qos)

        stamp = self.get_clock().now().to_msg()
        total_chunks = 0
        for tile in sorted(payloads_by_tile):
            payload = payloads_by_tile[tile]
            chunks = build_tile_chunks(
                payload,
                session_id=session_id,
                version=version,
                operation=SplatTileChunk.UPSERT_TILE,
                tile=tile,
                lod=lod,
                encoding=COMPACT_DC_FP16_COV_RGBA_V1,
                stride=COMPACT_DC_FP16_COV_RGBA_V1_STRIDE,
                max_chunk_bytes=max_chunk_bytes,
                splat_count=len(payload) // COMPACT_DC_FP16_COV_RGBA_V1_STRIDE,
                frame_id=frame_id,
                stamp=stamp,
            )
            for chunk in chunks:
                self._pub.publish(chunk)
            total_chunks += len(chunks)

        commit = build_tile_chunks(
            b'',
            session_id=session_id,
            version=version,
            operation=SplatTileChunk.COMMIT,
            tile=(0, 0, 0),
            lod=lod,
            encoding=COMPACT_DC_FP16_COV_RGBA_V1,
            stride=COMPACT_DC_FP16_COV_RGBA_V1_STRIDE,
            max_chunk_bytes=max_chunk_bytes,
            splat_count=0,
            frame_id=frame_id,
            stamp=stamp,
        )[0]
        self._pub.publish(commit)
        self.get_logger().info(
            f'Published {len(payloads_by_tile)} tile updates and COMMIT on /{topic} '
            f'({total_chunks + 1} chunks, tile size {tile_size_m:g} m, '
            f'session {session_id}, version {version})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TileDemoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
