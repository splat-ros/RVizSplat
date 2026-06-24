#!/usr/bin/env python3
"""Publish a PLY file as SplatBlobChunk snapshot chunks."""

from __future__ import annotations

import os
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from gsplat_msgs.msg import SplatBlobChunk
from gsplat_publisher.chunking import build_blob_chunks
from gsplat_publisher.compact_encoder import COMPACT_DC_FP16_COV_RGBA_V1
from gsplat_publisher.compact_encoder import encode_compact_dc_fp16_cov_rgba_v1
from gsplat_publisher.ply_reader import read_ply_vertex


class BlobSnapshotPublisher(Node):
    """Publish one latched blob snapshot from a PLY path."""

    def __init__(self):
        super().__init__('gsplat_blob_snapshot_publisher')

        self.declare_parameter('ply_path', '')
        self.declare_parameter('mode', 'compact')
        self.declare_parameter('topic', 'gaussian_splat_blob_chunks')
        self.declare_parameter('session_id', '')
        self.declare_parameter('version', 1)
        self.declare_parameter('max_chunk_bytes', 1024 * 1024)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('replace_current', True)

        ply_path = self.get_parameter('ply_path').get_parameter_value().string_value
        mode = self.get_parameter('mode').get_parameter_value().string_value
        topic = self.get_parameter('topic').get_parameter_value().string_value
        session_id = self.get_parameter('session_id').get_parameter_value().string_value
        version = self.get_parameter('version').get_parameter_value().integer_value
        max_chunk_bytes = self.get_parameter('max_chunk_bytes').get_parameter_value().integer_value
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        replace_current = self.get_parameter('replace_current').get_parameter_value().bool_value

        if not ply_path or not os.path.isfile(ply_path):
            self.get_logger().fatal(f'PLY file not found: {ply_path}')
            raise FileNotFoundError(ply_path)
        if not session_id:
            session_id = str(uuid.uuid4())

        payload, blob_format = self._load_payload(ply_path, mode)
        qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(SplatBlobChunk, topic, qos)

        stamp = self.get_clock().now().to_msg()
        chunks = build_blob_chunks(
            payload,
            session_id=session_id,
            version=version,
            format=blob_format,
            max_chunk_bytes=max_chunk_bytes,
            replace_current=replace_current,
            frame_id=frame_id,
            stamp=stamp,
        )
        for chunk in chunks:
            self._pub.publish(chunk)
        self.get_logger().info(
            f'Published {len(chunks)} {blob_format} chunks on /{topic} '
            f'({len(payload)} bytes, session {session_id}, version {version})'
        )

    @staticmethod
    def _load_payload(ply_path: str, mode: str) -> tuple[bytes, str]:
        if mode == 'binary_ply':
            with open(ply_path, 'rb') as ply_file:
                return ply_file.read(), 'ply_binary'
        if mode == 'compact':
            vertices = read_ply_vertex(ply_path)
            return encode_compact_dc_fp16_cov_rgba_v1(vertices), COMPACT_DC_FP16_COV_RGBA_V1
        raise ValueError("mode must be 'compact' or 'binary_ply'")


def main(args=None):
    rclpy.init(args=args)
    node = BlobSnapshotPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
