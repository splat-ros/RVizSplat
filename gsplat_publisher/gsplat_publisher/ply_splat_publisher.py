#!/usr/bin/env python3
"""Publishes a latched SplatArray message loaded from a PLY file."""

import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from gsplat_msgs.msg import Splat, SplatArray

from gsplat_publisher.ply_reader import read_ply_vertex


def _sigmoid_vec(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


def _compute_covariance(scales: np.ndarray, quats: np.ndarray) -> np.ndarray:
    """Return (n, 6, 6) covariance matrices with 3D Gaussian cov in top-left 3x3.

    scales : (n, 3)  actual scale (not log-space)
    quats  : (n, 4)  normalized quaternion [w, x, y, z]
    """
    n = scales.shape[0]
    qw, qx, qy, qz = quats[:, 0], quats[:, 1], quats[:, 2], quats[:, 3]

    R = np.zeros((n, 3, 3))
    R[:, 0, 0] = 1 - 2 * (qy * qy + qz * qz)
    R[:, 0, 1] = 2 * (qx * qy - qw * qz)
    R[:, 0, 2] = 2 * (qx * qz + qw * qy)
    R[:, 1, 0] = 2 * (qx * qy + qw * qz)
    R[:, 1, 1] = 1 - 2 * (qx * qx + qz * qz)
    R[:, 1, 2] = 2 * (qy * qz - qw * qx)
    R[:, 2, 0] = 2 * (qx * qz - qw * qy)
    R[:, 2, 1] = 2 * (qy * qz + qw * qx)
    R[:, 2, 2] = 1 - 2 * (qx * qx + qy * qy)

    s2 = scales ** 2  # (n, 3)
    # Σ[r,c] = Σ_k R[r,k] * s2[k] * R[c,k]
    A = R * s2[:, np.newaxis, :]   # (n, 3, 3)
    cov3 = np.einsum('nrk,nck->nrc', A, R)  # (n, 3, 3)

    cov36 = np.zeros((n, 36))
    for r in range(3):
        for c in range(3):
            cov36[:, r * 6 + c] = cov3[:, r, c]
    return cov36


def _load_splat_array(ply_path: str, sh_degree: int, stamp, frame_id: str) -> SplatArray:
    verts = read_ply_vertex(ply_path)
    n = len(verts)
    prop_names = set(verts.dtype.names)

    n_rest = sum(1 for p in prop_names if p.startswith('f_rest_'))
    # nrc: rest SH coefficients per channel (channel-major layout in PLY)
    nrc = n_rest // 3
    file_degree = 3 if nrc >= 15 else (2 if nrc >= 8 else (1 if nrc >= 3 else 0))
    actual_degree = min(sh_degree, file_degree)
    # Number of rest coefficients per channel to include for actual_degree
    nrc_to_use = (actual_degree + 1) ** 2 - 1

    # --- positions
    xs = verts['x'].astype(np.float64)
    ys = verts['y'].astype(np.float64)
    zs = verts['z'].astype(np.float64)

    # --- quaternions [w, x, y, z] (PLY convention: rot_0=w)
    quats = np.stack([
        verts['rot_0'].astype(np.float64),
        verts['rot_1'].astype(np.float64),
        verts['rot_2'].astype(np.float64),
        verts['rot_3'].astype(np.float64),
    ], axis=1)
    qnorms = np.linalg.norm(quats, axis=1, keepdims=True)
    quats = np.where(qnorms > 1e-6, quats / qnorms, quats)

    # --- scales (actual, not log)
    scales = np.stack([
        np.exp(verts['scale_0'].astype(np.float64)),
        np.exp(verts['scale_1'].astype(np.float64)),
        np.exp(verts['scale_2'].astype(np.float64)),
    ], axis=1)

    # --- covariance (n, 36)
    cov36 = _compute_covariance(scales, quats)

    # --- opacity as uint8
    raw_opacity = verts['opacity'].astype(np.float64)
    opacity_u8 = np.clip(_sigmoid_vec(raw_opacity) * 255.0, 0, 255).astype(np.uint8)

    # --- SH coefficients: interleaved RGB, coefficient-major
    # DC term (degree 0)
    sh_dc = np.stack([
        verts['f_dc_0'].astype(np.float32),
        verts['f_dc_1'].astype(np.float32),
        verts['f_dc_2'].astype(np.float32),
    ], axis=1)  # (n, 3)

    # Higher-order rest terms (PLY stores channel-major: R block, G block, B block)
    sh_parts = [sh_dc]
    for ci in range(nrc_to_use):
        r_key = f'f_rest_{ci}'
        g_key = f'f_rest_{ci + nrc}'
        b_key = f'f_rest_{ci + 2 * nrc}'
        r_col = verts[r_key].astype(np.float32) if r_key in prop_names else np.zeros(n, np.float32)
        g_col = verts[g_key].astype(np.float32) if g_key in prop_names else np.zeros(n, np.float32)
        b_col = verts[b_key].astype(np.float32) if b_key in prop_names else np.zeros(n, np.float32)
        sh_parts.append(np.stack([r_col, g_col, b_col], axis=1))  # (n, 3)

    sh_all = np.concatenate(sh_parts, axis=1)  # (n, 3*(1+nrc_to_use))

    # --- build message
    array_msg = SplatArray()
    for i in range(n):
        splat = Splat()

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = frame_id
        pose.pose.pose.position.x = float(xs[i])
        pose.pose.pose.position.y = float(ys[i])
        pose.pose.pose.position.z = float(zs[i])
        pose.pose.pose.orientation.w = float(quats[i, 0])
        pose.pose.pose.orientation.x = float(quats[i, 1])
        pose.pose.pose.orientation.y = float(quats[i, 2])
        pose.pose.pose.orientation.z = float(quats[i, 3])
        pose.pose.covariance = cov36[i].tolist()
        splat.pose = pose

        splat.opacity = int(opacity_u8[i])
        splat.harmonics_deg = actual_degree
        splat.spherical_harmonics = sh_all[i].tolist()

        array_msg.splats.append(splat)

    return array_msg


class PlySplatPublisher(Node):
    def __init__(self):
        super().__init__('gsplat_publisher')

        self.declare_parameter('ply_path', '')
        self.declare_parameter('sh_degree', 3)

        ply_path = self.get_parameter('ply_path').get_parameter_value().string_value
        sh_degree = self.get_parameter('sh_degree').get_parameter_value().integer_value

        if not ply_path:
            ply_path = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'demo_splat_files', 'only_bonsai.ply'))

        sh_degree = max(0, min(3, sh_degree))

        if not os.path.isfile(ply_path):
            self.get_logger().fatal(f'PLY file not found: {ply_path}')
            raise FileNotFoundError(ply_path)

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(SplatArray, 'gaussian_splats', qos)

        self.get_logger().info(f'Loading {ply_path} (SH degree {sh_degree}) …')
        stamp = self.get_clock().now().to_msg()
        msg = _load_splat_array(ply_path, sh_degree, stamp, frame_id='map')

        self._pub.publish(msg)
        self.get_logger().info(
            f'Published {len(msg.splats)} splats on /gaussian_splats (latched)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PlySplatPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
