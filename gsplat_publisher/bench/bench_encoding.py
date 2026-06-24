#!/usr/bin/env python3
"""Offline encoding benchmark — no ROS dependency.

Usage:
    python bench/bench_encoding.py \\
        --ply-dir /home/bjoern/git/gs-icp-slam-ros2/.tmp_bench/ \\
        --output bench_results.csv [--monitor]
"""

from __future__ import annotations

import argparse
import csv
import glob
import os
import subprocess
import sys
import time

# Allow running directly without install
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..'))

from gsplat_publisher.chunking import build_blob_chunks, estimate_chunk_count
from gsplat_publisher.compact_encoder import (
    COMPACT_DC_FP16_COV_RGBA_V1,
    COMPACT_DC_FP16_COV_RGBA_V1_STRIDE,
    encode_compact_dc_fp16_cov_rgba_v1,
    validate_3dgs_vertex_fields,
)
from gsplat_publisher.ply_reader import read_ply_vertex

_MAX_CHUNK = 1024 * 1024  # 1 MiB

_COLS = [
    'filename',
    'splat_count',
    'read_ms',
    'encode_ms',
    'splats_per_sec',
    'compact_bytes_per_splat',
    'ply_bytes_per_splat',
    'size_reduction_factor',
    'chunk_count',
    'chunking_ms',
]


class _FakeChunk:
    def __init__(self):
        pass


def _bench_one(ply_path: str) -> dict | None:
    filename = os.path.basename(ply_path)
    ply_size = os.path.getsize(ply_path)

    t0 = time.perf_counter()
    vertices = read_ply_vertex(ply_path)
    read_ms = (time.perf_counter() - t0) * 1000

    try:
        validate_3dgs_vertex_fields(vertices)
    except ValueError as exc:
        print(f'  SKIP {filename}: {exc}')
        return None

    splat_count = len(vertices)

    t0 = time.perf_counter()
    payload = encode_compact_dc_fp16_cov_rgba_v1(vertices)
    encode_ms = (time.perf_counter() - t0) * 1000

    splats_per_sec = splat_count / max(encode_ms / 1000, 1e-9)
    ply_bytes_per_splat = ply_size / max(splat_count, 1)
    size_reduction_factor = ply_bytes_per_splat / COMPACT_DC_FP16_COV_RGBA_V1_STRIDE

    t0 = time.perf_counter()
    build_blob_chunks(
        payload,
        session_id='bench',
        version=1,
        format=COMPACT_DC_FP16_COV_RGBA_V1,
        max_chunk_bytes=_MAX_CHUNK,
        msg_factory=_FakeChunk,
        enable_crc=True,
    )
    chunking_ms = (time.perf_counter() - t0) * 1000
    chunk_count = estimate_chunk_count(len(payload), _MAX_CHUNK)

    return {
        'filename': filename,
        'splat_count': splat_count,
        'read_ms': f'{read_ms:.1f}',
        'encode_ms': f'{encode_ms:.1f}',
        'splats_per_sec': f'{splats_per_sec:.0f}',
        'compact_bytes_per_splat': COMPACT_DC_FP16_COV_RGBA_V1_STRIDE,
        'ply_bytes_per_splat': f'{ply_bytes_per_splat:.1f}',
        'size_reduction_factor': f'{size_reduction_factor:.1f}',
        'chunk_count': chunk_count,
        'chunking_ms': f'{chunking_ms:.1f}',
    }


def main() -> None:
    parser = argparse.ArgumentParser(description='Benchmark compact splat encoding')
    parser.add_argument(
        '--ply-dir',
        default=None,
        help='Directory to search (non-recursively) for *.ply files with 3DGS fields'
    )
    parser.add_argument(
        '--ply-files', nargs='+', metavar='PLY',
        help='Explicit PLY file paths to benchmark (overrides --ply-dir)'
    )
    parser.add_argument('--output', default='bench_results.csv', help='CSV output path')
    parser.add_argument(
        '--monitor', action='store_true',
        help='Fork monitor_process.py to record CPU/RAM during benchmark'
    )
    args = parser.parse_args()

    if args.ply_files:
        ply_files = args.ply_files
    elif args.ply_dir:
        ply_files = sorted(glob.glob(os.path.join(args.ply_dir, '*.ply')))
    else:
        # Default: a curated set of known 3DGS PLY files
        ply_files = [
            '/home/bjoern/git/splatograph/tests/regression/golden/fixture.ply',
            '/home/bjoern/git/splatograph/output/floor3_orbbec_live/point_cloud/iteration_8000/point_cloud.ply',
            '/home/bjoern/git/splatograph/output/floor3_orbbec_live/point_cloud/iteration_18000/point_cloud.ply',
        ]
        ply_files = [f for f in ply_files if os.path.isfile(f)]
    if not ply_files:
        print(f'No PLY files found in {args.ply_dir}')
        sys.exit(1)

    monitor_proc = None
    if args.monitor:
        monitor_script = os.path.join(
            _HERE, '../../gsplat_plugin_evaluation/monitor_process.py'
        )
        monitor_proc = subprocess.Popen(
            [sys.executable, monitor_script, str(os.getpid()),
             '--interval', '0.5', '--output', 'monitor_out.csv'],
        )

    results = []
    for ply_path in ply_files:
        print(f'Benchmarking {os.path.basename(ply_path)} …')
        row = _bench_one(ply_path)
        if row:
            results.append(row)
            print(
                f'  {row["splat_count"]:>8} splats | '
                f'encode {row["encode_ms"]:>7}ms | '
                f'{float(row["splats_per_sec"]):>10.0f} splats/s | '
                f'{row["size_reduction_factor"]}x smaller | '
                f'{row["chunk_count"]} chunks'
            )

    if monitor_proc:
        monitor_proc.terminate()

    if not results:
        print('No valid results.')
        sys.exit(1)

    with open(args.output, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=_COLS)
        writer.writeheader()
        writer.writerows(results)
    print(f'\nWrote {len(results)} rows to {args.output}')

    # Summary
    throughputs = [float(r['splats_per_sec']) for r in results]
    print(
        f'\nEncoding throughput: '
        f'min={min(throughputs):,.0f}  '
        f'mean={sum(throughputs)/len(throughputs):,.0f}  '
        f'max={max(throughputs):,.0f}  splats/sec'
    )


if __name__ == '__main__':
    main()
