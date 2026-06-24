# Transport Performance Benchmark

## Transport Design Overview

RVizSplat provides four transport paths for streaming Gaussian Splat maps to RViz2. They
differ in what goes over ROS2 and where decoding happens.

### SplatArray (legacy)
**Format:** One `Splat` ROS message per Gaussian. Each message carries position
(PoseWithCovarianceStamped), per-Gaussian scale/rotation as a 6×6 covariance matrix in
float64, DC spherical-harmonic colour, and opacity — roughly 400+ bytes per Gaussian.

**How it works:** Publisher serialises every Gaussian individually into a `SplatArray`
and publishes it. Subscriber reconstructs the cloud and uploads to GPU.

**Trade-off:** Zero compression effort, human-readable on `ros2 topic echo`, but
throughput scales quadratically with splat count due to the per-Gaussian overhead in
CDR serialisation.

---

### Blob (compact binary, latched)
**Format:** `compact_dc_fp16_cov_rgba_v1` — 32 bytes per Gaussian:
```
bytes  0..11  float32 LE  centre x, y, z
bytes 12..23  float16 LE  covariance upper triangle {xx, xy, xz, yy, yz, zz}
bytes 24..27  uint8       RGBA — DC SH colour baked in, opacity
bytes 28..31  uint32      id_or_flags (reserved)
```
The full scene is encoded once, chunked into ≤ 1 MiB `SplatBlobChunk` messages, and
published with `TRANSIENT_LOCAL` (latched) QoS. The C++ `BlobAssembler` reassembles
chunks; CRC-64/ECMA-182 validates each chunk.

**How it works:** Publisher computes covariance (R S² Rᵀ) and bakes SH DC → uint8 RGB
once. Subscriber buffers arriving chunks, verifies CRC, decodes all Gaussians on the
first complete version and uploads to GPU. Static snapshots need only one publish.

**Trade-off:** 12× smaller than SplatArray, fast encode (~2M splats/s), but the entire
scene is retransmitted every cycle regardless of what changed.

---

### Tile stream (incremental spatial update)
**Format:** Same 32-byte compact encoding, but split into spatial tiles. Each
`SplatTileChunk` message carries an (x, y, z, lod) tile coordinate plus an operation:

| Operation | Meaning |
|-----------|---------|
| `CLEAR_ALL` | Discard all committed tiles for this session |
| `UPSERT_TILE` | Replace or insert one tile's Gaussian data |
| `DELETE_TILE` | Remove one tile |
| `COMMIT` | Atomically apply all pending ops and trigger a render update |
| `HEARTBEAT` | Keep-alive, returns current snapshot without update |

A publish cycle is: `CLEAR_ALL → N × UPSERT_TILE → COMMIT`. The C++ `TileCache`
accumulates pending ops per (session_id, version) and applies them atomically on COMMIT.
The `TileAssembler` handles tiles split across multiple chunks.

**How it works:** Because tiles are addressed spatially, only changed tiles need to be
sent on incremental updates (live SLAM), making bandwidth proportional to the volume of
change rather than total splat count.

**Trade-off:** Best for incremental live SLAM; requires a full-cycle CLEAR_ALL+UPSERT+COMMIT
at each publish. Tile count determines message count — choose `tile_size` so that the
full cycle fits comfortably within the QoS history depth (default 500).

---

### SnapshotRef (zero in-band data)
**Format:** A single `SplatSnapshotRef` message containing a `file://` URI (or
other scheme). 189 bytes total — essentially a filename over ROS2.

**How it works:** Publisher emits only a URI string. The C++ subscriber loads the file
directly from the path (must be accessible on the subscriber's filesystem). Decoding
happens entirely in the subscriber process, bypassing ROS2 data transport.

**Trade-off:** Zero network cost, loads any PLY the subscriber can read, but requires
publisher and subscriber to share a filesystem (co-located processes or NFS mount).

---

## Benchmark Results

**Scene:** `kf_e4_cap300k_fr3_20260622` iteration 30 000 (TUM fr3 desk, SLAM trajectory)
**Splat count:** 21,570
**Host:** cps-wkstn-hpz2minig1a (AMD ROCm workstation)
**Middleware:** `rmw_zenoh_cpp` (loopback, host network)
**Measured with:** `ros2 topic bw` over 10 s windows, one transport at a time
**Tile size:** 5.0 m (59 tiles, 61 messages/cycle)

### Bandwidth by transport and publish rate

| Transport | Msg size (mean) | 3 Hz | 5 Hz | 10 Hz | 30 Hz |
|---|---|---|---|---|---|
| **SplatArray** (legacy) | ~8 MB | ~25 MB/s | ~42 MB/s | ~83 MB/s | ~122 MB/s ⚠️ |
| **Blob (compact)** | 690 KB | 2.1 MB/s | 3.5 MB/s | 6.9 MB/s | 12.0 MB/s |
| **Tile stream** | 11–437 KB | 92 KB/s | 104 KB/s | 2.1 MB/s | 4.4 MB/s |
| **SnapshotRef** | 189 B | 571 B/s | 946 B/s | 1.9 KB/s | 5.7 KB/s |

> SplatArray values are extrapolated from prior run (serialisation is ~100× slower in
> pure Python; measurements taken at 3 / 5 / 10 Hz only).

### Size reduction

| Transport | Bytes / splat | vs SplatArray |
|---|---|---|
| SplatArray (legacy) | ~383 B | 1× (baseline) |
| Blob compact (`compact_dc_fp16_cov_rgba_v1`) | 32 B | **12× smaller** |
| Tile stream (same compact encoding per tile) | 32 B + framing | ~12× smaller |
| SnapshotRef | 189 B total (URI only) | no data in-band |

PLY file size for this scene: 5.2 MB → compact payload: 674 KB (**7.7× smaller than raw PLY**).

---

## Real-time Feasibility

### What "real-time" means here
Gaussian Splat maps from live SLAM change continuously. The transport must keep up with
the SLAM update rate (typically 1–10 Hz) without saturating the network link.

| Transport | 1 Gbps LAN ceiling | 100 Mbps LAN ceiling | Verdict |
|---|---|---|---|
| SplatArray | ~12 Hz (this scene) | ~1 Hz | Not viable above small scenes / low Hz |
| Blob | ~180 Hz | ~18 Hz | **Viable for 1–30 Hz live snapshots** |
| Tile stream | Well under ceiling at all rates | Same | **Ideal for incremental SLAM updates** |
| SnapshotRef | Network-free | Network-free | **Viable whenever filesystem is shared** |

### Operating point recommendations

| Use case | Transport | Notes |
|---|---|---|
| One-shot static map delivery | Blob (latched, TRANSIENT_LOCAL) | Subscriber gets full map whenever it joins |
| Live SLAM snapshot at ≤ 10 Hz | Blob | Retransmits full scene each cycle |
| Live SLAM incremental at ≤ 30 Hz | Tile stream | Only changed tiles sent; BW ∝ change volume |
| Co-located publisher + subscriber | SnapshotRef | Zero network cost |
| Legacy / compatibility | SplatArray | Only for small scenes (< 1k splats) or < 1 Hz |

### Blob saturation analysis
At 30 Hz, Blob requires 12 MB/s sustained. This is achievable on:
- **Loopback / same host**: up to ~30 Hz (tested)
- **1 Gbps LAN**: ceiling ~180 Hz — never saturates for this scene size
- **100 Mbps LAN**: ceiling ~18 Hz — viable at ≤ 10 Hz
- **Wi-Fi (50 Mbps typical)**: ceiling ~9 Hz — viable at ≤ 5 Hz
- **ROS2 DDS default**: saturates around 5–10 Hz without tuning (DDS fragmentation overhead)

`rmw_zenoh_cpp` (used here) handles large messages efficiently over TCP without DDS
fragmentation overhead, which is why it's preferred for this use case.

### Tile stream: choosing tile_size
The number of messages per publish cycle equals `n_tiles + 2` (CLEAR_ALL + COMMIT).
QoS history depth must cover this to support late-joining subscribers:

| Scene extent | tile_size | Tiles | Msgs/cycle | QoS depth needed |
|---|---|---|---|---|
| Desk (2 m) | 1.0 m | ~8 | ~10 | 20 |
| Room (10 m) | 2.0 m | ~125 | ~127 | 200 |
| Building (50 m) | 5.0 m | ~59 | ~61 | 100 |
| Campus (200 m) | 10.0 m | ~200 | ~202 | 300 |

The default `TILE_QOS` uses depth=500, sufficient for most scenes. For very large
scenes, increase depth or tile_size.

---

## Encoding throughput (Python publisher)

Measured with `bench/bench_encoding.py` on the same host:

| Scene | Splats | Encode time | Throughput |
|---|---|---|---|
| fixture (golden) | 600 | 0.5 ms | 1.3M splats/s |
| floor3 iter 8000 | 47,758 | 24 ms | 2.0M splats/s |
| floor3 iter 18000 | 47,941 | 23 ms | 2.1M splats/s |

CRC-64/ECMA-182 is computed per chunk (pure-Python fallback, no C extension available
on this host). For 1 MiB chunks the CRC adds ~80 ms/chunk. Disable with
`enable_crc=False` for high-rate tile streams where latency matters more than integrity.

---

## Known issues (resolved)

- **Tile stream not rendering in RViz2** — fixed. Four root causes patched:
  1. **QoS durability mismatch** (`tile_stream_source.cpp`): subscriber used `VOLATILE`,
     publisher used `TRANSIENT_LOCAL`. Subscriber now uses `TRANSIENT_LOCAL` depth=100.
  2. **QoS history depth too small** (`rate_publisher_all.py`): `LATCHED_QOS` depth=10
     could not hold a full CLEAR_ALL + N×UPSERT_TILE + COMMIT cycle. `TILE_QOS`
     depth=500 is now used for tile publishers.
  3. **Orphaned pending ops on failed COMMIT** (`tile_cache.cpp`): when COMMIT arrived
     with incomplete tile assemblies, pending ops and assembly entries were leaked.
     `TileAssembler::discardVersion()` now cleans them up on error.
  4. **Version not incremented between cycles** (`rate_publisher_all.py`): tile messages
     were pre-built once at version=1 and version never advanced. Each cycle now rebuilds
     with the current incremented version.
  5. **Tile size too small** (`rate_publisher_all.py`): default 1.0 m tile size produced
     317 tiles (319 msgs/cycle) for the fr3 desk scene which spans ~44 m. Changed default
     to 5.0 m (59 tiles, 61 msgs/cycle). Pass `--tile-size` to override.

- **SplatArray serialisation** is per-Gaussian in Python (loop), making it slow for large
  scenes. The splatograph-rvizsplat-transport fork uses a vectorised CDR serialiser
  (~100× faster) for live publishing.
