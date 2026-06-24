#include "gsplat_rviz_plugin/splat_loaders/tile_cache.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <sstream>
#include <utility>

namespace gsplat_rviz_plugin
{
namespace
{

constexpr const char * kCompactEncoding = "compact_dc_fp16_cov_rgba_v1";
constexpr size_t kCompactStride = 32;
constexpr float kShC0 = 0.28209479177387814f;

static LoadResult errorResult(const std::string & error)
{
  LoadResult result;
  result.error = error;
  return result;
}

static uint16_t readU16LE(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0]) |
    static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8);
}

static uint32_t readU32LE(const uint8_t * p)
{
  return static_cast<uint32_t>(p[0]) |
    (static_cast<uint32_t>(p[1]) << 8) |
    (static_cast<uint32_t>(p[2]) << 16) |
    (static_cast<uint32_t>(p[3]) << 24);
}

static float readF32LE(const uint8_t * p)
{
  const uint32_t raw = readU32LE(p);
  float value = 0.0f;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

static float halfToFloat(uint16_t h)
{
  const uint32_t sign = (static_cast<uint32_t>(h & 0x8000u)) << 16;
  uint32_t exp = (h >> 10) & 0x1Fu;
  uint32_t mant = h & 0x03FFu;
  uint32_t out = 0;

  if (exp == 0) {
    if (mant == 0) {
      out = sign;
    } else {
      exp = 127 - 15 + 1;
      while ((mant & 0x0400u) == 0) {
        mant <<= 1;
        --exp;
      }
      mant &= 0x03FFu;
      out = sign | (exp << 23) | (mant << 13);
    }
  } else if (exp == 0x1Fu) {
    out = sign | 0x7F800000u | (mant << 13);
  } else {
    exp = exp - 15 + 127;
    out = sign | (exp << 23) | (mant << 13);
  }

  float value = 0.0f;
  std::memcpy(&value, &out, sizeof(value));
  return value;
}

// Wire layout for compact_dc_fp16_cov_rgba_v1:
//   bytes  0..11  float32 LE center x/y/z
//   bytes 12..23  float16 LE covariance {xx, xy, xz, yy, yz, zz}
//   bytes 24..27  unorm8 rgba, DC colour already baked
//   bytes 28..31  reserved
static LoadResult decodeCompactTile(
  const std::vector<uint8_t> & bytes,
  uint32_t splat_count,
  uint32_t stride,
  uint64_t uncompressed_size)
{
  if (stride != kCompactStride) {
    std::ostringstream ss;
    ss << "Unsupported tile stride " << stride << " for " << kCompactEncoding
       << " (expected " << kCompactStride << ").";
    return errorResult(ss.str());
  }

  const uint64_t expected_size = static_cast<uint64_t>(splat_count) * stride;
  if (uncompressed_size != 0 && uncompressed_size != expected_size) {
    std::ostringstream ss;
    ss << "Tile uncompressed_size " << uncompressed_size
       << " does not match splat_count*stride " << expected_size << ".";
    return errorResult(ss.str());
  }
  if (bytes.size() != expected_size) {
    std::ostringstream ss;
    ss << "Decoded tile byte count " << bytes.size()
       << " does not match expected " << expected_size << ".";
    return errorResult(ss.str());
  }

  LoadResult out;
  out.sh_degree = 0;
  out.splats.reserve(splat_count);
  for (uint32_t i = 0; i < splat_count; ++i) {
    const uint8_t * p = bytes.data() + static_cast<size_t>(i) * kCompactStride;
    SplatGPU g{};

    g.center[0] = readF32LE(p + 0);
    g.center[1] = readF32LE(p + 4);
    g.center[2] = readF32LE(p + 8);

    g.covA[0] = halfToFloat(readU16LE(p + 12));
    g.covA[1] = halfToFloat(readU16LE(p + 14));
    g.covA[2] = halfToFloat(readU16LE(p + 16));
    g.covB[0] = halfToFloat(readU16LE(p + 18));
    g.covB[1] = halfToFloat(readU16LE(p + 20));
    g.covB[2] = halfToFloat(readU16LE(p + 22));

    const float r = static_cast<float>(p[24]) / 255.0f;
    const float gch = static_cast<float>(p[25]) / 255.0f;
    const float b = static_cast<float>(p[26]) / 255.0f;
    g.alpha = static_cast<float>(p[27]) / 255.0f;

    g.sh[0][0] = (r - 0.5f) / kShC0;
    g.sh[0][1] = (gch - 0.5f) / kShC0;
    g.sh[0][2] = (b - 0.5f) / kShC0;
    g.sh[0][3] = 0.0f;

    out.splats.push_back(g);
  }

  return out;
}

static TileKey tileKeyFromMsg(const gsplat_msgs::msg::SplatTileChunk & msg)
{
  return TileKey{msg.tile_x, msg.tile_y, msg.tile_z, msg.lod};
}

}  // namespace

bool TileKey::operator<(const TileKey & rhs) const
{
  if (x != rhs.x) return x < rhs.x;
  if (y != rhs.y) return y < rhs.y;
  if (z != rhs.z) return z < rhs.z;
  return lod < rhs.lod;
}

bool TileAssembler::AssemblyKey::operator<(const AssemblyKey & rhs) const
{
  if (session_id != rhs.session_id) return session_id < rhs.session_id;
  if (version != rhs.version) return version < rhs.version;
  return tile < rhs.tile;
}

TileAssembler::Result TileAssembler::addChunk(
  const gsplat_msgs::msg::SplatTileChunk & msg)
{
  Result result;

  if (msg.compression != "none") {
    result.error = "Unsupported tile compression '" + msg.compression +
      "'. Only compression='none' is supported.";
    return result;
  }
  if (msg.chunk_count == 0) {
    result.error = "Invalid tile chunk_count 0.";
    return result;
  }
  if (msg.chunk_index >= msg.chunk_count) {
    result.error = "Invalid tile chunk_index " + std::to_string(msg.chunk_index) +
      " for chunk_count " + std::to_string(msg.chunk_count) + ".";
    return result;
  }
  if (msg.uncompressed_size > static_cast<uint64_t>(std::numeric_limits<size_t>::max())) {
    result.error = "Tile uncompressed_size is too large for this platform.";
    return result;
  }

  const AssemblyKey key{msg.session_id, msg.version, tileKeyFromMsg(msg)};
  auto & assembly = assemblies_[key];
  if (assembly.chunk_count == 0) {
    assembly.chunk_count = msg.chunk_count;
    assembly.uncompressed_size = msg.uncompressed_size;
    assembly.chunks.resize(msg.chunk_count);
    assembly.received.assign(msg.chunk_count, false);
  } else {
    if (assembly.chunk_count != msg.chunk_count ||
      assembly.uncompressed_size != msg.uncompressed_size)
    {
      assemblies_.erase(key);
      result.error = "Inconsistent chunk metadata for tile assembly.";
      return result;
    }
  }

  if (assembly.received[msg.chunk_index]) {
    return result;
  }
  assembly.received[msg.chunk_index] = true;
  ++assembly.received_count;
  assembly.chunks[msg.chunk_index] = msg.data;

  if (assembly.received_count != assembly.chunk_count) {
    return result;
  }

  size_t total = 0;
  for (const auto & chunk : assembly.chunks) {
    total += chunk.size();
  }
  if (total != assembly.uncompressed_size) {
    assemblies_.erase(key);
    result.error = "Assembled tile byte count " + std::to_string(total) +
      " does not match uncompressed_size " +
      std::to_string(assembly.uncompressed_size) + ".";
    return result;
  }

  result.data.reserve(total);
  for (const auto & chunk : assembly.chunks) {
    result.data.insert(result.data.end(), chunk.begin(), chunk.end());
  }
  result.complete = true;
  assemblies_.erase(key);
  return result;
}

bool TileAssembler::hasIncomplete(const std::string & session_id, uint64_t version) const
{
  for (const auto & entry : assemblies_) {
    if (entry.first.session_id == session_id && entry.first.version == version &&
      entry.second.received_count != entry.second.chunk_count)
    {
      return true;
    }
  }
  return false;
}

void TileAssembler::discardVersion(const std::string & session_id, uint64_t version)
{
  for (auto it = assemblies_.begin(); it != assemblies_.end(); ) {
    if (it->first.session_id == session_id && it->first.version == version) {
      it = assemblies_.erase(it);
    } else {
      ++it;
    }
  }
}

LoadResult TileCache::handleMessage(const gsplat_msgs::msg::SplatTileChunk & msg)
{
  const auto pending_key = std::make_pair(msg.session_id, msg.version);
  const TileKey tile = tileKeyFromMsg(msg);

  switch (msg.operation) {
    case gsplat_msgs::msg::SplatTileChunk::HEARTBEAT:
      return snapshot();

    case gsplat_msgs::msg::SplatTileChunk::CLEAR_ALL:
      pending_[pending_key].push_back(PendingOp{PendingOp::Kind::ClearAll, tile, {}});
      return snapshot();

    case gsplat_msgs::msg::SplatTileChunk::DELETE_TILE:
      pending_[pending_key].push_back(PendingOp{PendingOp::Kind::DeleteTile, tile, {}});
      return snapshot();

    case gsplat_msgs::msg::SplatTileChunk::UPSERT_TILE: {
      auto assembled = assembler_.addChunk(msg);
      if (!assembled.error.empty()) return errorResult(assembled.error);
      if (!assembled.complete) return snapshot();
      return decodeAndQueueUpsert(msg, std::move(assembled.data));
    }

    case gsplat_msgs::msg::SplatTileChunk::COMMIT:
      return commit(msg.session_id, msg.version);

    default:
      return errorResult("Unknown tile operation " + std::to_string(msg.operation) + ".");
  }
}

LoadResult TileCache::decodeAndQueueUpsert(
  const gsplat_msgs::msg::SplatTileChunk & msg,
  std::vector<uint8_t> bytes)
{
  if (msg.encoding != kCompactEncoding) {
    return errorResult("Unsupported tile encoding '" + msg.encoding +
      "'. Expected '" + kCompactEncoding + "'.");
  }
  if (msg.compression != "none") {
    return errorResult("Unsupported tile compression '" + msg.compression +
      "'. Only compression='none' is supported.");
  }

  LoadResult decoded = decodeCompactTile(
    bytes, msg.splat_count, msg.stride, msg.uncompressed_size);
  if (!decoded.ok()) return decoded;

  TileData tile_data;
  tile_data.splats = std::move(decoded.splats);
  tile_data.sh_degree = 0;
  tile_data.encoding = msg.encoding;
  tile_data.splat_count = msg.splat_count;

  const auto pending_key = std::make_pair(msg.session_id, msg.version);
  pending_[pending_key].push_back(
    PendingOp{PendingOp::Kind::UpsertTile, tileKeyFromMsg(msg), std::move(tile_data)});

  return snapshot();
}

LoadResult TileCache::commit(const std::string & session_id, uint64_t version)
{
  if (assembler_.hasIncomplete(session_id, version)) {
    // Discard orphaned pending ops for this version to avoid a memory leak
    // and leave a stale partial state if chunks never arrive.
    pending_.erase(std::make_pair(session_id, version));
    assembler_.discardVersion(session_id, version);
    return errorResult("Cannot commit tile version " + std::to_string(version) +
      " while one or more tile assemblies are incomplete.");
  }

  const auto key = std::make_pair(session_id, version);
  auto it = pending_.find(key);
  if (it == pending_.end()) return snapshot();

  auto next_tiles = tiles_;
  for (auto & op : it->second) {
    switch (op.kind) {
      case PendingOp::Kind::ClearAll:
        next_tiles.clear();
        break;
      case PendingOp::Kind::DeleteTile:
        next_tiles.erase(op.tile);
        break;
      case PendingOp::Kind::UpsertTile:
        next_tiles[op.tile] = std::move(op.data);
        break;
    }
  }

  tiles_ = std::move(next_tiles);
  pending_.erase(it);
  return snapshot();
}

LoadResult TileCache::snapshot() const
{
  LoadResult out;
  out.sh_degree = 0;

  size_t total = 0;
  for (const auto & entry : tiles_) {
    total += entry.second.splats.size();
    out.sh_degree = std::max(out.sh_degree, entry.second.sh_degree);
  }

  out.splats.reserve(total);
  for (const auto & entry : tiles_) {
    const auto & splats = entry.second.splats;
    out.splats.insert(out.splats.end(), splats.begin(), splats.end());
  }
  return out;
}

}  // namespace gsplat_rviz_plugin
