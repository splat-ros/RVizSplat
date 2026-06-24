#include "gsplat_rviz_plugin/splat_loaders/blob_assembler.hpp"

#include <algorithm>

namespace gsplat_rviz_plugin
{
namespace
{

constexpr uint64_t kCrc64EcmaPoly = 0x42F0E1EBA9EA3693ULL;

}  // namespace

uint64_t crc64Ecma(const uint8_t * data, size_t size)
{
  uint64_t crc = 0;
  for (size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint64_t>(data[i]) << 56;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000000000000000ULL)
        ? (crc << 1) ^ kCrc64EcmaPoly
        : (crc << 1);
    }
  }
  return crc;
}

BlobAssembler::Result BlobAssembler::addChunk(
  const gsplat_msgs::msg::SplatBlobChunk & msg)
{
  Result out;

  if (msg.chunk_count == 0) {
    out.error = "Blob chunk_count is 0.";
    return out;
  }
  if (msg.chunk_index >= msg.chunk_count) {
    out.error = "Blob chunk_index is out of range.";
    return out;
  }
  if (msg.chunk_size != msg.data.size()) {
    out.error = "Blob chunk_size does not match data size.";
    return out;
  }
  if (msg.chunk_size > msg.total_size) {
    out.error = "Blob chunk_size exceeds total_size.";
    return out;
  }
  if (msg.chunk_crc64 != 0 &&
      crc64Ecma(msg.data.data(), msg.data.size()) != msg.chunk_crc64) {
    out.error = "Blob chunk_crc64 mismatch.";
    return out;
  }

  const Key key{msg.session_id, msg.version};
  auto & session = sessions_[key];
  if (session.chunk_count == 0) {
    session.chunk_count = msg.chunk_count;
    session.total_size = msg.total_size;
    session.uncompressed_size = msg.uncompressed_size;
    session.format = msg.format;
    session.compression = msg.compression;
    session.map_from_source = msg.map_from_source;
    session.chunks.resize(msg.chunk_count);
    session.received.assign(msg.chunk_count, false);
  } else if (
    session.chunk_count != msg.chunk_count ||
    session.total_size != msg.total_size ||
    session.uncompressed_size != msg.uncompressed_size ||
    session.format != msg.format ||
    session.compression != msg.compression) {
    sessions_.erase(key);
    out.error = "Blob metadata changed within a session/version.";
    return out;
  }

  if (session.received[msg.chunk_index]) {
    return out;
  }

  if (session.received_size + msg.data.size() > session.total_size) {
    sessions_.erase(key);
    out.error = "Blob received data exceeds total_size.";
    return out;
  }

  session.chunks[msg.chunk_index] = msg.data;
  session.received[msg.chunk_index] = true;
  session.received_size += msg.data.size();
  ++session.received_count;

  if (session.received_count != session.chunk_count) {
    return out;
  }
  if (session.received_size != session.total_size) {
    sessions_.erase(key);
    out.error = "Blob assembled size does not match total_size.";
    return out;
  }

  out.payload.reserve(static_cast<size_t>(session.total_size));
  for (const auto & chunk : session.chunks) {
    out.payload.insert(out.payload.end(), chunk.begin(), chunk.end());
  }
  out.format = session.format;
  out.compression = session.compression;
  out.uncompressed_size = session.uncompressed_size;
  out.map_from_source = session.map_from_source;
  out.complete = true;
  sessions_.erase(key);
  return out;
}

}  // namespace gsplat_rviz_plugin
