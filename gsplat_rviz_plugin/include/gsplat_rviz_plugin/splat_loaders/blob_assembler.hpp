#ifndef GSPLAT_RVIZ_PLUGIN__BLOB_ASSEMBLER_HPP_
#define GSPLAT_RVIZ_PLUGIN__BLOB_ASSEMBLER_HPP_

#include <cstdint>
#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform.hpp>
#include <gsplat_msgs/msg/splat_blob_chunk.hpp>

namespace gsplat_rviz_plugin
{

class BlobAssembler
{
public:
  struct Result
  {
    bool complete = false;
    std::vector<uint8_t> payload;
    std::string format;
    std::string compression;
    uint64_t uncompressed_size = 0;
    geometry_msgs::msg::Transform map_from_source;
    std::string error;
  };

  Result addChunk(const gsplat_msgs::msg::SplatBlobChunk & msg);

private:
  struct Key
  {
    std::string session_id;
    uint64_t version = 0;

    bool operator<(const Key & other) const
    {
      if (session_id != other.session_id) return session_id < other.session_id;
      return version < other.version;
    }
  };

  struct Session
  {
    uint32_t chunk_count = 0;
    uint64_t total_size = 0;
    uint64_t uncompressed_size = 0;
    std::string format;
    std::string compression;
    geometry_msgs::msg::Transform map_from_source;
    std::vector<std::vector<uint8_t>> chunks;
    std::vector<bool> received;
    uint64_t received_size = 0;
    uint32_t received_count = 0;
  };

  std::map<Key, Session> sessions_;
};

uint64_t crc64Ecma(const uint8_t * data, size_t size);

}  // namespace gsplat_rviz_plugin

#endif  // GSPLAT_RVIZ_PLUGIN__BLOB_ASSEMBLER_HPP_
