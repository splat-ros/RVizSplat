#ifndef GSPLAT_RVIZ_PLUGIN__SPLAT_LOADERS__TILE_CACHE_HPP_
#define GSPLAT_RVIZ_PLUGIN__SPLAT_LOADERS__TILE_CACHE_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <gsplat_msgs/msg/splat_tile_chunk.hpp>

#include "gsplat_rviz_plugin/splat_gpu.hpp"
#include "gsplat_rviz_plugin/splat_loaders/i_splat_source.hpp"

namespace gsplat_rviz_plugin
{

struct TileKey
{
  int32_t x = 0;
  int32_t y = 0;
  int32_t z = 0;
  uint8_t lod = 0;

  bool operator<(const TileKey & rhs) const;
};

struct TileData
{
  std::vector<SplatGPU> splats;
  int sh_degree = 0;
  std::string encoding;
  uint32_t splat_count = 0;
};

class TileAssembler
{
public:
  struct Result
  {
    bool complete = false;
    std::vector<uint8_t> data;
    std::string error;
  };

  Result addChunk(const gsplat_msgs::msg::SplatTileChunk & msg);
  bool hasIncomplete(const std::string & session_id, uint64_t version) const;
  void discardVersion(const std::string & session_id, uint64_t version);

private:
  struct AssemblyKey
  {
    std::string session_id;
    uint64_t version = 0;
    TileKey tile;

    bool operator<(const AssemblyKey & rhs) const;
  };

  struct Assembly
  {
    uint32_t chunk_count = 0;
    uint64_t uncompressed_size = 0;
    uint64_t received_count = 0;
    std::vector<std::vector<uint8_t>> chunks;
    std::vector<bool> received;
  };

  std::map<AssemblyKey, Assembly> assemblies_;
};

class TileCache
{
public:
  LoadResult handleMessage(const gsplat_msgs::msg::SplatTileChunk & msg);
  LoadResult snapshot() const;

private:
  struct PendingOp
  {
    enum class Kind { ClearAll, UpsertTile, DeleteTile };

    Kind kind = Kind::DeleteTile;
    TileKey tile;
    TileData data;
  };

  LoadResult decodeAndQueueUpsert(
    const gsplat_msgs::msg::SplatTileChunk & msg,
    std::vector<uint8_t> bytes);
  LoadResult commit(const std::string & session_id, uint64_t version);

  std::map<TileKey, TileData> tiles_;
  std::map<std::pair<std::string, uint64_t>, std::vector<PendingOp>> pending_;
  TileAssembler assembler_;
};

}  // namespace gsplat_rviz_plugin

#endif  // GSPLAT_RVIZ_PLUGIN__SPLAT_LOADERS__TILE_CACHE_HPP_
