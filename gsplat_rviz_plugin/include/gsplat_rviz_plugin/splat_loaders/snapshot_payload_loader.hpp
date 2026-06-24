#ifndef GSPLAT_RVIZ_PLUGIN__SNAPSHOT_PAYLOAD_LOADER_HPP_
#define GSPLAT_RVIZ_PLUGIN__SNAPSHOT_PAYLOAD_LOADER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform.hpp>

#include "gsplat_rviz_plugin/splat_loaders/i_splat_source.hpp"

namespace gsplat_rviz_plugin
{

LoadResult loadSnapshotPayload(
  const std::vector<uint8_t> & payload,
  const std::string & format,
  const std::string & compression,
  uint64_t uncompressed_size,
  const geometry_msgs::msg::Transform & map_from_source);

}  // namespace gsplat_rviz_plugin

#endif  // GSPLAT_RVIZ_PLUGIN__SNAPSHOT_PAYLOAD_LOADER_HPP_
