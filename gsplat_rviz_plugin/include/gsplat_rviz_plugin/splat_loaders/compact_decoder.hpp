#ifndef GSPLAT_RVIZ_PLUGIN__COMPACT_DECODER_HPP_
#define GSPLAT_RVIZ_PLUGIN__COMPACT_DECODER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform.hpp>

#include "gsplat_rviz_plugin/splat_gpu.hpp"

namespace gsplat_rviz_plugin
{

std::vector<SplatGPU> decodeCompactDcFp16CovRgbaV1(
  const std::vector<uint8_t> & bytes,
  std::string & error_msg);

void applyMapFromSourceTransform(
  std::vector<SplatGPU> & splats,
  const geometry_msgs::msg::Transform & transform);

}  // namespace gsplat_rviz_plugin

#endif  // GSPLAT_RVIZ_PLUGIN__COMPACT_DECODER_HPP_
