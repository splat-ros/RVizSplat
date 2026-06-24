#ifndef GSPLAT_RVIZ_PLUGIN__PLY_LOADER_HPP_
#define GSPLAT_RVIZ_PLUGIN__PLY_LOADER_HPP_

#include <string>
#include <cstdint>
#include <vector>

#include "gsplat_rviz_plugin/splat_gpu.hpp"
#include "gsplat_rviz_plugin/visibility_control.hpp"

namespace gsplat_rviz_plugin
{

// Load all Gaussian splats from a 3DGS-format PLY file into GPU-ready structs.
// sh_degree is set to 0/1/2/3 based on the number of f_rest_* properties found.
// Returns an empty vector and sets error_msg on failure.
GSPLAT_RVIZ_PLUGIN_PUBLIC
std::vector<SplatGPU> loadPly(
  const std::string & path,
  std::string & error_msg,
  int & sh_degree);

GSPLAT_RVIZ_PLUGIN_PUBLIC
std::vector<SplatGPU> loadPlyBytes(
  const std::vector<uint8_t> & bytes,
  std::string & error_msg,
  int & sh_degree);

GSPLAT_RVIZ_PLUGIN_PUBLIC
std::vector<SplatGPU> loadPlyBytes(
  const std::string & bytes,
  std::string & error_msg,
  int & sh_degree);

}  // namespace gsplat_rviz_plugin

#endif  // GSPLAT_RVIZ_PLUGIN__PLY_LOADER_HPP_
