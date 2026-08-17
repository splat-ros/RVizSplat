#ifndef GSPLAT_RVIZ_PLUGIN__SPLAT_LOADERS__MESH_PLY_LOADER_HPP_
#define GSPLAT_RVIZ_PLUGIN__SPLAT_LOADERS__MESH_PLY_LOADER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "gsplat_rviz_plugin/visibility_control.hpp"

namespace gsplat_rviz_plugin
{

// CPU-side representation of a "mesh splat": an ordinary triangle mesh whose
// vertices carry spherical-harmonics colour (DC f_dc_* + higher-order f_rest_*),
// exactly the format produced by Mesh Splatting (see custom_sh_mesh.ply).
//
// Unlike Gaussian splats these are opaque surfaces — no covariance, opacity,
// sorting or WBOIT.  Colour is view-dependent via the SH coefficients.
struct MeshSplatData
{
  // Vertex positions, 3 floats per vertex (object space).
  std::vector<float>    positions;
  // Triangle indices into `positions`, 3 per triangle.
  std::vector<uint32_t> indices;
  // SH coefficients, coefficient-major RGB per vertex:
  //   [v0_c0.rgb, v0_c1.rgb, …, v0_c(stride-1).rgb, v1_c0.rgb, …]
  // where stride = (sh_degree+1)² and coeff 0 is the DC term.
  std::vector<float>    sh;
  int                   sh_degree    = 0;   // 0/1/2/3 from f_rest_* count
  int                   sh_stride    = 1;    // (sh_degree+1)² coeffs per vertex

  std::string           error;              // non-empty on failure

  bool ok() const { return error.empty(); }
  uint32_t vertexCount()   const { return static_cast<uint32_t>(positions.size() / 3); }
  uint32_t triangleCount() const { return static_cast<uint32_t>(indices.size() / 3); }
};

// Load a mesh-splat PLY (binary_little_endian or ascii) from `path`.
// On failure the returned data has a non-empty `error`.
GSPLAT_RVIZ_PLUGIN_PUBLIC
MeshSplatData loadMeshPly(const std::string & path);

}  // namespace gsplat_rviz_plugin

#endif  // GSPLAT_RVIZ_PLUGIN__SPLAT_LOADERS__MESH_PLY_LOADER_HPP_
