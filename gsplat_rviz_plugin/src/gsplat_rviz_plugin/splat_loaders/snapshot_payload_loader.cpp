#include "gsplat_rviz_plugin/splat_loaders/snapshot_payload_loader.hpp"

#include "gsplat_rviz_plugin/splat_loaders/compact_decoder.hpp"
#include "gsplat_rviz_plugin/splat_loaders/ply_loader.hpp"

namespace gsplat_rviz_plugin
{

LoadResult loadSnapshotPayload(
  const std::vector<uint8_t> & payload,
  const std::string & format,
  const std::string & compression,
  uint64_t uncompressed_size,
  const geometry_msgs::msg::Transform & map_from_source)
{
  LoadResult out;

  if (!compression.empty() && compression != "none") {
    out.error = "Unsupported snapshot compression '" + compression + "'; only 'none' is supported.";
    return out;
  }
  if (uncompressed_size != 0 && uncompressed_size != payload.size()) {
    out.error = "Snapshot uncompressed_size does not match payload size for compression='none'.";
    return out;
  }

  if (format == "ply_binary" || format == "ply_ascii" || format == "ply") {
    out.splats = loadPlyBytes(payload, out.error, out.sh_degree);
  } else if (format == "compact_dc_fp16_cov_rgba_v1") {
    out.splats = decodeCompactDcFp16CovRgbaV1(payload, out.error);
    out.sh_degree = 0;
  } else {
    out.error = "Unsupported snapshot format '" + format + "'.";
    return out;
  }

  if (out.ok()) {
    applyMapFromSourceTransform(out.splats, map_from_source);
  }
  return out;
}

}  // namespace gsplat_rviz_plugin
