#include "gsplat_rviz_plugin/splat_loaders/snapshot_ref_source.hpp"

#include <fstream>
#include <iterator>
#include <utility>
#include <vector>

#include "gsplat_rviz_plugin/splat_loaders/snapshot_payload_loader.hpp"

namespace gsplat_rviz_plugin
{
namespace
{

bool isAbsolutePath(const std::string & path)
{
  return !path.empty() && path.front() == '/';
}

std::string pathFromUri(const std::string & uri, std::string & error)
{
  error.clear();
  constexpr const char * file_prefix = "file://";
  if (uri.rfind(file_prefix, 0) == 0) {
    const std::string path = uri.substr(std::char_traits<char>::length(file_prefix));
    if (!isAbsolutePath(path)) {
      error = "file:// snapshot URI must contain an absolute path.";
    }
    return path;
  }
  if (isAbsolutePath(uri)) {
    return uri;
  }
  error = "Snapshot URI must be file:// or a plain absolute path.";
  return {};
}

std::vector<uint8_t> readFileBytes(const std::string & path, std::string & error)
{
  error.clear();
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) {
    error = "Cannot open snapshot file: " + path;
    return {};
  }
  return std::vector<uint8_t>(
    std::istreambuf_iterator<char>(file),
    std::istreambuf_iterator<char>());
}

}  // namespace

SnapshotRefSource::SnapshotRefSource(
  rclcpp::Node::SharedPtr node,
  const std::string & topic)
: node_(std::move(node)), topic_(topic) {}

SnapshotRefSource::~SnapshotRefSource()
{
  subscription_.reset();
}

void SnapshotRefSource::start(Callback cb)
{
  if (!cb) return;
  callback_ = std::move(cb);

  if (!node_) {
    LoadResult r; r.error = "ROS node unavailable.";
    callback_(std::move(r));
    return;
  }
  if (topic_.empty()) {
    LoadResult r; r.error = "No snapshot ref topic selected.";
    callback_(std::move(r));
    return;
  }

  rclcpp::QoS qos(1);
  qos.reliable().transient_local();

  try {
    subscription_ = node_->create_subscription<gsplat_msgs::msg::SplatSnapshotRef>(
      topic_, qos,
      [this](gsplat_msgs::msg::SplatSnapshotRef::ConstSharedPtr msg) {
        callback_(loadRef(*msg));
      });
  } catch (const std::exception & e) {
    LoadResult r; r.error = std::string("Snapshot ref subscription failed: ") + e.what();
    callback_(std::move(r));
  }
}

LoadResult SnapshotRefSource::loadRef(
  const gsplat_msgs::msg::SplatSnapshotRef & msg) const
{
  LoadResult out;
  std::string error;
  const std::string path = pathFromUri(msg.uri, error);
  if (!error.empty()) {
    out.error = error;
    return out;
  }

  auto bytes = readFileBytes(path, error);
  if (!error.empty()) {
    out.error = error;
    return out;
  }
  if (msg.byte_size != 0 && msg.byte_size != bytes.size()) {
    out.error = "Snapshot byte_size does not match file size.";
    return out;
  }

  return loadSnapshotPayload(
    bytes, msg.format, msg.compression, msg.uncompressed_size, msg.map_from_source);
}

}  // namespace gsplat_rviz_plugin
