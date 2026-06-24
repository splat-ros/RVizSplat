#include "gsplat_rviz_plugin/splat_loaders/blob_topic_source.hpp"

#include <utility>

#include "gsplat_rviz_plugin/splat_loaders/snapshot_payload_loader.hpp"

namespace gsplat_rviz_plugin
{

BlobTopicSource::BlobTopicSource(
  rclcpp::Node::SharedPtr node,
  const std::string & topic)
: node_(std::move(node)), topic_(topic) {}

BlobTopicSource::~BlobTopicSource()
{
  subscription_.reset();
}

void BlobTopicSource::start(Callback cb)
{
  if (!cb) return;
  callback_ = std::move(cb);

  if (!node_) {
    LoadResult r; r.error = "ROS node unavailable.";
    callback_(std::move(r));
    return;
  }
  if (topic_.empty()) {
    LoadResult r; r.error = "No blob topic selected.";
    callback_(std::move(r));
    return;
  }

  rclcpp::QoS qos(rclcpp::KeepLast(64));
  qos.reliable().transient_local();

  try {
    subscription_ = node_->create_subscription<gsplat_msgs::msg::SplatBlobChunk>(
      topic_, qos,
      [this](gsplat_msgs::msg::SplatBlobChunk::ConstSharedPtr msg) {
        onChunk(std::move(msg));
      });
  } catch (const std::exception & e) {
    LoadResult r; r.error = std::string("Blob topic subscription failed: ") + e.what();
    callback_(std::move(r));
  }
}

void BlobTopicSource::onChunk(
  gsplat_msgs::msg::SplatBlobChunk::ConstSharedPtr msg)
{
  auto assembled = assembler_.addChunk(*msg);
  if (!assembled.error.empty()) {
    LoadResult r;
    r.error = assembled.error;
    callback_(std::move(r));
    return;
  }
  if (!assembled.complete) {
    return;
  }

  callback_(loadSnapshotPayload(
    assembled.payload,
    assembled.format,
    assembled.compression,
    assembled.uncompressed_size,
    assembled.map_from_source));
}

}  // namespace gsplat_rviz_plugin
