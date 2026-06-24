#include "gsplat_rviz_plugin/splat_loaders/tile_stream_source.hpp"

#include <utility>

namespace gsplat_rviz_plugin
{

TileStreamSource::TileStreamSource(
  rclcpp::Node::SharedPtr node,
  const std::string & topic)
: node_(std::move(node)), topic_(topic) {}

TileStreamSource::~TileStreamSource()
{
  subscription_.reset();
}

void TileStreamSource::start(Callback cb)
{
  if (!cb) return;
  callback_ = std::move(cb);

  if (!node_) {
    LoadResult r; r.error = "ROS node unavailable.";
    callback_(std::move(r));
    return;
  }
  if (topic_.empty()) {
    LoadResult r; r.error = "No tile stream topic selected.";
    callback_(std::move(r));
    return;
  }

  rclcpp::QoS qos(rclcpp::KeepLast(100));
  qos.reliable().transient_local();

  try {
    subscription_ = node_->create_subscription<gsplat_msgs::msg::SplatTileChunk>(
      topic_, qos,
      [this](gsplat_msgs::msg::SplatTileChunk::ConstSharedPtr msg) {
        LoadResult result = cache_.handleMessage(*msg);
        if (!result.ok() ||
          msg->operation == gsplat_msgs::msg::SplatTileChunk::COMMIT ||
          msg->operation == gsplat_msgs::msg::SplatTileChunk::HEARTBEAT)
        {
          callback_(std::move(result));
        }
      });
  } catch (const std::exception & e) {
    LoadResult r; r.error = std::string("Tile stream subscription failed: ") + e.what();
    callback_(std::move(r));
  }
}

}  // namespace gsplat_rviz_plugin
