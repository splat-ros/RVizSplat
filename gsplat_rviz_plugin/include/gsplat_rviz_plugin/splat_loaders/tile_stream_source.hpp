#ifndef GSPLAT_RVIZ_PLUGIN__SPLAT_LOADERS__TILE_STREAM_SOURCE_HPP_
#define GSPLAT_RVIZ_PLUGIN__SPLAT_LOADERS__TILE_STREAM_SOURCE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <gsplat_msgs/msg/splat_tile_chunk.hpp>

#include "gsplat_rviz_plugin/splat_loaders/i_splat_source.hpp"
#include "gsplat_rviz_plugin/splat_loaders/tile_cache.hpp"

namespace gsplat_rviz_plugin
{

class TileStreamSource : public ISplatSource
{
public:
  TileStreamSource(
    rclcpp::Node::SharedPtr node,
    const std::string & topic);
  ~TileStreamSource() override;

  void start(Callback cb) override;

private:
  rclcpp::Node::SharedPtr node_;
  std::string topic_;
  Callback callback_;
  TileCache cache_;
  rclcpp::Subscription<gsplat_msgs::msg::SplatTileChunk>::SharedPtr subscription_;
};

}  // namespace gsplat_rviz_plugin

#endif  // GSPLAT_RVIZ_PLUGIN__SPLAT_LOADERS__TILE_STREAM_SOURCE_HPP_
