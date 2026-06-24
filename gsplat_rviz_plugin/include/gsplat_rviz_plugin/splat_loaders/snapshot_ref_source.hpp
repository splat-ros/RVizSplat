#ifndef GSPLAT_RVIZ_PLUGIN__SNAPSHOT_REF_SOURCE_HPP_
#define GSPLAT_RVIZ_PLUGIN__SNAPSHOT_REF_SOURCE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <gsplat_msgs/msg/splat_snapshot_ref.hpp>

#include "gsplat_rviz_plugin/splat_loaders/i_splat_source.hpp"

namespace gsplat_rviz_plugin
{

class SnapshotRefSource : public ISplatSource
{
public:
  SnapshotRefSource(
    rclcpp::Node::SharedPtr node,
    const std::string & topic);
  ~SnapshotRefSource() override;

  void start(Callback cb) override;

private:
  LoadResult loadRef(const gsplat_msgs::msg::SplatSnapshotRef & msg) const;

  rclcpp::Node::SharedPtr node_;
  std::string topic_;
  Callback callback_;
  rclcpp::Subscription<gsplat_msgs::msg::SplatSnapshotRef>::SharedPtr subscription_;
};

}  // namespace gsplat_rviz_plugin

#endif  // GSPLAT_RVIZ_PLUGIN__SNAPSHOT_REF_SOURCE_HPP_
