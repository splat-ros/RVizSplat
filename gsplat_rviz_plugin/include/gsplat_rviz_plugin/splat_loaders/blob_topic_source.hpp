#ifndef GSPLAT_RVIZ_PLUGIN__BLOB_TOPIC_SOURCE_HPP_
#define GSPLAT_RVIZ_PLUGIN__BLOB_TOPIC_SOURCE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <gsplat_msgs/msg/splat_blob_chunk.hpp>

#include "gsplat_rviz_plugin/splat_loaders/blob_assembler.hpp"
#include "gsplat_rviz_plugin/splat_loaders/i_splat_source.hpp"

namespace gsplat_rviz_plugin
{

class BlobTopicSource : public ISplatSource
{
public:
  BlobTopicSource(
    rclcpp::Node::SharedPtr node,
    const std::string & topic);
  ~BlobTopicSource() override;

  void start(Callback cb) override;

private:
  void onChunk(gsplat_msgs::msg::SplatBlobChunk::ConstSharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  std::string topic_;
  Callback callback_;
  BlobAssembler assembler_;
  rclcpp::Subscription<gsplat_msgs::msg::SplatBlobChunk>::SharedPtr subscription_;
};

}  // namespace gsplat_rviz_plugin

#endif  // GSPLAT_RVIZ_PLUGIN__BLOB_TOPIC_SOURCE_HPP_
