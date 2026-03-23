//
// Created by antique on 3/22/26.
//

#ifndef DEPTH_IMAGE_FILTERS_FILTER_CHAIN_HPP
#define DEPTH_IMAGE_FILTERS_FILTER_CHAIN_HPP

#include "depth_image_filters/filters/depth_filter_base.hpp"

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

namespace depth_image_filters {
class FilterChain {
public:
  void addFilter(const DepthFilterBase::SharedPtr &filter);
  bool apply(const sensor_msgs::msg::Image::ConstSharedPtr &input,
             const sensor_msgs::msg::CameraInfo &camera_info,
             sensor_msgs::msg::Image::SharedPtr &output);
  void clear();

private:
  std::vector<DepthFilterBase::SharedPtr> filters_;
  rclcpp::Logger logger_{rclcpp::get_logger("filter_chain")};
};
} // namespace depth_image_filters

#endif // DEPTH_IMAGE_FILTERS_FILTER_CHAIN_HPP
