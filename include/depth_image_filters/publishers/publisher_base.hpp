//
// Created by antique on 3/23/26.
//

#ifndef DEPTH_IMAGE_FILTERS_PUBLISHER_BASE_HPP
#define DEPTH_IMAGE_FILTERS_PUBLISHER_BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "depth_image_filters/depth_image_filters_parameters.hpp"

namespace depth_image_filters {

class PublisherBase {
public:
  explicit PublisherBase(const std::string &name);
  virtual ~PublisherBase() = default;

  virtual void setup(rclcpp::Node *node, const Params &params) = 0;
  virtual void publish(const sensor_msgs::msg::Image::SharedPtr &depth_msg,
                       const sensor_msgs::msg::CameraInfo &camera_info,
                       const Params &params) = 0;

protected:
  rclcpp::Logger logger_;
};

} // namespace depth_image_filters

#endif // DEPTH_IMAGE_FILTERS_PUBLISHER_BASE_HPP
