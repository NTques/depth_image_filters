//
// Created by antique on 3/23/26.
//

#ifndef DEPTH_IMAGE_FILTERS_DEPTH_IMAGE_PUBLISHER_HPP
#define DEPTH_IMAGE_FILTERS_DEPTH_IMAGE_PUBLISHER_HPP

#include "depth_image_filters/publishers/publisher_base.hpp"

#include <image_transport/image_transport.hpp>

namespace depth_image_filters {

class DepthImagePublisher : public PublisherBase {
public:
  DepthImagePublisher();

  void setup(rclcpp::Node *node, const Params &params) override;
  void publish(const sensor_msgs::msg::Image::SharedPtr &depth_msg,
               const sensor_msgs::msg::CameraInfo &camera_info,
               const Params &params) override;

private:
  image_transport::Publisher pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
};

} // namespace depth_image_filters

#endif // DEPTH_IMAGE_FILTERS_DEPTH_IMAGE_PUBLISHER_HPP
