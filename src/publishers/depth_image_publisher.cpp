//
// Created by antique on 3/23/26.
//

#include "depth_image_filters/publishers/depth_image_publisher.hpp"

namespace depth_image_filters {

DepthImagePublisher::DepthImagePublisher()
    : PublisherBase("depth_image_publisher") {}

void DepthImagePublisher::setup(rclcpp::Node *node, const Params &/*params*/) {
  pub_ = image_transport::create_publisher(node, "camera/depth/filtered_image");
}

void DepthImagePublisher::publish(
    const sensor_msgs::msg::Image::SharedPtr &depth_msg,
    const sensor_msgs::msg::CameraInfo &/*camera_info*/,
    const Params &/*params*/) {
  pub_.publish(depth_msg);
}

} // namespace depth_image_filters
