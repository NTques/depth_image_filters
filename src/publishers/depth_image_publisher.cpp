//
// Created by antique on 3/23/26.
//

#include "depth_image_filters/publishers/depth_image_publisher.hpp"

namespace depth_image_filters {

DepthImagePublisher::DepthImagePublisher()
    : PublisherBase("depth_image_publisher") {}

void DepthImagePublisher::setup(rclcpp::Node *node, const Params &params) {
  pub_ = image_transport::create_publisher(node, params.depth_image_publisher.image_topic);
  camera_info_pub_ = node->create_publisher<sensor_msgs::msg::CameraInfo>(
      params.depth_image_publisher.camera_info_topic, rclcpp::SensorDataQoS());
}

void DepthImagePublisher::publish(
    const sensor_msgs::msg::Image::SharedPtr &depth_msg,
    const sensor_msgs::msg::CameraInfo &camera_info,
    const Params &/*params*/) {
  pub_.publish(depth_msg);
  camera_info_pub_->publish(camera_info);
}

} // namespace depth_image_filters
