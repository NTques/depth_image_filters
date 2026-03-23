//
// Created by antique on 3/23/26.
//

#ifndef DEPTH_IMAGE_FILTERS_LASER_SCAN_PUBLISHER_HPP
#define DEPTH_IMAGE_FILTERS_LASER_SCAN_PUBLISHER_HPP

#include "depth_image_filters/publishers/publisher_base.hpp"

#include <image_geometry/pinhole_camera_model.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace depth_image_filters {

class LaserScanPublisher : public PublisherBase {
public:
  LaserScanPublisher();

  void setup(rclcpp::Node *node, const Params &params) override;
  void publish(const sensor_msgs::msg::Image::SharedPtr &depth_msg,
               const sensor_msgs::msg::CameraInfo &camera_info,
               const Params &params) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  image_geometry::PinholeCameraModel camera_model_;
};

} // namespace depth_image_filters

#endif // DEPTH_IMAGE_FILTERS_LASER_SCAN_PUBLISHER_HPP
