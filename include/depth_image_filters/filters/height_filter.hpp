//
// Created by antique on 3/23/26.
//

#ifndef DEPTH_IMAGE_FILTERS_HEIGHT_FILTER_HPP
#define DEPTH_IMAGE_FILTERS_HEIGHT_FILTER_HPP

#include "depth_image_filters/filters/depth_filter_base.hpp"

#include <image_geometry/pinhole_camera_model.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace depth_image_filters {
class HeightFilter : public DepthFilterBase {
public:
  explicit HeightFilter();

  void initialize(const std::shared_ptr<ParamListener> &param_listener,
                  std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  bool apply(cv::Mat &image, const std::string &encoding,
             sensor_msgs::msg::CameraInfo &camera_info) override;

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  image_geometry::PinholeCameraModel camera_model_;
};
} // namespace depth_image_filters

#endif // DEPTH_IMAGE_FILTERS_HEIGHT_FILTER_HPP
