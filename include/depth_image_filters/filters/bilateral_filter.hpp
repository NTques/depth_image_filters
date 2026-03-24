#ifndef DEPTH_IMAGE_FILTERS_BILATERAL_FILTER_HPP
#define DEPTH_IMAGE_FILTERS_BILATERAL_FILTER_HPP

#include "depth_image_filters/filters/depth_filter_base.hpp"

namespace depth_image_filters {
class BilateralFilter : public DepthFilterBase {
public:
  explicit BilateralFilter();
  bool apply(cv::Mat &image, const std::string &encoding,
             const sensor_msgs::msg::CameraInfo &camera_info) override;
};
} // namespace depth_image_filters

#endif // DEPTH_IMAGE_FILTERS_BILATERAL_FILTER_HPP
