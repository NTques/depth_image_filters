//
// Created by antique on 3/23/26.
//

#ifndef DEPTH_IMAGE_FILTERS_SPECKLE_FILTER_HPP
#define DEPTH_IMAGE_FILTERS_SPECKLE_FILTER_HPP

#include "depth_image_filters/filters/depth_filter_base.hpp"

namespace depth_image_filters {
class SpeckleFilter : public DepthFilterBase {
public:
  explicit SpeckleFilter();
  bool apply(cv::Mat &image, const std::string &encoding,
             sensor_msgs::msg::CameraInfo &camera_info) override;
};
} // namespace depth_image_filters

#endif // DEPTH_IMAGE_FILTERS_SPECKLE_FILTER_HPP
