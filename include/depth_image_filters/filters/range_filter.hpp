//
// Created by antique on 3/22/26.
//

#ifndef DEPTH_IMAGE_FILTERS_RANGE_FILTER_HPP
#define DEPTH_IMAGE_FILTERS_RANGE_FILTER_HPP

#include "depth_image_filters/filters/depth_filter_base.hpp"

namespace depth_image_filters {
class RangeFilter : public DepthFilterBase {
public:
  explicit RangeFilter();
  bool apply(cv::Mat &image, const std::string &encoding,
             sensor_msgs::msg::CameraInfo &camera_info) override;
};
} // namespace depth_image_filters

#endif // DEPTH_IMAGE_FILTERS_RANGE_FILTER_HPP
