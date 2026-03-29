//
// Created by antique on 3/22/26.
//

#ifndef DEPTH_IMAGE_FILTERS_CROP_FILTER_HPP
#define DEPTH_IMAGE_FILTERS_CROP_FILTER_HPP

#include "depth_image_filters/filters/depth_filter_base.hpp"
#include <opencv2/core.hpp>

namespace depth_image_filters {
class CropFilter : public DepthFilterBase {
public:
  explicit CropFilter();
  bool apply(cv::Mat &image, const std::string &encoding,
             sensor_msgs::msg::CameraInfo &camera_info) override;

private:
  cv::Rect computeCropRect(int img_width, int img_height) const;
  bool validateRect(const cv::Rect &rect, int img_width, int img_height) const;
};
} // namespace depth_image_filters
#endif // DEPTH_IMAGE_FILTERS_CROP_FILTER_HPP
