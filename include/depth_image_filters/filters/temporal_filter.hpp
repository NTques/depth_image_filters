#ifndef DEPTH_IMAGE_FILTERS_TEMPORAL_FILTER_HPP
#define DEPTH_IMAGE_FILTERS_TEMPORAL_FILTER_HPP

#include "depth_image_filters/filters/depth_filter_base.hpp"
#include <deque>

namespace depth_image_filters {
class TemporalFilter : public DepthFilterBase {
public:
  explicit TemporalFilter();
  bool apply(cv::Mat &image, const std::string &encoding,
             const sensor_msgs::msg::CameraInfo &camera_info) override;

private:
  std::deque<cv::Mat> buffer_;
  int prev_buffer_size_ = 0;
};
} // namespace depth_image_filters

#endif // DEPTH_IMAGE_FILTERS_TEMPORAL_FILTER_HPP
