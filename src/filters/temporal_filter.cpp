#include "depth_image_filters/filters/temporal_filter.hpp"

#include <sensor_msgs/image_encodings.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace depth_image_filters {

TemporalFilter::TemporalFilter() : DepthFilterBase("temporal_filter") {}

bool TemporalFilter::apply(cv::Mat &image, const std::string &encoding,
                           sensor_msgs::msg::CameraInfo & /*camera_info*/) {
  const auto &p = params_cache_.temporal_filter;
  const int buffer_size = static_cast<int>(p.buffer_size);

  const bool is_float =
      (encoding == sensor_msgs::image_encodings::TYPE_32FC1);
  const bool is_uint16 =
      (encoding == sensor_msgs::image_encodings::TYPE_16UC1);

  if (!is_float && !is_uint16) {
    RCLCPP_ERROR(logger_,
                 "Unsupported encoding: %s (expected 32FC1 or 16UC1)",
                 encoding.c_str());
    return false;
  }

  // Reset buffer if size param changed or image dimensions changed
  if (buffer_size != prev_buffer_size_ ||
      (!buffer_.empty() && (buffer_.front().size() != image.size() ||
                            buffer_.front().type() != image.type()))) {
    buffer_.clear();
    prev_buffer_size_ = buffer_size;
  }

  buffer_.push_back(image.clone());
  while (static_cast<int>(buffer_.size()) > buffer_size) {
    buffer_.pop_front();
  }

  if (buffer_.size() < 2) {
    return true;
  }

  const int rows = image.rows;
  const int cols = image.cols;
  const int n = static_cast<int>(buffer_.size());
  const int mid = n / 2;

  if (is_float) {
    std::vector<float> vals;
    vals.reserve(n);

    for (int r = 0; r < rows; ++r) {
      auto *out = image.ptr<float>(r);
      for (int c = 0; c < cols; ++c) {
        vals.clear();
        for (int i = 0; i < n; ++i) {
          float v = buffer_[i].at<float>(r, c);
          if (std::isfinite(v) && v > 0.0f) {
            vals.push_back(v);
          }
        }
        if (!vals.empty()) {
          std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
          out[c] = vals[vals.size() / 2];
        }
      }
    }
  } else {
    std::vector<uint16_t> vals;
    vals.reserve(n);

    for (int r = 0; r < rows; ++r) {
      auto *out = image.ptr<uint16_t>(r);
      for (int c = 0; c < cols; ++c) {
        vals.clear();
        for (int i = 0; i < n; ++i) {
          uint16_t v = buffer_[i].at<uint16_t>(r, c);
          if (v > 0) {
            vals.push_back(v);
          }
        }
        if (!vals.empty()) {
          std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
          out[c] = vals[vals.size() / 2];
        }
      }
    }
  }

  return true;
}

} // namespace depth_image_filters
