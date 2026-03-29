//
// Created by antique on 3/23/26.
//

#include "depth_image_filters/filters/speckle_filter.hpp"

#include <opencv2/calib3d.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cmath>
#include <limits>

namespace depth_image_filters {
SpeckleFilter::SpeckleFilter() : DepthFilterBase("speckle_filter") {}

bool SpeckleFilter::apply(cv::Mat &image, const std::string &encoding,
                          sensor_msgs::msg::CameraInfo & /*camera_info*/) {
  const auto &p = params_cache_.speckle_filter;

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

  if (is_float) {
    // filterSpeckles works on 16SC1, so convert float(m) -> int16(mm)
    // and apply, then map removed pixels back to NaN
    const float nan = std::numeric_limits<float>::quiet_NaN();
    constexpr int16_t kInvalid = 0;

    cv::Mat depth_mm(image.size(), CV_16SC1);
    for (int r = 0; r < image.rows; ++r) {
      for (int c = 0; c < image.cols; ++c) {
        float v = image.at<float>(r, c);
        if (std::isfinite(v) && v > 0.0f) {
          depth_mm.at<int16_t>(r, c) =
              static_cast<int16_t>(std::clamp(v * 1000.0f, -32767.0f, 32767.0f));
        } else {
          depth_mm.at<int16_t>(r, c) = kInvalid;
        }
      }
    }

    double max_diff_mm = p.max_diff * 1000.0;
    cv::filterSpeckles(depth_mm, kInvalid, static_cast<int>(p.max_speckle_size),
                       max_diff_mm);

    // Mark newly zeroed pixels as NaN in the original
    for (int r = 0; r < image.rows; ++r) {
      for (int c = 0; c < image.cols; ++c) {
        if (depth_mm.at<int16_t>(r, c) == kInvalid &&
            std::isfinite(image.at<float>(r, c))) {
          image.at<float>(r, c) = nan;
        }
      }
    }
  } else {
    // 16UC1: filterSpeckles needs 16SC1
    // Scale by 0.5 to fit uint16 (0-65535) into int16 (0-32767) without clipping
    cv::Mat signed_depth;
    image.convertTo(signed_depth, CV_16SC1, 0.5);

    constexpr int16_t kInvalid = 0;
    double max_diff_scaled = p.max_diff * 1000.0 * 0.5;
    cv::filterSpeckles(signed_depth, kInvalid, static_cast<int>(p.max_speckle_size),
                       max_diff_scaled);

    // Map removed pixels back to 0 in original
    for (int r = 0; r < image.rows; ++r) {
      for (int c = 0; c < image.cols; ++c) {
        if (signed_depth.at<int16_t>(r, c) == kInvalid) {
          image.at<uint16_t>(r, c) = 0;
        }
      }
    }
  }

  return true;
}
} // namespace depth_image_filters
