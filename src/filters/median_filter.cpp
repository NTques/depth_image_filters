//
// Created by antique on 3/23/26.
//

#include "depth_image_filters/filters/median_filter.hpp"

#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cmath>
#include <limits>

namespace depth_image_filters {
MedianFilter::MedianFilter() : DepthFilterBase("median_filter") {}

bool MedianFilter::apply(cv::Mat &image, const std::string &encoding,
                         const sensor_msgs::msg::CameraInfo & /*camera_info*/) {
  const auto &p = params_cache_.median_filter;
  int ksize = static_cast<int>(p.kernel_size);

  // kernel_size must be odd
  if (ksize % 2 == 0) {
    RCLCPP_ERROR(logger_, "kernel_size must be odd, got %d", ksize);
    return false;
  }

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
    // For 32FC1: replace NaN with 0 before median, restore NaN after
    // to prevent NaN from spreading through the median window
    const float nan = std::numeric_limits<float>::quiet_NaN();
    cv::Mat nan_mask(image.size(), CV_8UC1);

    for (int r = 0; r < image.rows; ++r) {
      for (int c = 0; c < image.cols; ++c) {
        if (!std::isfinite(image.at<float>(r, c))) {
          nan_mask.at<uint8_t>(r, c) = 255;
          image.at<float>(r, c) = 0.0f;
        } else {
          nan_mask.at<uint8_t>(r, c) = 0;
        }
      }
    }

    cv::medianBlur(image, image, ksize);

    // Restore NaN for originally invalid pixels
    image.setTo(nan, nan_mask);
  } else {
    // 16UC1: convert to float, apply median, convert back
    cv::Mat zero_mask;
    cv::compare(image, 0, zero_mask, cv::CMP_EQ);

    cv::Mat float_img;
    image.convertTo(float_img, CV_32F);
    float_img.setTo(0.0f, zero_mask);

    cv::medianBlur(float_img, float_img, ksize);

    float_img.convertTo(image, CV_16U);
    image.setTo(0, zero_mask);
  }

  return true;
}
} // namespace depth_image_filters
