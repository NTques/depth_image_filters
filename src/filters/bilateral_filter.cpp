#include "depth_image_filters/filters/bilateral_filter.hpp"

#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cmath>
#include <limits>

namespace depth_image_filters {

BilateralFilter::BilateralFilter() : DepthFilterBase("bilateral_filter") {}

bool BilateralFilter::apply(cv::Mat &image, const std::string &encoding,
                            const sensor_msgs::msg::CameraInfo & /*camera_info*/) {
  const auto &p = params_cache_.bilateral_filter;
  const int d = static_cast<int>(p.diameter);
  const double sigma_color = p.sigma_color;
  const double sigma_space = p.sigma_space;

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

  cv::Mat filtered;

  if (is_float) {
    // Replace NaN/inf with 0 before filtering, restore after
    const float nan = std::numeric_limits<float>::quiet_NaN();
    cv::Mat nan_mask(image.size(), CV_8UC1);

    for (int r = 0; r < image.rows; ++r) {
      auto *row = image.ptr<float>(r);
      auto *mask_row = nan_mask.ptr<uint8_t>(r);
      for (int c = 0; c < image.cols; ++c) {
        if (!std::isfinite(row[c]) || row[c] <= 0.0f) {
          mask_row[c] = 255;
          row[c] = 0.0f;
        } else {
          mask_row[c] = 0;
        }
      }
    }

    cv::bilateralFilter(image, filtered, d, sigma_color, sigma_space);
    filtered.setTo(nan, nan_mask);
  } else {
    // 16UC1: convert to float, filter, convert back
    cv::Mat zero_mask;
    cv::compare(image, 0, zero_mask, cv::CMP_EQ);

    cv::Mat float_img;
    image.convertTo(float_img, CV_32F);
    float_img.setTo(0.0f, zero_mask);

    cv::Mat float_filtered;
    cv::bilateralFilter(float_img, float_filtered, d, sigma_color, sigma_space);

    float_filtered.convertTo(filtered, CV_16U);
    filtered.setTo(0, zero_mask);
  }

  image = filtered;
  return true;
}

} // namespace depth_image_filters
