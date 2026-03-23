//
// Created by antique on 3/23/26.
//

#include "depth_image_filters/publishers/laser_scan_publisher.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/core.hpp>
#include <cmath>
#include <limits>

namespace depth_image_filters {

LaserScanPublisher::LaserScanPublisher()
    : PublisherBase("laser_scan_publisher") {}

void LaserScanPublisher::setup(rclcpp::Node *node, const Params &/*params*/) {
  pub_ = node->create_publisher<sensor_msgs::msg::LaserScan>(
      "camera/depth/scan", rclcpp::SensorDataQoS());
}

void LaserScanPublisher::publish(
    const sensor_msgs::msg::Image::SharedPtr &depth_msg,
    const sensor_msgs::msg::CameraInfo &camera_info,
    const Params &params) {
  const auto &s = params.scan_publisher;

  camera_model_.fromCameraInfo(camera_info);

  cv_bridge::CvImageConstPtr cv_depth;
  try {
    cv_depth = cv_bridge::toCvShare(depth_msg);
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(logger_, "cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat &image = cv_depth->image;
  const int cols = image.cols;

  const double fx = camera_model_.fx();
  const double cx = camera_model_.cx();

  if (fx == 0.0) {
    RCLCPP_ERROR(logger_, "Invalid camera intrinsics: fx=0");
    return;
  }

  // Compute per-column min depth using OpenCV reduce (column-wise MIN)
  cv::Mat col_min;
  const bool is_float =
      (cv_depth->encoding == sensor_msgs::image_encodings::TYPE_32FC1);

  if (is_float) {
    cv::Mat sanitized;
    image.copyTo(sanitized);
    const float inf = std::numeric_limits<float>::infinity();
    sanitized.forEach<float>([&](float &px, const int[]) {
      if (!std::isfinite(px) || px <= 0.0f) {
        px = inf;
      }
    });
    cv::reduce(sanitized, col_min, 0, cv::REDUCE_MIN, CV_32F);
  } else {
    cv::Mat sanitized;
    image.copyTo(sanitized);
    sanitized.setTo(std::numeric_limits<uint16_t>::max(), sanitized == 0);
    cv::reduce(sanitized, col_min, 0, cv::REDUCE_MIN, CV_32F);
    col_min /= 1000.0f;
  }

  // Build LaserScan message
  auto scan = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan->header = depth_msg->header;
  if (!s.output_frame_id.empty()) {
    scan->header.frame_id = s.output_frame_id;
  }

  const float angle_max = static_cast<float>(std::atan2(cx, fx));
  const float angle_min =
      static_cast<float>(-std::atan2(cols - 1.0 - cx, fx));

  scan->angle_min = angle_min;
  scan->angle_max = angle_max;
  scan->angle_increment =
      (cols > 1) ? static_cast<float>((angle_max - angle_min) / (cols - 1))
                 : 0.0f;
  scan->time_increment = 0.0f;
  scan->scan_time = static_cast<float>(s.scan_time);
  scan->range_min = static_cast<float>(s.range_min);
  scan->range_max = static_cast<float>(s.range_max);

  scan->ranges.resize(cols);
  const float range_min_f = scan->range_min;
  const float range_max_f = scan->range_max;
  const float inf = std::numeric_limits<float>::infinity();

  const float *min_row = col_min.ptr<float>(0);
  for (int c = 0; c < cols; ++c) {
    const int scan_idx = cols - 1 - c;
    const float depth = min_row[c];

    const double theta = std::atan2(static_cast<double>(c) - cx, fx);
    const float range = static_cast<float>(depth / std::cos(theta));

    if (std::isfinite(range) && range >= range_min_f && range <= range_max_f) {
      scan->ranges[scan_idx] = range;
    } else {
      scan->ranges[scan_idx] = inf;
    }
  }

  pub_->publish(std::move(scan));
}

} // namespace depth_image_filters
