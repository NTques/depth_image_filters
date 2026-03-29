//
// Created by antique on 3/23/26.
//

#include "depth_image_filters/filters/height_filter.hpp"

#include <sensor_msgs/image_encodings.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <limits>

namespace depth_image_filters {
HeightFilter::HeightFilter() : DepthFilterBase("height_filter") {}

void HeightFilter::initialize(
    const std::shared_ptr<ParamListener> &param_listener,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
  DepthFilterBase::initialize(param_listener);
  tf_buffer_ = std::move(tf_buffer);
}

bool HeightFilter::apply(cv::Mat &image, const std::string &encoding,
                         sensor_msgs::msg::CameraInfo &camera_info) {
  if (!tf_buffer_) {
    RCLCPP_ERROR(logger_, "TF buffer not set. Use initialize(param_listener, tf_buffer).");
    return false;
  }

  const auto &p = params_cache_.height_filter;

  if (p.min_height >= p.max_height) {
    RCLCPP_ERROR(logger_,
                 "min_height(%.3f) must be less than max_height(%.3f)",
                 p.min_height, p.max_height);
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

  camera_model_.fromCameraInfo(camera_info);

  if (camera_model_.fx() == 0.0 || camera_model_.fy() == 0.0) {
    RCLCPP_ERROR(logger_, "Invalid camera intrinsics: fx=%.1f, fy=%.1f",
                 camera_model_.fx(), camera_model_.fy());
    return false;
  }

  // Look up transform: camera_frame -> target_frame
  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_->lookupTransform(
        p.target_frame, camera_info.header.frame_id,
        camera_info.header.stamp);
  } catch (const tf2::TransformException &e) {
    RCLCPP_ERROR(logger_, "TF lookup failed: %s", e.what());
    return false;
  }

  // Extract rotation row for z (height) and translation z
  // height = R[2,0]*X_cam + R[2,1]*Y_cam + R[2,2]*Z_cam + t_z
  const auto &q = tf_stamped.transform.rotation;
  const auto &t = tf_stamped.transform.translation;

  // Rotation matrix third row from quaternion
  const double r20 = 2.0 * (q.x * q.z - q.w * q.y);
  const double r21 = 2.0 * (q.y * q.z + q.w * q.x);
  const double r22 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  const double tz = t.z;

  const double fx = camera_model_.fx();
  const double fy = camera_model_.fy();
  const double cx = camera_model_.cx();
  const double cy = camera_model_.cy();

  const double min_h = p.min_height;
  const double max_h = p.max_height;

  if (is_float) {
    const float nan = std::numeric_limits<float>::quiet_NaN();

    image.forEach<float>([&](float &px, const int pos[]) {
      const double d = static_cast<double>(px);
      if (!std::isfinite(d) || d <= 0.0) return;

      const double x_cam = (pos[1] - cx) * d / fx;
      const double y_cam = (pos[0] - cy) * d / fy;
      const double z_cam = d;

      const double height = r20 * x_cam + r21 * y_cam + r22 * z_cam + tz;

      if (height < min_h || height > max_h) {
        px = nan;
      }
    });
  } else {
    image.forEach<uint16_t>([&](uint16_t &px, const int pos[]) {
      if (px == 0) return;

      const double d = static_cast<double>(px) / 1000.0;  // mm -> m

      const double x_cam = (pos[1] - cx) * d / fx;
      const double y_cam = (pos[0] - cy) * d / fy;
      const double z_cam = d;

      const double height = r20 * x_cam + r21 * y_cam + r22 * z_cam + tz;

      if (height < min_h || height > max_h) {
        px = 0u;
      }
    });
  }

  return true;
}
} // namespace depth_image_filters
