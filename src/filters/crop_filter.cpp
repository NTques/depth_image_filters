//
// Created by antique on 3/22/26.
//

#include "depth_image_filters/filters/crop_filter.hpp"

namespace depth_image_filters {
CropFilter::CropFilter() : DepthFilterBase("crop_filter") {}

cv::Rect CropFilter::computeCropRect(int img_width, int img_height) const {
  const auto &p = params_cache_.crop_filter;

  int top, bottom, left, right;

  if (p.margin_type == "ratio") {
    top = static_cast<int>(p.top * img_height);
    bottom = static_cast<int>(p.bottom * img_height);
    left = static_cast<int>(p.left * img_width);
    right = static_cast<int>(p.right * img_width);
  } else {
    top = static_cast<int>(p.top);
    bottom = static_cast<int>(p.bottom);
    left = static_cast<int>(p.left);
    right = static_cast<int>(p.right);
  }

  return cv::Rect(left, top, img_width - left - right,
                  img_height - top - bottom);
}

bool CropFilter::validateRect(const cv::Rect &rect, int img_width,
                              int img_height) const {
  if (rect.width <= 0 || rect.height <= 0) {
    RCLCPP_ERROR(logger_, "Margin too large — resulting size: %dx%d",
                 rect.width, rect.height);
    return false;
  }
  if (rect.x < 0 || rect.y < 0 || rect.x + rect.width > img_width ||
      rect.y + rect.height > img_height) {
    RCLCPP_ERROR(
        logger_, "Crop rect out of bounds: (%d,%d) %dx%d vs image %dx%d",
        rect.x, rect.y, rect.width, rect.height, img_width, img_height);
    return false;
  }
  return true;
}

bool CropFilter::apply(cv::Mat &image, const std::string &encoding,
                       sensor_msgs::msg::CameraInfo &camera_info) {
  const cv::Rect crop_rect = computeCropRect(image.cols, image.rows);

  if (!validateRect(crop_rect, image.cols, image.rows)) {
    return false;
  }

  image = image(crop_rect).clone();

  // Update camera_info to reflect cropped image dimensions
  const double cx = camera_info.k[2] - crop_rect.x;
  const double cy = camera_info.k[5] - crop_rect.y;

  // K matrix: [fx, 0, cx; 0, fy, cy; 0, 0, 1]
  camera_info.k[2] = cx;
  camera_info.k[5] = cy;

  // P matrix: [fx, 0, cx, Tx; 0, fy, cy, Ty; 0, 0, 1, 0]
  camera_info.p[2] = cx;
  camera_info.p[6] = cy;

  camera_info.width = crop_rect.width;
  camera_info.height = crop_rect.height;

  // Set ROI to reflect the crop within the original full resolution image
  camera_info.roi.x_offset += crop_rect.x;
  camera_info.roi.y_offset += crop_rect.y;
  camera_info.roi.width = crop_rect.width;
  camera_info.roi.height = crop_rect.height;

  return true;
}
} // namespace depth_image_filters
