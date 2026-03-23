#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <filesystem>
#include <cmath>

#include "depth_image_filters/filters/range_filter.hpp"
#include "depth_image_filters/filters/crop_filter.hpp"
#include "depth_image_filters/filters/height_filter.hpp"
#include "depth_image_filters/filters/speckle_filter.hpp"
#include "depth_image_filters/filters/median_filter.hpp"

#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <random>

static const sensor_msgs::msg::CameraInfo kEmptyCameraInfo;

namespace fs = std::filesystem;

static const std::string kOutputDir = "visual_results";

static cv::Mat colorize(const cv::Mat& depth, double min_val, double max_val) {
  cv::Mat normalized;
  if (depth.type() == CV_32FC1) {
    cv::Mat valid_mask = cv::Mat::zeros(depth.size(), CV_8UC1);
    depth.forEach<float>([&](const float& px, const int pos[]) {
      if (std::isfinite(px)) {
        valid_mask.at<uint8_t>(pos[0], pos[1]) = 255;
      }
    });
    cv::Mat clamped;
    depth.convertTo(clamped, CV_64F);
    clamped = (clamped - min_val) / (max_val - min_val) * 255.0;
    clamped.convertTo(normalized, CV_8UC1);
    normalized.setTo(0, ~valid_mask);
  } else {
    depth.convertTo(normalized, CV_8UC1, 255.0 / max_val);
  }
  cv::Mat colored;
  cv::applyColorMap(normalized, colored, cv::COLORMAP_TURBO);

  // Show invalid pixels (NaN for float, 0 for uint16) as gray
  cv::Mat invalid_mask;
  if (depth.type() == CV_32FC1) {
    invalid_mask = cv::Mat::zeros(depth.size(), CV_8UC1);
    depth.forEach<float>([&](const float& px, const int pos[]) {
      if (!std::isfinite(px)) {
        invalid_mask.at<uint8_t>(pos[0], pos[1]) = 255;
      }
    });
  } else {
    cv::compare(depth, 0, invalid_mask, cv::CMP_EQ);
  }
  cv::Mat gray(depth.size(), CV_8UC3, cv::Scalar(40, 40, 40));
  gray.copyTo(colored, invalid_mask);

  return colored;
}

static cv::Mat enlargeForVisibility(const cv::Mat& img, int target_width = 480) {
  int scale = std::max(1, target_width / img.cols);
  cv::Mat enlarged;
  cv::resize(img, enlarged, cv::Size(img.cols * scale, img.rows * scale),
             0, 0, cv::INTER_NEAREST);
  return enlarged;
}

static cv::Mat makeComparisonImage(const cv::Mat& before, const cv::Mat& after,
                                   const std::string& before_label,
                                   const std::string& after_label) {
  int pad = 40;
  int total_w = before.cols + after.cols + pad * 3;
  int total_h = std::max(before.rows, after.rows) + pad * 2;
  cv::Mat canvas(total_h, total_w, CV_8UC3, cv::Scalar(30, 30, 30));

  before.copyTo(canvas(cv::Rect(pad, pad, before.cols, before.rows)));
  after.copyTo(canvas(cv::Rect(pad * 2 + before.cols, pad, after.cols, after.rows)));

  cv::putText(canvas, before_label, cv::Point(pad, pad - 10),
              cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
  cv::putText(canvas, after_label, cv::Point(pad * 2 + before.cols, pad - 10),
              cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);

  return canvas;
}

class VisualOutputTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_visual_output");
    fs::create_directories(kOutputDir);
  }

  void TearDown() override { rclcpp::shutdown(); }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(VisualOutputTest, RangeFilter_Float32) {
  // Create gradient depth image (0.0m ~ 10.0m)
  const int rows = 120, cols = 160;
  cv::Mat depth(rows, cols, CV_32FC1);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      depth.at<float>(r, c) = static_cast<float>(c) / cols * 10.0f;
    }
  }

  cv::Mat before = depth.clone();

  node_->declare_parameter("range_filter.min_distance", 2.0);
  node_->declare_parameter("range_filter.max_distance", 7.0);

  auto param_listener =
      std::make_shared<depth_image_filters::ParamListener>(node_);
  depth_image_filters::RangeFilter filter;
  filter.initialize(param_listener);

  std::string encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  ASSERT_TRUE(filter.apply(depth, encoding, kEmptyCameraInfo));

  cv::Mat before_vis = enlargeForVisibility(colorize(before, 0.0, 10.0));
  cv::Mat after_vis = enlargeForVisibility(colorize(depth, 0.0, 10.0));
  cv::Mat comparison =
      makeComparisonImage(before_vis, after_vis, "Before (0~10m)", "After (range: 2~7m)");

  std::string path = kOutputDir + "/range_filter_float32.png";
  cv::imwrite(path, comparison);
  std::cout << "  -> Saved: " << fs::absolute(path) << std::endl;
}

TEST_F(VisualOutputTest, RangeFilter_Uint16) {
  // Create gradient depth image (0mm ~ 5000mm)
  const int rows = 120, cols = 160;
  cv::Mat depth(rows, cols, CV_16UC1);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      depth.at<uint16_t>(r, c) = static_cast<uint16_t>(
          static_cast<float>(c) / cols * 5000);
    }
  }

  cv::Mat before = depth.clone();

  node_->declare_parameter("range_filter.min_distance", 1.0);
  node_->declare_parameter("range_filter.max_distance", 3.5);

  auto param_listener =
      std::make_shared<depth_image_filters::ParamListener>(node_);
  depth_image_filters::RangeFilter filter;
  filter.initialize(param_listener);

  std::string encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  ASSERT_TRUE(filter.apply(depth, encoding, kEmptyCameraInfo));

  cv::Mat before_vis = enlargeForVisibility(colorize(before, 0, 5000));
  cv::Mat after_vis = enlargeForVisibility(colorize(depth, 0, 5000));
  cv::Mat comparison = makeComparisonImage(before_vis, after_vis,
                                           "Before (0~5000mm)",
                                           "After (range: 1000~3500mm)");

  std::string path = kOutputDir + "/range_filter_uint16.png";
  cv::imwrite(path, comparison);
  std::cout << "  -> Saved: " << fs::absolute(path) << std::endl;
}

TEST_F(VisualOutputTest, CropFilter_Pixel) {
  // Create checkerboard-like depth image
  const int rows = 120, cols = 160;
  cv::Mat depth(rows, cols, CV_16UC1);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      float dist_r = std::abs(r - rows / 2.0f) / (rows / 2.0f);
      float dist_c = std::abs(c - cols / 2.0f) / (cols / 2.0f);
      float dist = std::sqrt(dist_r * dist_r + dist_c * dist_c);
      depth.at<uint16_t>(r, c) = static_cast<uint16_t>(dist * 5000);
    }
  }

  cv::Mat before = depth.clone();

  node_->declare_parameter("crop_filter.margin_type", std::string("pixel"));
  node_->declare_parameter("crop_filter.top", 20.0);
  node_->declare_parameter("crop_filter.bottom", 20.0);
  node_->declare_parameter("crop_filter.left", 30.0);
  node_->declare_parameter("crop_filter.right", 30.0);

  auto param_listener =
      std::make_shared<depth_image_filters::ParamListener>(node_);
  depth_image_filters::CropFilter filter;
  filter.initialize(param_listener);

  std::string encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  ASSERT_TRUE(filter.apply(depth, encoding, kEmptyCameraInfo));

  cv::Mat before_vis = enlargeForVisibility(colorize(before, 0, 5000));
  cv::Mat after_vis = enlargeForVisibility(colorize(depth, 0, 5000));
  cv::Mat comparison = makeComparisonImage(
      before_vis, after_vis, "Before (120x160)",
      "After (crop: t=20 b=20 l=30 r=30)");

  std::string path = kOutputDir + "/crop_filter_pixel.png";
  cv::imwrite(path, comparison);
  std::cout << "  -> Saved: " << fs::absolute(path) << std::endl;
}

TEST_F(VisualOutputTest, CropFilter_Ratio) {
  const int rows = 120, cols = 160;
  cv::Mat depth(rows, cols, CV_16UC1);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      float dist_r = std::abs(r - rows / 2.0f) / (rows / 2.0f);
      float dist_c = std::abs(c - cols / 2.0f) / (cols / 2.0f);
      float dist = std::sqrt(dist_r * dist_r + dist_c * dist_c);
      depth.at<uint16_t>(r, c) = static_cast<uint16_t>(dist * 5000);
    }
  }

  cv::Mat before = depth.clone();

  node_->declare_parameter("crop_filter.margin_type", std::string("ratio"));
  node_->declare_parameter("crop_filter.top", 0.15);
  node_->declare_parameter("crop_filter.bottom", 0.15);
  node_->declare_parameter("crop_filter.left", 0.2);
  node_->declare_parameter("crop_filter.right", 0.2);

  auto param_listener =
      std::make_shared<depth_image_filters::ParamListener>(node_);
  depth_image_filters::CropFilter filter;
  filter.initialize(param_listener);

  std::string encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  ASSERT_TRUE(filter.apply(depth, encoding, kEmptyCameraInfo));

  cv::Mat before_vis = enlargeForVisibility(colorize(before, 0, 5000));
  cv::Mat after_vis = enlargeForVisibility(colorize(depth, 0, 5000));
  cv::Mat comparison = makeComparisonImage(
      before_vis, after_vis, "Before (120x160)",
      "After (crop: ratio 15%/15%/20%/20%)");

  std::string path = kOutputDir + "/crop_filter_ratio.png";
  cv::imwrite(path, comparison);
  std::cout << "  -> Saved: " << fs::absolute(path) << std::endl;
}

TEST_F(VisualOutputTest, HeightFilter) {
  // Camera at 1.0m height, looking forward horizontally
  // height range [0.0, 1.5] -> upper part of image (high objects) gets removed
  const int rows = 120, cols = 160;
  const double fx = 200.0, fy = 200.0;
  const double cx = 80.0, cy = 60.0;

  // Radial depth pattern
  cv::Mat depth(rows, cols, CV_32FC1);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      float dist_r = std::abs(r - rows / 2.0f) / (rows / 2.0f);
      float dist_c = std::abs(c - cols / 2.0f) / (cols / 2.0f);
      depth.at<float>(r, c) = 1.0f + std::sqrt(dist_r * dist_r + dist_c * dist_c) * 4.0f;
    }
  }

  cv::Mat before = depth.clone();

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = rclcpp::Time(0);
  tf.header.frame_id = "base_link";
  tf.child_frame_id = "camera_optical";
  tf.transform.translation.z = 1.0;
  tf2::Quaternion q;
  q.setRPY(-M_PI / 2.0, 0.0, -M_PI / 2.0);
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  tf_buffer->setTransform(tf, "test", true);

  node_->declare_parameter("height_filter.min_height", 0.0);
  node_->declare_parameter("height_filter.max_height", 1.5);
  node_->declare_parameter("height_filter.target_frame", std::string("base_link"));

  auto param_listener =
      std::make_shared<depth_image_filters::ParamListener>(node_);
  depth_image_filters::HeightFilter filter;
  filter.initialize(param_listener, tf_buffer);

  sensor_msgs::msg::CameraInfo cam_info;
  cam_info.header.stamp = rclcpp::Time(0);
  cam_info.header.frame_id = "camera_optical";
  cam_info.width = cols;
  cam_info.height = rows;
  cam_info.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
  cam_info.p = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0};
  cam_info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  cam_info.distortion_model = "plumb_bob";
  cam_info.d = {0, 0, 0, 0, 0};

  std::string encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  ASSERT_TRUE(filter.apply(depth, encoding, cam_info));

  cv::Mat before_vis = enlargeForVisibility(colorize(before, 0.0, 5.0));
  cv::Mat after_vis = enlargeForVisibility(colorize(depth, 0.0, 5.0));
  cv::Mat comparison = makeComparisonImage(
      before_vis, after_vis, "Before (cam@1.0m)",
      "After (height: 0~1.5m)");

  std::string hpath = kOutputDir + "/height_filter.png";
  cv::imwrite(hpath, comparison);
  std::cout << "  -> Saved: " << fs::absolute(hpath) << std::endl;
}

TEST_F(VisualOutputTest, SpeckleFilter) {
  const int rows = 120, cols = 160;

  // Uniform depth at 2000mm with scattered small blob noise
  cv::Mat depth(rows, cols, CV_16UC1, cv::Scalar(2000));

  // Add small blob speckles (3x3 clusters) at random positions
  std::mt19937 rng(42);
  std::uniform_int_distribution<int> row_dist(2, rows - 3);
  std::uniform_int_distribution<int> col_dist(2, cols - 3);
  for (int i = 0; i < 30; ++i) {
    int r = row_dist(rng);
    int c = col_dist(rng);
    // 3x3 speckle blob
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        depth.at<uint16_t>(r + dr, c + dc) = 5000;
      }
    }
  }

  cv::Mat before = depth.clone();

  node_->declare_parameter("speckle_filter.max_speckle_size", 50);
  node_->declare_parameter("speckle_filter.max_diff", 0.2);

  auto param_listener =
      std::make_shared<depth_image_filters::ParamListener>(node_);
  depth_image_filters::SpeckleFilter filter;
  filter.initialize(param_listener);

  std::string encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  ASSERT_TRUE(filter.apply(depth, encoding, kEmptyCameraInfo));

  cv::Mat before_vis = enlargeForVisibility(colorize(before, 0, 6000));
  cv::Mat after_vis = enlargeForVisibility(colorize(depth, 0, 6000));
  cv::Mat comparison = makeComparisonImage(
      before_vis, after_vis, "Before (with speckles)",
      "After (speckle removed)");

  std::string spath = kOutputDir + "/speckle_filter.png";
  cv::imwrite(spath, comparison);
  std::cout << "  -> Saved: " << fs::absolute(spath) << std::endl;
}

TEST_F(VisualOutputTest, MedianFilter) {
  const int rows = 120, cols = 160;

  // Smooth gradient with salt-and-pepper noise
  cv::Mat depth(rows, cols, CV_16UC1);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      depth.at<uint16_t>(r, c) = static_cast<uint16_t>(1000 + c * 20);
    }
  }

  // Add salt-and-pepper noise
  std::mt19937 rng(123);
  std::uniform_int_distribution<int> row_dist(0, rows - 1);
  std::uniform_int_distribution<int> col_dist(0, cols - 1);
  for (int i = 0; i < 300; ++i) {
    int r = row_dist(rng);
    int c = col_dist(rng);
    depth.at<uint16_t>(r, c) = (i % 2 == 0) ? 8000 : 200;  // salt or pepper
  }

  cv::Mat before = depth.clone();

  node_->declare_parameter("median_filter.kernel_size", 5);

  auto param_listener =
      std::make_shared<depth_image_filters::ParamListener>(node_);
  depth_image_filters::MedianFilter filter;
  filter.initialize(param_listener);

  std::string encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  ASSERT_TRUE(filter.apply(depth, encoding, kEmptyCameraInfo));

  cv::Mat before_vis = enlargeForVisibility(colorize(before, 0, 5000));
  cv::Mat after_vis = enlargeForVisibility(colorize(depth, 0, 5000));
  cv::Mat comparison = makeComparisonImage(
      before_vis, after_vis, "Before (salt & pepper)",
      "After (median k=5)");

  std::string mpath = kOutputDir + "/median_filter.png";
  cv::imwrite(mpath, comparison);
  std::cout << "  -> Saved: " << fs::absolute(mpath) << std::endl;
}
