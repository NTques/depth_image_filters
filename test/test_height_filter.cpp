#include <gtest/gtest.h>
#include <rcutils/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <limits>

#include "depth_image_filters/filters/height_filter.hpp"

class HeightFilterTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_height_filter");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  }

  void TearDown() override { rclcpp::shutdown(); }

  void initFilter(double min_height, double max_height,
                  const std::string &target_frame) {
    node_->declare_parameter("height_filter.min_height", min_height);
    node_->declare_parameter("height_filter.max_height", max_height);
    node_->declare_parameter("height_filter.target_frame", target_frame);

    auto param_listener =
        std::make_shared<depth_image_filters::ParamListener>(node_);
    filter_.initialize(param_listener, tf_buffer_);
  }

  // Set a static TF: camera is at given height, looking forward (optical frame: z-forward, x-right, y-down)
  // target_frame z-up convention: camera z-forward maps to target x-forward,
  // camera y-down maps to target z-up (negated)
  void setTransform(const std::string &target_frame,
                    const std::string &camera_frame,
                    double camera_height,
                    double pitch_rad = 0.0) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = rclcpp::Time(0);
    tf.header.frame_id = target_frame;
    tf.child_frame_id = camera_frame;
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = camera_height;

    // Optical frame convention: z-forward, x-right, y-down
    // To map to base_link (x-forward, y-left, z-up):
    // R = Ry(pitch) * R_optical_to_base
    // R_optical_to_base rotates optical axes to ROS axes
    tf2::Quaternion q_optical;
    // Rotation from optical frame to base frame:
    // x_base = z_optical, y_base = -x_optical, z_base = -y_optical
    // This is a rotation: roll=-pi/2, then yaw=-pi/2
    q_optical.setRPY(-M_PI / 2.0, 0.0, -M_PI / 2.0);

    tf2::Quaternion q_pitch;
    q_pitch.setRPY(0.0, pitch_rad, 0.0);

    tf2::Quaternion q_combined = q_pitch * q_optical;
    tf.transform.rotation.x = q_combined.x();
    tf.transform.rotation.y = q_combined.y();
    tf.transform.rotation.z = q_combined.z();
    tf.transform.rotation.w = q_combined.w();

    tf_buffer_->setTransform(tf, "test", true);
  }

  sensor_msgs::msg::CameraInfo makeCameraInfo(
      const std::string &frame_id, double fx, double fy, double cx, double cy,
      uint32_t width, uint32_t height) {
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = rclcpp::Time(0);
    info.header.frame_id = frame_id;
    info.width = width;
    info.height = height;
    info.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
    info.p = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0};
    info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    info.distortion_model = "plumb_bob";
    info.d = {0, 0, 0, 0, 0};
    return info;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  depth_image_filters::HeightFilter filter_;
};

// Camera at 1.0m height, looking forward horizontally.
// A pixel in the image center at depth 2.0m should have height = 1.0m (camera height).
// Pixels above center map to higher points, below center to lower points.
TEST_F(HeightFilterTest, Float32_HorizontalCamera) {
  initFilter(0.0, 1.5, "base_link");
  setTransform("base_link", "camera_optical", 1.0);

  const int rows = 100, cols = 100;
  const double fx = 100.0, fy = 100.0;
  const double cx = 50.0, cy = 50.0;

  // Uniform depth = 2.0m
  cv::Mat image(rows, cols, CV_32FC1, cv::Scalar(2.0f));
  std::string encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  auto cam_info = makeCameraInfo("camera_optical", fx, fy, cx, cy, cols, rows);

  ASSERT_TRUE(filter_.apply(image, encoding, cam_info));

  // Center pixel (50,50): y_cam=0 -> height = camera_height = 1.0m (in range)
  EXPECT_TRUE(std::isfinite(image.at<float>(50, 50)));

  // Top pixel (0,50): y_cam = (0-50)*2/100 = -1.0 -> maps to z_target = 1.0 + 1.0 = 2.0m (out of range >1.5)
  EXPECT_TRUE(std::isnan(image.at<float>(0, 50)));

  // Bottom pixel (99,50): y_cam = (99-50)*2/100 = 0.98 -> maps to z_target = 1.0 - 0.98 = 0.02m (in range)
  EXPECT_TRUE(std::isfinite(image.at<float>(99, 50)));
}

TEST_F(HeightFilterTest, Uint16_HorizontalCamera) {
  initFilter(0.0, 1.5, "base_link");
  setTransform("base_link", "camera_optical", 1.0);

  const int rows = 100, cols = 100;
  const double fx = 100.0, fy = 100.0;
  const double cx = 50.0, cy = 50.0;

  // Uniform depth = 2000mm = 2.0m
  cv::Mat image(rows, cols, CV_16UC1, cv::Scalar(2000));
  std::string encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  auto cam_info = makeCameraInfo("camera_optical", fx, fy, cx, cy, cols, rows);

  ASSERT_TRUE(filter_.apply(image, encoding, cam_info));

  // Center pixel: height = 1.0m (in range)
  EXPECT_NE(image.at<uint16_t>(50, 50), 0);

  // Top pixel: height ~2.0m (out of range)
  EXPECT_EQ(image.at<uint16_t>(0, 50), 0);

  // Bottom pixel: height ~0.02m (in range)
  EXPECT_NE(image.at<uint16_t>(99, 50), 0);
}

TEST_F(HeightFilterTest, UnsupportedEncoding) {
  initFilter(-1.0, 2.0, "base_link");
  setTransform("base_link", "camera_optical", 1.0);

  cv::Mat image = cv::Mat::zeros(2, 2, CV_8UC1);
  std::string encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  auto cam_info = makeCameraInfo("camera_optical", 100, 100, 1, 1, 2, 2);

  rcutils_logging_set_logger_level("height_filter", RCUTILS_LOG_SEVERITY_FATAL);
  EXPECT_FALSE(filter_.apply(image, encoding, cam_info));
  rcutils_logging_set_logger_level("height_filter", RCUTILS_LOG_SEVERITY_INFO);
}

TEST_F(HeightFilterTest, MinGreaterThanMax) {
  initFilter(3.0, 1.0, "base_link");
  setTransform("base_link", "camera_optical", 1.0);

  cv::Mat image = cv::Mat::ones(2, 2, CV_32FC1);
  std::string encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  auto cam_info = makeCameraInfo("camera_optical", 100, 100, 1, 1, 2, 2);

  rcutils_logging_set_logger_level("height_filter", RCUTILS_LOG_SEVERITY_FATAL);
  EXPECT_FALSE(filter_.apply(image, encoding, cam_info));
  rcutils_logging_set_logger_level("height_filter", RCUTILS_LOG_SEVERITY_INFO);
}
