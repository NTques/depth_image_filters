#include <gtest/gtest.h>
#include <rcutils/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cmath>
#include <limits>

#include "depth_image_filters/filters/median_filter.hpp"

static sensor_msgs::msg::CameraInfo kEmptyCameraInfo;

class MedianFilterTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_median_filter");
  }

  void TearDown() override { rclcpp::shutdown(); }

  void initFilter(int kernel_size) {
    node_->declare_parameter("median_filter.kernel_size", kernel_size);

    auto param_listener =
        std::make_shared<depth_image_filters::ParamListener>(node_);
    filter_.initialize(param_listener);
  }

  rclcpp::Node::SharedPtr node_;
  depth_image_filters::MedianFilter filter_;
};

TEST_F(MedianFilterTest, Uint16_RemovesSaltAndPepper) {
  initFilter(3);

  // 5x5 uniform image at 2000mm with one outlier
  cv::Mat image(5, 5, CV_16UC1, cv::Scalar(2000));
  image.at<uint16_t>(2, 2) = 9000;  // salt noise

  std::string encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  ASSERT_TRUE(filter_.apply(image, encoding, kEmptyCameraInfo));

  // Center pixel should be smoothed to 2000 by median
  EXPECT_EQ(image.at<uint16_t>(2, 2), 2000);
}

TEST_F(MedianFilterTest, Uint16_PreservesZeroPixels) {
  initFilter(3);

  cv::Mat image(5, 5, CV_16UC1, cv::Scalar(2000));
  image.at<uint16_t>(2, 2) = 0;  // invalid pixel

  std::string encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  ASSERT_TRUE(filter_.apply(image, encoding, kEmptyCameraInfo));

  // Originally invalid pixel should stay 0
  EXPECT_EQ(image.at<uint16_t>(2, 2), 0);
}

TEST_F(MedianFilterTest, Float32_RemovesSaltAndPepper) {
  initFilter(3);

  cv::Mat image(5, 5, CV_32FC1, cv::Scalar(2.0f));
  image.at<float>(2, 2) = 9.0f;  // outlier

  std::string encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  ASSERT_TRUE(filter_.apply(image, encoding, kEmptyCameraInfo));

  // Should be smoothed to 2.0
  EXPECT_FLOAT_EQ(image.at<float>(2, 2), 2.0f);
}

TEST_F(MedianFilterTest, Float32_PreservesNaN) {
  initFilter(3);

  const float nan = std::numeric_limits<float>::quiet_NaN();
  cv::Mat image(5, 5, CV_32FC1, cv::Scalar(2.0f));
  image.at<float>(2, 2) = nan;

  std::string encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  ASSERT_TRUE(filter_.apply(image, encoding, kEmptyCameraInfo));

  // Originally NaN pixel should stay NaN
  EXPECT_TRUE(std::isnan(image.at<float>(2, 2)));
}

TEST_F(MedianFilterTest, EvenKernelFails) {
  initFilter(4);

  cv::Mat image(5, 5, CV_16UC1, cv::Scalar(2000));
  std::string encoding = sensor_msgs::image_encodings::TYPE_16UC1;

  rcutils_logging_set_logger_level("median_filter",
                                   RCUTILS_LOG_SEVERITY_FATAL);
  EXPECT_FALSE(filter_.apply(image, encoding, kEmptyCameraInfo));
  rcutils_logging_set_logger_level("median_filter",
                                   RCUTILS_LOG_SEVERITY_INFO);
}

TEST_F(MedianFilterTest, UnsupportedEncoding) {
  initFilter(3);

  cv::Mat image = cv::Mat::zeros(2, 2, CV_8UC1);
  std::string encoding = sensor_msgs::image_encodings::TYPE_8UC1;

  rcutils_logging_set_logger_level("median_filter",
                                   RCUTILS_LOG_SEVERITY_FATAL);
  EXPECT_FALSE(filter_.apply(image, encoding, kEmptyCameraInfo));
  rcutils_logging_set_logger_level("median_filter",
                                   RCUTILS_LOG_SEVERITY_INFO);
}
