#include <gtest/gtest.h>
#include <rcutils/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cmath>
#include <limits>

#include "depth_image_filters/filters/range_filter.hpp"

static sensor_msgs::msg::CameraInfo kEmptyCameraInfo;

class RangeFilterTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_range_filter");
  }

  void TearDown() override { rclcpp::shutdown(); }

  void initFilter(double min_distance, double max_distance) {
    node_->declare_parameter("range_filter.min_distance", min_distance);
    node_->declare_parameter("range_filter.max_distance", max_distance);

    auto param_listener =
        std::make_shared<depth_image_filters::ParamListener>(node_);
    filter_.initialize(param_listener);
  }

  rclcpp::Node::SharedPtr node_;
  depth_image_filters::RangeFilter filter_;
};

TEST_F(RangeFilterTest, Float32InRange) {
  initFilter(0.5, 5.0);

  cv::Mat image = (cv::Mat_<float>(1, 5) << 0.3f, 0.5f, 2.5f, 5.0f, 6.0f);
  std::string encoding = sensor_msgs::image_encodings::TYPE_32FC1;

  ASSERT_TRUE(filter_.apply(image, encoding, kEmptyCameraInfo));

  EXPECT_TRUE(std::isnan(image.at<float>(0, 0)));   // 0.3 < min
  EXPECT_FLOAT_EQ(image.at<float>(0, 1), 0.5f);     // == min, keep
  EXPECT_FLOAT_EQ(image.at<float>(0, 2), 2.5f);     // in range
  EXPECT_FLOAT_EQ(image.at<float>(0, 3), 5.0f);     // == max, keep
  EXPECT_TRUE(std::isnan(image.at<float>(0, 4)));    // 6.0 > max
}

TEST_F(RangeFilterTest, Float32NaNAndInf) {
  initFilter(0.0, 10.0);

  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  cv::Mat image = (cv::Mat_<float>(1, 3) << nan, inf, 5.0f);
  std::string encoding = sensor_msgs::image_encodings::TYPE_32FC1;

  ASSERT_TRUE(filter_.apply(image, encoding, kEmptyCameraInfo));

  EXPECT_TRUE(std::isnan(image.at<float>(0, 0)));
  EXPECT_TRUE(std::isnan(image.at<float>(0, 1)));
  EXPECT_FLOAT_EQ(image.at<float>(0, 2), 5.0f);
}

TEST_F(RangeFilterTest, Uint16InRange) {
  initFilter(0.5, 3.0);  // 500mm ~ 3000mm

  cv::Mat image = (cv::Mat_<uint16_t>(1, 5) << 200, 500, 1500, 3000, 4000);
  std::string encoding = sensor_msgs::image_encodings::TYPE_16UC1;

  ASSERT_TRUE(filter_.apply(image, encoding, kEmptyCameraInfo));

  EXPECT_EQ(image.at<uint16_t>(0, 0), 0);      // 200 < 500
  EXPECT_EQ(image.at<uint16_t>(0, 1), 500);     // == min, keep
  EXPECT_EQ(image.at<uint16_t>(0, 2), 1500);    // in range
  EXPECT_EQ(image.at<uint16_t>(0, 3), 3000);    // == max, keep
  EXPECT_EQ(image.at<uint16_t>(0, 4), 0);       // 4000 > 3000
}

TEST_F(RangeFilterTest, UnsupportedEncoding) {
  initFilter(0.5, 5.0);

  cv::Mat image = cv::Mat::zeros(2, 2, CV_8UC1);
  std::string encoding = sensor_msgs::image_encodings::TYPE_8UC1;

  rcutils_logging_set_logger_level("range_filter", RCUTILS_LOG_SEVERITY_FATAL);
  EXPECT_FALSE(filter_.apply(image, encoding, kEmptyCameraInfo));
  rcutils_logging_set_logger_level("range_filter", RCUTILS_LOG_SEVERITY_INFO);
}

TEST_F(RangeFilterTest, MinGreaterThanMax) {
  initFilter(5.0, 1.0);

  cv::Mat image = cv::Mat::ones(2, 2, CV_32FC1);
  std::string encoding = sensor_msgs::image_encodings::TYPE_32FC1;

  rcutils_logging_set_logger_level("range_filter", RCUTILS_LOG_SEVERITY_FATAL);
  EXPECT_FALSE(filter_.apply(image, encoding, kEmptyCameraInfo));
  rcutils_logging_set_logger_level("range_filter", RCUTILS_LOG_SEVERITY_INFO);
}
