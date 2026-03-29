#include <gtest/gtest.h>
#include <rcutils/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cmath>
#include <limits>

#include "depth_image_filters/filters/speckle_filter.hpp"

static sensor_msgs::msg::CameraInfo kEmptyCameraInfo;

class SpeckleFilterTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_speckle_filter");
  }

  void TearDown() override { rclcpp::shutdown(); }

  void initFilter(int max_speckle_size, double max_diff) {
    node_->declare_parameter("speckle_filter.max_speckle_size",
                             max_speckle_size);
    node_->declare_parameter("speckle_filter.max_diff", max_diff);

    auto param_listener =
        std::make_shared<depth_image_filters::ParamListener>(node_);
    filter_.initialize(param_listener);
  }

  rclcpp::Node::SharedPtr node_;
  depth_image_filters::SpeckleFilter filter_;
};

TEST_F(SpeckleFilterTest, Uint16_RemovesSmallBlob) {
  initFilter(10, 0.1);  // remove blobs < 10 pixels, max_diff 100mm

  // 20x20 uniform image at 2000mm, with a small 2x2 speckle at (5,5)
  cv::Mat image(20, 20, CV_16UC1, cv::Scalar(2000));
  image.at<uint16_t>(5, 5) = 3000;
  image.at<uint16_t>(5, 6) = 3000;
  image.at<uint16_t>(6, 5) = 3000;
  image.at<uint16_t>(6, 6) = 3000;

  std::string encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  ASSERT_TRUE(filter_.apply(image, encoding, kEmptyCameraInfo));

  // Speckle (4 pixels < 10) should be removed
  EXPECT_EQ(image.at<uint16_t>(5, 5), 0);
  EXPECT_EQ(image.at<uint16_t>(5, 6), 0);
  EXPECT_EQ(image.at<uint16_t>(6, 5), 0);
  EXPECT_EQ(image.at<uint16_t>(6, 6), 0);

  // Large region should remain
  EXPECT_EQ(image.at<uint16_t>(0, 0), 2000);
  EXPECT_EQ(image.at<uint16_t>(10, 10), 2000);
}

TEST_F(SpeckleFilterTest, Uint16_KeepsLargeBlob) {
  initFilter(3, 0.1);  // remove blobs < 3 pixels

  cv::Mat image(20, 20, CV_16UC1, cv::Scalar(2000));
  // 2x2 speckle = 4 pixels, threshold is 3 -> should be kept
  image.at<uint16_t>(5, 5) = 3000;
  image.at<uint16_t>(5, 6) = 3000;
  image.at<uint16_t>(6, 5) = 3000;
  image.at<uint16_t>(6, 6) = 3000;

  std::string encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  ASSERT_TRUE(filter_.apply(image, encoding, kEmptyCameraInfo));

  // 4 pixels >= threshold 3, should be kept
  EXPECT_EQ(image.at<uint16_t>(5, 5), 3000);
}

TEST_F(SpeckleFilterTest, Float32_RemovesSmallBlob) {
  initFilter(10, 0.1);

  cv::Mat image(20, 20, CV_32FC1, cv::Scalar(2.0f));
  // Small speckle at 3.0m (diff > 100mm from surroundings)
  image.at<float>(5, 5) = 3.0f;
  image.at<float>(5, 6) = 3.0f;

  std::string encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  ASSERT_TRUE(filter_.apply(image, encoding, kEmptyCameraInfo));

  // Small speckle should become NaN
  EXPECT_TRUE(std::isnan(image.at<float>(5, 5)));
  EXPECT_TRUE(std::isnan(image.at<float>(5, 6)));

  // Large region untouched
  EXPECT_FLOAT_EQ(image.at<float>(0, 0), 2.0f);
}

TEST_F(SpeckleFilterTest, UnsupportedEncoding) {
  initFilter(10, 0.1);

  cv::Mat image = cv::Mat::zeros(2, 2, CV_8UC1);
  std::string encoding = sensor_msgs::image_encodings::TYPE_8UC1;

  rcutils_logging_set_logger_level("speckle_filter",
                                   RCUTILS_LOG_SEVERITY_FATAL);
  EXPECT_FALSE(filter_.apply(image, encoding, kEmptyCameraInfo));
  rcutils_logging_set_logger_level("speckle_filter",
                                   RCUTILS_LOG_SEVERITY_INFO);
}
