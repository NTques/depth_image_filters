#include <gtest/gtest.h>
#include <rcutils/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "depth_image_filters/filters/crop_filter.hpp"

static sensor_msgs::msg::CameraInfo kEmptyCameraInfo;

class CropFilterTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_crop_filter");
  }

  void TearDown() override { rclcpp::shutdown(); }

  void initFilter(const std::string &margin_type, double top, double bottom,
                  double left, double right) {
    node_->declare_parameter("crop_filter.margin_type", margin_type);
    node_->declare_parameter("crop_filter.top", top);
    node_->declare_parameter("crop_filter.bottom", bottom);
    node_->declare_parameter("crop_filter.left", left);
    node_->declare_parameter("crop_filter.right", right);

    auto param_listener =
        std::make_shared<depth_image_filters::ParamListener>(node_);
    filter_.initialize(param_listener);
  }

  cv::Mat createTestImage(int rows, int cols) {
    cv::Mat image(rows, cols, CV_16UC1);
    uint16_t val = 0;
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        image.at<uint16_t>(r, c) = val++;
      }
    }
    return image;
  }

  rclcpp::Node::SharedPtr node_;
  depth_image_filters::CropFilter filter_;
};

TEST_F(CropFilterTest, PixelModeCrop) {
  initFilter("pixel", 10, 10, 20, 20);

  cv::Mat image = createTestImage(100, 200);
  std::string encoding = sensor_msgs::image_encodings::TYPE_16UC1;

  ASSERT_TRUE(filter_.apply(image, encoding, kEmptyCameraInfo));

  EXPECT_EQ(image.rows, 80);   // 100 - 10 - 10
  EXPECT_EQ(image.cols, 160);  // 200 - 20 - 20
}

TEST_F(CropFilterTest, RatioModeCrop) {
  initFilter("ratio", 0.1, 0.1, 0.2, 0.2);

  cv::Mat image = createTestImage(100, 200);
  std::string encoding = sensor_msgs::image_encodings::TYPE_16UC1;

  ASSERT_TRUE(filter_.apply(image, encoding, kEmptyCameraInfo));

  EXPECT_EQ(image.rows, 80);   // 100 - 10 - 10
  EXPECT_EQ(image.cols, 120);  // 200 - 40 - 40
}

TEST_F(CropFilterTest, ZeroMarginReturnsOriginalSize) {
  initFilter("pixel", 0, 0, 0, 0);

  cv::Mat image = createTestImage(50, 80);
  std::string encoding = sensor_msgs::image_encodings::TYPE_16UC1;

  ASSERT_TRUE(filter_.apply(image, encoding, kEmptyCameraInfo));

  EXPECT_EQ(image.rows, 50);
  EXPECT_EQ(image.cols, 80);
}

TEST_F(CropFilterTest, MarginTooLargeFails) {
  initFilter("pixel", 60, 60, 0, 0);  // top + bottom > height

  cv::Mat image = createTestImage(100, 100);
  std::string encoding = sensor_msgs::image_encodings::TYPE_16UC1;

  rcutils_logging_set_logger_level("crop_filter", RCUTILS_LOG_SEVERITY_FATAL);
  EXPECT_FALSE(filter_.apply(image, encoding, kEmptyCameraInfo));
  rcutils_logging_set_logger_level("crop_filter", RCUTILS_LOG_SEVERITY_INFO);
}

TEST_F(CropFilterTest, CroppedDataIsCorrect) {
  initFilter("pixel", 1, 1, 1, 1);

  // 4x4 image with sequential values
  cv::Mat image = createTestImage(4, 4);
  std::string encoding = sensor_msgs::image_encodings::TYPE_16UC1;

  ASSERT_TRUE(filter_.apply(image, encoding, kEmptyCameraInfo));

  EXPECT_EQ(image.rows, 2);
  EXPECT_EQ(image.cols, 2);
  // Original row=1,col=1 => value = 1*4+1 = 5
  EXPECT_EQ(image.at<uint16_t>(0, 0), 5);
  EXPECT_EQ(image.at<uint16_t>(0, 1), 6);
  EXPECT_EQ(image.at<uint16_t>(1, 0), 9);
  EXPECT_EQ(image.at<uint16_t>(1, 1), 10);
}

TEST_F(CropFilterTest, DeepCopyAfterCrop) {
  initFilter("pixel", 10, 10, 10, 10);

  cv::Mat original = createTestImage(100, 100);
  cv::Mat image = original.clone();
  std::string encoding = sensor_msgs::image_encodings::TYPE_16UC1;

  ASSERT_TRUE(filter_.apply(image, encoding, kEmptyCameraInfo));

  // Modifying cropped image should not affect original
  image.at<uint16_t>(0, 0) = 65535;
  EXPECT_NE(original.at<uint16_t>(10, 10), 65535);
}
