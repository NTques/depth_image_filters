#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>

#include "depth_image_filters/depth_image_filters_parameters.hpp"
#include "depth_image_filters/filter_chain.hpp"
#include "depth_image_filters/filters/range_filter.hpp"
#include "depth_image_filters/filters/crop_filter.hpp"
#include "depth_image_filters/filters/height_filter.hpp"
#include "depth_image_filters/filters/speckle_filter.hpp"
#include "depth_image_filters/filters/median_filter.hpp"
#include "depth_image_filters/filters/bilateral_filter.hpp"
#include "depth_image_filters/filters/temporal_filter.hpp"
#include "depth_image_filters/publishers/publisher_base.hpp"
#include "depth_image_filters/publishers/depth_image_publisher.hpp"
#include "depth_image_filters/publishers/laser_scan_publisher.hpp"

namespace depth_image_filters {

class DepthImageFilterNode : public rclcpp::Node {
public:
  explicit DepthImageFilterNode(const rclcpp::NodeOptions &options)
      : Node("depth_image_filter_node", options) {
    param_listener_ = std::make_shared<ParamListener>(
        this->get_node_parameters_interface());
    params_ = param_listener_->get_params();

    setupTf();
    setupFilters();
    setupPublishers();
    setupSubscribers();

    RCLCPP_INFO(get_logger(), "DepthImageFilterNode initialized with %zu filters, %zu publishers.",
                filter_chain_filters_count_, publishers_.size());
  }

private:
  void setupTf() {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void setupFilters() {
    filter_chain_.clear();
    filter_chain_filters_count_ = 0;

    for (const auto &name : params_.filter_chains) {
      DepthFilterBase::SharedPtr filter;

      if (name == "range_filter") {
        filter = std::make_shared<RangeFilter>();
      } else if (name == "crop_filter") {
        filter = std::make_shared<CropFilter>();
      } else if (name == "height_filter") {
        filter = std::make_shared<HeightFilter>();
      } else if (name == "speckle_filter") {
        filter = std::make_shared<SpeckleFilter>();
      } else if (name == "median_filter") {
        filter = std::make_shared<MedianFilter>();
      } else if (name == "bilateral_filter") {
        filter = std::make_shared<BilateralFilter>();
      } else if (name == "temporal_filter") {
        filter = std::make_shared<TemporalFilter>();
      } else {
        RCLCPP_WARN(get_logger(), "Unknown filter: %s", name.c_str());
        continue;
      }

      if (auto hf = std::dynamic_pointer_cast<HeightFilter>(filter)) {
        hf->initialize(param_listener_, tf_buffer_);
      } else {
        filter->initialize(param_listener_);
      }

      filter_chain_.addFilter(filter);
      filter_chain_filters_count_++;
    }
  }

  void setupPublishers() {
    if (params_.depth_image_publisher.enable) {
      auto pub = std::make_unique<DepthImagePublisher>();
      pub->setup(this, params_);
      publishers_.push_back(std::move(pub));
    }

    if (params_.scan_publisher.enable) {
      auto pub = std::make_unique<LaserScanPublisher>();
      pub->setup(this, params_);
      publishers_.push_back(std::move(pub));
    }
  }

  void setupSubscribers() {
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        params_.camera_info_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(camera_info_mutex_);
          latest_camera_info_ = *msg;
          camera_info_received_ = true;
        });

    depth_sub_ = image_transport::create_subscription(
        this, params_.image_topic,
        std::bind(&DepthImageFilterNode::depthCallback, this,
                  std::placeholders::_1),
        "raw", rmw_qos_profile_sensor_data);
  }

  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    sensor_msgs::msg::CameraInfo camera_info;
    {
      std::lock_guard<std::mutex> lock(camera_info_mutex_);
      if (!camera_info_received_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "No CameraInfo received yet, skipping frame.");
        return;
      }
      camera_info = latest_camera_info_;
    }

    if (publishers_.empty()) {
      return;
    }

    if (param_listener_->is_old(params_)) {
      params_ = param_listener_->get_params();
    }

    sensor_msgs::msg::Image::SharedPtr output;
    if (!filter_chain_.apply(msg, camera_info, output)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Filter chain failed.");
      return;
    }

    for (auto &pub : publishers_) {
      pub->publish(output, camera_info, params_);
    }
  }

  // Parameters
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Filter chain
  FilterChain filter_chain_;
  size_t filter_chain_filters_count_ = 0;

  // Publishers
  std::vector<std::unique_ptr<PublisherBase>> publishers_;

  // Subscribers
  image_transport::Subscriber depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Cached camera info
  std::mutex camera_info_mutex_;
  sensor_msgs::msg::CameraInfo latest_camera_info_;
  bool camera_info_received_ = false;
};

} // namespace depth_image_filters

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<depth_image_filters::DepthImageFilterNode>(
      rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
