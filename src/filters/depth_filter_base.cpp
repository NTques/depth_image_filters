//
// Created by antique on 3/22/26.
//

#include "depth_image_filters/filters/depth_filter_base.hpp"

namespace depth_image_filters {
DepthFilterBase::DepthFilterBase(const std::string &filter_name)
    : logger_(rclcpp::get_logger(filter_name)) {}

void DepthFilterBase::initialize(
    const std::shared_ptr<ParamListener> &param_listener) {
  param_listener_ = param_listener;
  params_cache_ = param_listener_->get_params();
}

void DepthFilterBase::update_params() {
  if (!param_listener_)
    return;
  if (param_listener_->is_old(params_cache_)) {
    params_cache_ = param_listener_->get_params();
  }
}
} // namespace depth_image_filters
