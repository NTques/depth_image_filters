//
// Created by antique on 3/23/26.
//

#include "depth_image_filters/publishers/publisher_base.hpp"

namespace depth_image_filters {

PublisherBase::PublisherBase(const std::string &name)
    : logger_(rclcpp::get_logger(name)) {}

} // namespace depth_image_filters
