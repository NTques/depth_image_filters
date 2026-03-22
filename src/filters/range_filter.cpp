//
// Created by antique on 3/22/26.
//

#include "depth_image_filters/filters/range_filter.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <limits>

namespace depth_image_filters
{
    RangeFilter::RangeFilter()
        : DepthFilterBase("range_filter")
    {
    }

    bool RangeFilter::apply(cv::Mat& image, const std::string& encoding)
    {
        const auto& p = params_cache_.range_filter;

        if (p.min_distance >= p.max_distance)
        {
            RCLCPP_ERROR(logger_,
                         "min_distance(%.3f) must be less than max_distance(%.3f)",
                         p.min_distance, p.max_distance);
            return false;
        }

        const bool is_float = (encoding == sensor_msgs::image_encodings::TYPE_32FC1);
        const bool is_uint16 = (encoding == sensor_msgs::image_encodings::TYPE_16UC1);

        if (!is_float && !is_uint16)
        {
            RCLCPP_ERROR(logger_,
                         "Unsupported encoding: %s (expected 32FC1 or 16UC1)",
                         encoding.c_str());
            return false;
        }

        if (is_float)
        {
            const float min_val = static_cast<float>(p.min_distance);
            const float max_val = static_cast<float>(p.max_distance);
            const float nan = std::numeric_limits<float>::quiet_NaN();

            image.forEach<float>([&](float& px, const int [])
            {
                if (!std::isfinite(px) || px < min_val || px > max_val)
                {
                    px = nan;
                }
            });
        }
        else
        {
            const uint16_t min_val = static_cast<uint16_t>(p.min_distance * 1000.0);
            const uint16_t max_val = static_cast<uint16_t>(p.max_distance * 1000.0);

            image.forEach<uint16_t>([&](uint16_t& px, const int [])
            {
                if (px < min_val || px > max_val)
                {
                    px = 0u;
                }
            });
        }

        return true;
    }
} // namespace depth_image_filters
