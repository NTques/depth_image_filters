//
// Created by antique on 3/22/26.
//

#include "depth_image_filters/filter_chain.hpp"

#include <cv_bridge/cv_bridge.hpp>

namespace depth_image_filters
{
    void FilterChain::addFilter(const DepthFilterBase::SharedPtr& filter)
    {
        filters_.push_back(filter);
    }

    bool FilterChain::apply(const sensor_msgs::msg::Image::ConstSharedPtr& input,
                            sensor_msgs::msg::CameraInfo& camera_info,
                            sensor_msgs::msg::Image::SharedPtr& output)
    {
        if (filters_.empty())
        {
            output = std::make_shared<sensor_msgs::msg::Image>(*input);
            return true;
        }

        cv_bridge::CvImagePtr cv_image;
        try
        {
            cv_image = cv_bridge::toCvCopy(input);
        }
        catch (const cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(logger_, "cv_bridge exception: %s", e.what());
            return false;
        }

        for (const auto& filter : filters_)
        {
            filter->update_params();
            if (!filter->apply(cv_image->image, cv_image->encoding, camera_info))
            {
                RCLCPP_ERROR(logger_, "Filter failed.");
                return false;
            }
        }

        output = cv_image->toImageMsg();
        return true;
    }

    void FilterChain::clear()
    {
        filters_.clear();
    }
} // depth_image_filters
