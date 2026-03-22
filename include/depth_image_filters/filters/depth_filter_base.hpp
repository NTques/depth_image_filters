//
// Created by antique on 3/22/26.
//

#ifndef DEPTH_IMAGE_FILTERS_DEPTH_FILTER_BASE_HPP
#define DEPTH_IMAGE_FILTERS_DEPTH_FILTER_BASE_HPP

#include "depth_image_filters/depth_image_filters_parameters.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <opencv2/core.hpp>

namespace depth_image_filters
{
    class DepthFilterBase
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(DepthFilterBase)

        explicit DepthFilterBase(const std::string& filter_name);
        virtual ~DepthFilterBase() = default;

        virtual void initialize(const std::shared_ptr<ParamListener>& param_listener);
        virtual bool apply(cv::Mat& image, const std::string& encoding) = 0;

        void update_params();

    protected:
        rclcpp::Logger logger_;
        std::shared_ptr<ParamListener> param_listener_;
        Params params_cache_;
    };
} // namespace depth_image_filters

#endif // DEPTH_IMAGE_FILTERS_DEPTH_FILTER_BASE_HPP
