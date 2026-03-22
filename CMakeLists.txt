cmake_minimum_required(VERSION 3.8)
project(depth_image_filters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(
        depth_image_filters_parameters
        depth_image_filters_params.yaml
)

set(EXECUTABLE_NAME depth_image_filter_node)
set(DEPENDENCIES
        rclcpp
        sensor_msgs
        std_msgs
        image_transport
        cv_bridge
        image_geometry
        tf2
        tf2_ros
        OpenCV
)
add_executable(${EXECUTABLE_NAME} src/depth_image_filter_node.cpp
        src/filters/depth_filter_base.cpp
        include/depth_image_filters/filters/depth_filter_base.hpp
        src/filters/crop_filter.cpp
        include/depth_image_filters/filters/crop_filter.hpp
        src/filters/range_filter.cpp
        include/depth_image_filters/filters/range_filter.hpp
        src/filter_chain.cpp
        include/depth_image_filters/filter_chain.hpp)
target_include_directories(${EXECUTABLE_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(${EXECUTABLE_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(${EXECUTABLE_NAME}
        ${DEPENDENCIES}
)
target_link_libraries(${EXECUTABLE_NAME}
        depth_image_filters_parameters
)

install(TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
