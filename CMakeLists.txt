cmake_minimum_required(VERSION 3.8)
project(depth_image_filters)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
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
    tf2_geometry_msgs
)

add_executable(${EXECUTABLE_NAME}
    src/depth_image_filter_node.cpp
    src/filters/depth_filter_base.cpp
    src/filters/crop_filter.cpp
    src/filters/range_filter.cpp
    src/filters/height_filter.cpp
    src/filters/speckle_filter.cpp
    src/filters/median_filter.cpp
    src/filter_chain.cpp
    src/publishers/publisher_base.cpp
    src/publishers/depth_image_publisher.cpp
    src/publishers/laser_scan_publisher.cpp)
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

install(DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(FILTER_SOURCES
      src/filters/depth_filter_base.cpp
      src/filters/range_filter.cpp
      src/filters/crop_filter.cpp
      src/filters/height_filter.cpp
      src/filters/speckle_filter.cpp
      src/filters/median_filter.cpp
  )

  ament_add_gtest(test_range_filter test/test_range_filter.cpp ${FILTER_SOURCES})
  target_include_directories(test_range_filter PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_range_filter ${DEPENDENCIES})
  target_link_libraries(test_range_filter depth_image_filters_parameters)

  ament_add_gtest(test_crop_filter test/test_crop_filter.cpp ${FILTER_SOURCES})
  target_include_directories(test_crop_filter PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_crop_filter ${DEPENDENCIES})
  target_link_libraries(test_crop_filter depth_image_filters_parameters)

  ament_add_gtest(test_height_filter test/test_height_filter.cpp ${FILTER_SOURCES})
  target_include_directories(test_height_filter PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_height_filter ${DEPENDENCIES})
  target_link_libraries(test_height_filter depth_image_filters_parameters)

  ament_add_gtest(test_speckle_filter test/test_speckle_filter.cpp ${FILTER_SOURCES})
  target_include_directories(test_speckle_filter PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_speckle_filter ${DEPENDENCIES})
  target_link_libraries(test_speckle_filter depth_image_filters_parameters)

  ament_add_gtest(test_median_filter test/test_median_filter.cpp ${FILTER_SOURCES})
  target_include_directories(test_median_filter PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_median_filter ${DEPENDENCIES})
  target_link_libraries(test_median_filter depth_image_filters_parameters)

  ament_add_gtest(test_visual_output test/test_visual_output.cpp ${FILTER_SOURCES})
  target_include_directories(test_visual_output PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_visual_output ${DEPENDENCIES})
  target_link_libraries(test_visual_output depth_image_filters_parameters)
endif()

ament_package()
