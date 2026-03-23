import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('depth_image_filters')
    default_params_file = os.path.join(pkg_dir, 'config', 'depth_image_filters.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the parameter file',
    )

    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/depth/image_raw',
        description='Input depth image topic',
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/depth/camera_info',
        description='Input camera info topic',
    )

    node = Node(
        package='depth_image_filters',
        executable='depth_image_filter_node',
        name='depth_image_filter_node',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('~/input/depth', LaunchConfiguration('depth_topic')),
            ('~/input/camera_info', LaunchConfiguration('camera_info_topic')),
        ],
        output='screen',
    )

    return LaunchDescription([
        params_file_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        node,
    ])
