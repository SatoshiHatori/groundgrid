#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    point_cloud_topic = LaunchConfiguration('point_cloud_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'point_cloud_topic',
            default_value='/sensors/velodyne_points',
            description='Point cloud topic to subscribe to'
        ),
        ComposableNodeContainer(
            name='groundgrid_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='groundgrid',
                    plugin='groundgrid::GroundGridCp',
                    name='groundgrid',
                    remappings=[('/sensors/velodyne_points', point_cloud_topic)],
                )
            ],
        ),
    ])
