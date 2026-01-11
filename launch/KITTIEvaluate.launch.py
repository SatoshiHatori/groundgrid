#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Arguments
    rate = LaunchConfiguration("rate")
    paused = LaunchConfiguration("paused")
    point_cloud_topic = LaunchConfiguration("point_cloud_topic")
    sequence = LaunchConfiguration("sequence")
    directory = LaunchConfiguration("directory")
    start = LaunchConfiguration("start")
    end = LaunchConfiguration("end")

    pkg_share = get_package_share_directory("groundgrid")
    groundgrid_launch = os.path.join(pkg_share, "launch", "GroundGrid.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("rate", default_value="1.0"),
            DeclareLaunchArgument("paused", default_value="true"),
            DeclareLaunchArgument("point_cloud_topic", default_value="/pointcloud"),
            DeclareLaunchArgument("sequence", default_value="00"),
            DeclareLaunchArgument("directory", default_value=""),
            DeclareLaunchArgument("start", default_value="0.0"),
            DeclareLaunchArgument("end", default_value="99999999"),
            SetParameter(name="use_sim_time", value=True),
            # static transforms
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="kitti_base_to_base",
                arguments=[
                    "1.95",
                    "0",
                    "-1.73",
                    "0",
                    "0",
                    "0",
                    "kitti_base_link",
                    "base_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="kitti_map_to_map",
                arguments=[
                    "-2.48",
                    "0",
                    "1.733",
                    "0",
                    "0",
                    "0",
                    "1",
                    "kitti_map",
                    "map",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom",
                arguments=["0", "0", "0", "0", "0", "0", "1", "map", "odom"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="kitti_base_to_velodyne",
                arguments=["0", "0", "0", "0", "0", "0", "kitti_base_link", "velodyne"],
            ),
            # KITTI player
            Node(
                package="groundgrid",
                executable="kitti_data_publisher.py",
                name="kitti_data_publisher",
                output="screen",
                parameters=[
                    {
                        "/kitti_player/sequence": sequence,
                        "/kitti_player/directory": directory,
                        "/kitti_player/start": start,
                        "/kitti_player/end": end,
                        "/kitti_player/paused": paused,
                        "rate": rate,
                    }
                ],
                remappings=[("/kitti/cloud", point_cloud_topic)],
            ),
            # GroundGrid composable component
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(groundgrid_launch),
                launch_arguments={"point_cloud_topic": point_cloud_topic}.items(),
            ),
            # Evaluation node
            Node(
                package="groundgrid",
                executable="eval_groundpoint_classifier.py",
                name="eval_groundpoint_classifier",
                output="screen",
            ),
        ]
    )
