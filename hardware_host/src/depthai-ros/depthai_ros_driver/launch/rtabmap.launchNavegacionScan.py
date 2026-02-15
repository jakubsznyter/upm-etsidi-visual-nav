import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration("name").perform(context)
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    params_file = LaunchConfiguration("params_file")
   
    return [
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="camera_base_link_static_transform_publisher",
            arguments=["0.10", "0.0", "0.07", "0.0", "0.0", "0.0","base_link","oak-d-base-frame"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_imu_static_tf",
            arguments=["0", "0", "0", "0", "0", "0","base_link", "imu"],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, "launch", "camera.launch.py")
            ),
            launch_arguments={"name": name, "params_file": params_file}.items(),
        ),
         IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('witmotion_ros'),
                '/launch/wt901.launch.py'
            ]),
        ),
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(depthai_prefix, "config", "rgbd.yaml"),
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
