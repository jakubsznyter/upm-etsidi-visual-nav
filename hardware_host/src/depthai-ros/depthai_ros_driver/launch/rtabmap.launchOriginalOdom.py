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
    parameters = [
        {
            "frame_id": "oak-d-base-frame",
            "subscribe_rgb": True,
            "subscribe_depth": True,
            "odom_topic": "/odom",
            "approx_sync": True,
            "Rtabmap/DetectionRate": "2",
            "subscribe_imu": False,
            "publish_tf": True,
            #"Odom/Strategy": "1",
            "approx_sync":True,
            "Vis/CorType": "1",
            "Vis/MinInliers": "15",         # <<< Aumentar umbral de calidad
            #"Odom/ResetCountdown": "1",     # <<< RECUPERACIÓN AUTOMÁTICA
            "sync_queue_size": 30,
            "delete_db_on_start": True,
            
        
        # --- EL TRUCO PARA ROBUSTEZ 2D ---
            "Reg/Force3DoF": "true",      # <<< Forzar a 2.5D (movimiento en un plano)
            "Mem/IncrementalMemory": "true",
            "Grid/FromDepth": "true",
            "Grid/3D": "false",

            "Grid/MaxObstacleHeight": "0.4",   # Ignora puntos por encima de 1.5 metros (ej. el techo)
            "Grid/NoiseFilteringRadius": "0.1", # Ayuda a eliminar puntos aislados
            "Grid/NoiseFilteringMinNeighbors": "5",
            "Grid/RangeMax": "4.0",
            "Grid/RangeMin": "0.3",
             "Grid/RayTracing": "true",
             "Grid/MaxGroundHeight": "0.16"  # Ignora puntos por debajo de 10 cm del suelo del robot
            
        } 

    ]

    remappings = [
        ("rgb/image", name + "/rgb/image_rect"),
        ("rgb/camera_info", name + "/rgb/camera_info"),
        ("depth/image", name + "/stereo/image_raw"),
        ('odom', '/odom'),
    ]

    return [
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="camera_base_link_static_transform_publisher",
            arguments=["0.10", "0.0", "0.07", "0.0", "0.0", "0.0", "base_link", "oak-d-base-frame"],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, "launch", "camera.launch.py")
            ),
            launch_arguments={"name": name, "params_file": params_file}.items(),
        ),
       
        LoadComposableNodes(
            target_container=name + "_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="rtabmap_slam",
                    plugin="rtabmap_slam::CoreWrapper",
                    name="rtabmap",
                    parameters=parameters,
                    remappings=remappings,
                ),
            ],
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
