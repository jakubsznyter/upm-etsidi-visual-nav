import os

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

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


def generate_launch_description():
    # ... (definición de camera_params_file) ...
    
    # Ruta al archivo de configuración del EKF
    ekf_params_file = '/home/luis/Robot/src/robot_driver/config/ekf.yaml'

    return LaunchDescription([
        # Incluir el launch file de la IMU WitMotion
        ##IncludeLaunchDescription(
         #   PythonLaunchDescriptionSource([
        #        get_package_share_directory('witmotion_ros'),
        #        '/launch/wt901.launch.py'
        #    ]),
        #),

       # Node(
       #     package='tf2_ros',
       #     executable='static_transform_publisher',
        #    name='imu_base_link_static_transform_publisher',
        ##    arguments=[
         #       # x, y, z, roll, pitch, yaw
        #        '0.0', '0.0', '0.0',  # Posición relativa de la IMU respecto a base_link
         #       '0.0', '0.0', '0.0',  # Orientación (ajústala si es necesario)
         #       'base_link',
          #      'imu'
           # ],
          # output='screen'
       # ),


        # 3. Driver del Robot (ahora publica en /odom/odom_encoder)
        Node(
           package='robot_driver',
           executable='robot_driver_node',
           name='robot_driver',
           output='screen',
       ),


        # 4. Nodo del Filtro de Kalman (EKF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ ekf_params_file],
            remappings=[('odometry/filtered', '/odom')]
            
        ),
    ])
