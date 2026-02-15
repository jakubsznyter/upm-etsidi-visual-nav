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




def generate_launch_description():
    # --- Rutas a tus archivos de configuración ---
    robot_driver_prefix = get_package_share_directory('robot_driver')
    map_file = os.path.join(robot_driver_prefix, 'config', 'Mapa_Lab.yaml')
    nav2_params_file = os.path.join(robot_driver_prefix, 'config', 'Parametros_Nav2.yaml')
    ekf_params_file = '/home/luis/Robot/src/robot_driver/config/ekf.yaml'



    # --- NODO 1: El driver de bajo nivel que mueve el robot ---
    robot_driver_node = Node(
        package='robot_driver',
        executable='robot_driver_node',
        name='robot_driver_node',
        output='screen'
    )
    
    depthimage_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        # Tu remapping corregido
        remappings=[
            ('depth', '/oak/stereo/image_raw'),
            ('depth_camera_info', '/oak/stereo/camera_info'),
            ('scan', '/scan')
        ],
        parameters=[{
            'output_frame': 'base_link',
            'range_min': 0.3,
            'range_max': 3.0,
            'scan_height': 1,           # REDUCIR: procesar solo 1 fila de píxeles
            #'scan_time': 0.033,         # ~30 Hz
            'target_frame': 'base_link',
            'use_inf': False,
            'concurrency_level': 1      # Evitar sobrecarga de CPU
        }]
    )

    # --- NODO 2: La Pila de Navegación Nav2 (Lanzada manualmente SIN AMCL) ---
    
    # El Lifecycle Manager es el "director de orquesta" de Nav2
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': True},
                    # ¡CORREGIDO! Hemos quitado 'amcl' de la lista
                    {'node_names': ['map_server',
                                    #'amcl',
                                    'controller_server',
                                    'planner_server',
                                    'bt_navigator']}]
    )
    filtro_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ ekf_params_file],
        remappings=[('odometry/filtered', '/odom')]
            
        ),

    # El Map Server carga tu mapa guardado
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}]
    )
    
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file]
    )


    # El Controller Server se encarga del control local
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file]
    )
    
    # El Planner Server se encarga de la planificación global
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file]
    )
    
    # El BT Navigator ejecuta el árbol de comportamiento
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file]
    )
    

    # NODO 3: RViz para visualizar
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
    )
    
    return LaunchDescription([
        #robot_driver_node,
        
        # Añadimos todos los nodos de Nav2 a la descripción (sin amcl)
        #amcl_node,
        lifecycle_manager_node,
        map_server_node,
        controller_server_node,
        planner_server_node,
        bt_navigator_node,
        depthimage_to_laserscan_node,
        #filtro_node,
        
        
        rviz_node
    ])
