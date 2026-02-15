import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    

    # --- 2. IMU (WitMotion) ---
    imu_node = Node(
        package='wit_imu_driver',
        executable='wit_imu_node',
        name='wit_imu_node',
        parameters=[{
            'device': '/dev/ttyUSB0',
            'baud': 230400
        }],

    )

# --- 3. STEROWNIK KAMERY OAK-D (Stereo, 25 FPS, Jasny Obraz) ---
    camera_node = Node(
        package='depthai_ros_driver',
        executable='camera_node',
        name='oak',
        output='screen',
        arguments=[
            'raw',
            '--ros-args',
            '-p', 'qos_overrides./oak/left/image.raw.publisher.reliability:=best_effort',
            '-p','qos_overrides./oak/left/image.raw.publisher.history:=keep_last',
            '-p','qos_overrides./oak/left/image.raw.publisher.depth:=1',
            '-p', 'qos_overrides./oak/right/image.raw.publisher.reliability:=best_effort',
            '-p','qos_overrides./oak/right/image.raw.publisher.history:=keep_last',
            '-p','qos_overrides./oak/right/image.raw.publisher.depth:=1',
        ],
        parameters=[{
            'camera.i_pipeline_type': 'Stereo',
            'camera.i_nn_type': 'none',
            'camera.i_usb_speed': 'SUPER_PLUS',
	    	'camera.i_enable_imu': False,
            'camera.i_enable_ir': False,
         
            # --- LEWA KAMERA ---
            'left.i_resolution': 'THE_480_P',
            'left.i_fps': 15.0,            
            'left.i_output_ISP': False,
            'left.i_low_bandwidth': False,
            'left.i_publish_topic': True,
            'left.i_low_bandwidth': True,
            'left.i_low_bandwidth_quality': 50,
            'left.i_publish_compressed': False,

            # --- PRAWA KAMERA ---
            'right.i_resolution': 'THE_480_P',
            'right.i_fps': 15.0,
            'right.i_publish_topic': True,
            'right.i_enable_feature_tracker': False,
            'right.i_output_ISP': False,
            'right.i_low_bandwidth': True,
            'right.i_low_bandwidth_quality': 50,
            'right.i_publish_compressed': False,


            # --- STEREO ---
            'stereo.i_publish_topic': False,
            'stereo.i_align_depth': False,
            'rgb.i_publish_topic': False,
            'stereo.i_set_input_size': True,
            'stereo.i_input_width': 640,
            'stereo.i_input_height': 480
        }]
    )


    # --- 5. Sterownik Robota ---
    robot_driver_node = Node(
        package='robot_driver',
        executable='robot_driver_node',
        name='robot_driver',
        output='screen'
    )

    # --- SEKWENCJA STARTOWA ---
    return LaunchDescription([
        imu_node,
        camera_node,
        robot_driver_node
    ])
