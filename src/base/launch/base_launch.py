import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_base = get_package_share_directory('base')
    
    # Config paths
    kinematics_config = os.path.join(pkg_base, 'params', 'kinematics_params.yaml')
    hardware_config = os.path.join(pkg_base, 'params', 'hardware_params.yaml')

    # 1. Kinematics Node
    kinematics_node = Node(
        package='base',
        executable='kinematics_node',
        name='kinematics_node',
        output='screen',
        parameters=[kinematics_config]
    )

    # 2. Hardware_Node
    hardware_node = Node(
        package='base',
        executable='hardware_node',
        name='hardware_node',
        output='screen',

        parameters=[hardware_config]
        
    )


    

    return LaunchDescription([
        kinematics_node,
        hardware_node
    ])

