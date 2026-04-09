from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    package_name = "robot_description"

    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        "urdf",
        "robot.urdf.xacro"
    )

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": open(urdf_path).read()
            }]
        )
    ])