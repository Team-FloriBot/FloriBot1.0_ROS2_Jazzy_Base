from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import FileContent, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ''use_sim_time'' is used to have ros2 use /clock topic for the time source
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf = FileContent(
        PathJoinSubstitution([FindPackageShare('robot_description'), 'urdf', 'r2d2.urdf.xml']))

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': urdf}],
            arguments=[urdf]),
        Node(
            package='robot_description',
            executable='robot_description',
            name='robot_description',
            output='screen'),
    ])