from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare("stl50b2_driver"),
        "config",
        "stl50b2.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to the driver parameter file",
        ),
        Node(
            package="stl50b2_driver",
            executable="stl50b2_node",
            name="stl50b2_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
