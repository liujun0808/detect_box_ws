from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_parameters = PathJoinSubstitution(
        [FindPackageShare("detect_pkg"), "config", "box_position_estimation.yaml"]
    )
    parameters_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_parameters,
            description="ROS 2 YAML file containing all detection parameters.",
        ),
        Node(
            package="detect_pkg",
            executable="detect_server_node",
            name="detect_server_node",
            output="screen",
            emulate_tty=True,
            parameters=[parameters_file],
        ),
    ])
