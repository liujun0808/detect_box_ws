from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    service_name = LaunchConfiguration("service_name")
    enable_debug_image = LaunchConfiguration("enable_debug_image")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    camera_namespace = LaunchConfiguration("camera_namespace")
    camera_name = LaunchConfiguration("camera_name")
    serial_no = LaunchConfiguration("serial_no")
    color_profile = LaunchConfiguration("color_profile")

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        launch_arguments={
            "camera_name": camera_name,
            "camera_namespace": camera_namespace,
            "serial_no": serial_no,
            "enable_color": "true",
            "enable_depth": "false",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "rgb_camera.color_profile": color_profile,
        }.items(),
    )

    detect_server = Node(
        package="detect_pkg",
        executable="detect_server_node",
        name="detect_server_node",
        output="screen",
        parameters=[
            {
                "service_name": service_name,
                "image_topic": image_topic,
                "camera_info_topic": camera_info_topic,
                "enable_debug_image": enable_debug_image,
                "image_qos": "reliable",
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("service_name", default_value="detect"),
            DeclareLaunchArgument("enable_debug_image", default_value="false"),
            DeclareLaunchArgument("camera_namespace", default_value="camera"),
            DeclareLaunchArgument("camera_name", default_value="camera"),
            DeclareLaunchArgument("serial_no", default_value=""),
            DeclareLaunchArgument(
                "image_topic", default_value="/camera/camera/color/image_raw"
            ),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/camera/color/camera_info"
            ),
            DeclareLaunchArgument("color_profile", default_value="640x480x15"),
            realsense_launch,
            detect_server,
        ]
    )
