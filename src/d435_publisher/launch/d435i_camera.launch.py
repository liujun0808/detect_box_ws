from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    camera_namespace = LaunchConfiguration("camera_namespace")
    serial_no = LaunchConfiguration("serial_no")

    publish_camera_extrinsic_tf = LaunchConfiguration("publish_camera_extrinsic_tf")
    base_frame = LaunchConfiguration("base_frame")
    camera_frame = LaunchConfiguration("camera_frame")

    realsense_launch = PathJoinSubstitution([
        FindPackageShare("realsense2_camera"),
        "launch",
        "rs_launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_namespace",
            default_value="camera",
            description="RealSense namespace. Empty avoids duplicated /camera/camera topics.",
        ),
        DeclareLaunchArgument(
            "serial_no",
            default_value="",
            description="RealSense serial number. Empty selects the first connected camera.",
        ),
        DeclareLaunchArgument(
            "publish_camera_extrinsic_tf",
            default_value="true",
            description="Publish the fixed base_link -> D435 camera extrinsic TF.",
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="base_link",
            description="Parent frame for the D435 camera extrinsic TF.",
        ),
        DeclareLaunchArgument(
            "camera_frame",
            default_value="d435_camera",
            description="Child camera frame for the D435 camera extrinsic TF.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={
                "camera_namespace": camera_namespace,
                "serial_no": serial_no,
                "enable_color": "true",
                "enable_depth": "true",
                "enable_infra1": "false",
                "enable_infra2": "false",
                "enable_gyro": "false",
                "enable_accel": "false",
                "enable_sync": "true",
                "align_depth.enable": "true",
                "pointcloud.enable": "false",
                "rgb_camera.color_profile": "640x480x15",
                "depth_module.depth_profile": "640x480x15",
                "base_frame_id": "d435_camera",
                "tf_prefix": "d435_camera",
            }.items(),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="d435_camera_extrinsic_tf_publisher",
            output="screen",
            arguments=[
                "--x",
                "0.12972",
                "--y",
                "0.0325000",
                "--z",
                "0.24561",
                "--qx",
                "-0.469846",
                "--qy",
                "0.469846",
                "--qz",
                "-0.529592",
                "--qw",
                "0.529592",
                "--frame-id",
                base_frame,
                "--child-frame-id",
                camera_frame,
            ],
            condition=IfCondition(publish_camera_extrinsic_tf),
        ),
        
    ])
