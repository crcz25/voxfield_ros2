from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("voxblox_ros")
    default_params = PathJoinSubstitution(
        [pkg_share, "config", "voxfield_server.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("pointcloud_topic", default_value="pointcloud"),
            DeclareLaunchArgument("world_frame", default_value="world"),
            DeclareLaunchArgument("sensor_frame", default_value="sensor"),
            Node(
                package="voxblox_ros",
                executable="voxfield_server",
                name="voxfield",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "world_frame": LaunchConfiguration("world_frame"),
                        "sensor_frame": LaunchConfiguration("sensor_frame"),
                    },
                ],
                remappings=[
                    ("pointcloud", LaunchConfiguration("pointcloud_topic")),
                ],
            ),
        ]
    )
