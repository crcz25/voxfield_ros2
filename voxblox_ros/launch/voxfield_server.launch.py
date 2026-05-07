from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("voxblox_ros")
    default_params = PathJoinSubstitution(
        [pkg_share, "config", "voxfield_server.yaml"]
    )

    return LaunchDescription(
        [
            # Path to the ROS 2 YAML parameter file.
            # Override with: params_file:=/path/to/your_params.yaml
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to the voxfield_server YAML parameter file.",
            ),

            # Main pointcloud input topic. Remapped to the node's 'pointcloud'
            # subscriber. Override with: pointcloud_topic:=/your/cloud/topic
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="pointcloud",
                description="Input pointcloud topic (sensor_msgs/PointCloud2).",
            ),

            # Freespace pointcloud topic. Only subscribed to when
            # use_freespace_pointcloud is true in the YAML.
            DeclareLaunchArgument(
                "freespace_pointcloud_topic",
                default_value="freespace_pointcloud",
                description=(
                    "Freespace pointcloud topic. Active only when "
                    "use_freespace_pointcloud: true in the YAML."
                ),
            ),

            # Global map frame. Must match world_frame in the YAML if not
            # overriding here. All incoming clouds are transformed into this frame.
            DeclareLaunchArgument(
                "world_frame",
                default_value="odom",
                description="Global map frame for TSDF/ESDF integration.",
            ),

            # Sensor frame override. Empty string = use the frame_id from the
            # incoming pointcloud header (recommended default).
            DeclareLaunchArgument(
                "sensor_frame",
                default_value="",
                description=(
                    "Override for the sensor TF frame. "
                    "Empty = use pointcloud header frame_id."
                ),
            ),

            # Use /clock from a rosbag or simulator instead of wall time.
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time (/clock topic).",
            ),

            DeclareLaunchArgument(
                "publish_traversable",
                default_value="false",
                description="Publish /voxfield/traversable pointcloud output.",
            ),

            DeclareLaunchArgument(
                "eval_esdf_on",
                default_value="false",
                description=(
                    "Enable ESDF error evaluation and publish "
                    "/voxfield/esdf_error_slice."
                ),
            ),

            DeclareLaunchArgument(
                "publish_robot_model",
                default_value="false",
                description="Publish the /voxfield/Robot_model mesh marker.",
            ),

            DeclareLaunchArgument(
                "robot_model_file",
                default_value="package://voxblox_ros/cfg/model/camera.dae",
                description=(
                    "Robot mesh resource URI or absolute path. Used when "
                    "publish_robot_model is true."
                ),
            ),

            Node(
                package="voxblox_ros",
                executable="voxfield_server",
                name="voxfield",
                output="screen",
                parameters=[
                    # Load the full YAML parameter file first.
                    LaunchConfiguration("params_file"),
                    # Override specific parameters from launch arguments.
                    # These take precedence over the YAML values.
                    {
                        "world_frame": LaunchConfiguration("world_frame"),
                        "sensor_frame": LaunchConfiguration("sensor_frame"),
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "publish_traversable": ParameterValue(
                            LaunchConfiguration("publish_traversable"),
                            value_type=bool,
                        ),
                        "eval_esdf_on": ParameterValue(
                            LaunchConfiguration("eval_esdf_on"), value_type=bool
                        ),
                        "publish_robot_model": ParameterValue(
                            LaunchConfiguration("publish_robot_model"),
                            value_type=bool,
                        ),
                        "robot_model_file": LaunchConfiguration(
                            "robot_model_file"
                        ),
                    },
                ],
                remappings=[
                    ("pointcloud", LaunchConfiguration("pointcloud_topic")),
                    (
                        "freespace_pointcloud",
                        LaunchConfiguration("freespace_pointcloud_topic"),
                    ),
                ],
            ),
        ]
    )
