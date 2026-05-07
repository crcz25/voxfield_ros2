# ROS 2 Porting Notes

This repository is being ported in-place from ROS 1/catkin to ROS 2 Jazzy/ament.
The repository folder remains `voxfield_ros2`; package names are preserved.

## Current Package Structure

- `voxblox_msgs`: Voxblox map, mesh, and utility message/service definitions.
- `voxblox`: ROS-independent core TSDF, non-projective TSDF, ESDF, Voxfield ESDF,
  FIESTA, EDT, mesh, IO, simulation, and utility library code.
- `voxblox_ros`: ROS runtime wrappers, servers, point cloud conversion,
  visualization helpers, TF handling, launch/configuration.
- `voxblox_rviz_plugin`: RViz2 display plugin package for Voxblox/Voxfield mesh
  messages.

## ROS 1 Dependencies Found

- Build: `catkin`, `catkin_simple`, `message_generation`, `message_runtime`.
- Core wrappers: `eigen_catkin`, `gflags_catkin`, `glog_catkin`,
  `protobuf_catkin`, `minkindr`.
- Runtime ROS: `roscpp`, `ros/ros.h`, `tf`, `tf::TransformListener`,
  `tf::TransformBroadcaster`, `sensor_msgs`, `geometry_msgs`,
  `visualization_msgs`, `std_msgs`, `std_srvs`, `nav_msgs`.
- Point clouds: `pcl_ros`, `pcl_conversions`, PCL.
- Conversions/config: `minkindr_conversions`, `XmlRpc`.
- Optional tools: `cv_bridge`, `interactive_markers`, RViz 1 plugin APIs.

## ROS 2 Replacements

- Build: `ament_cmake`, `rosidl_default_generators`,
  `rosidl_default_runtime`.
- Core/system: `Eigen3`, `gflags`, `glog`, `Protobuf`.
- Runtime ROS: `rclcpp`, ROS 2 message namespaces, `tf2_ros`, `tf2_eigen`,
  `pcl_conversions`, PCL.
- Services: ROS 2 service callback signatures using
  `std_srvs::srv::*` and `voxblox_msgs::srv::*`.
- Parameters: declared node-local ROS 2 parameters.
- Launch: Python launch files installed through `ament_cmake`.

## Transformation Strategy

No ROS 2 `minkindr`/`minkindr_conversions` package is installed in this
environment. The least invasive approach is:

- keep the core `voxblox::Transformation` API intact through a small local
  compatibility implementation of the subset of
  `kindr::minimal::QuatTransformationTemplate` used by Voxfield;
- replace ROS conversion-only usage in `voxblox_ros` with ROS 2
  `geometry_msgs`, TF2, and Eigen conversions;
- preserve transform semantics: timestamped lookup, configured sensor frame,
  queued transform fallback when TF lookup is disabled, and static calibration
  transforms.

## Files Changed

This section is updated as the port progresses.

- `ROS2_PORTING_NOTES.md`: created migration notes and validation guidance.
- `voxblox_msgs/package.xml`: converted to ROS 2 package format.
- `voxblox_msgs/CMakeLists.txt`: replaced catkin-simple with
  `ament_cmake` and `rosidl_generate_interfaces`.
- `voxblox/package.xml`: converted to ROS 2 package format with system
  Eigen/gflags/glog/protobuf dependencies.
- `voxblox/CMakeLists.txt`: replaced catkin-simple and protobuf-catkin macros
  with target-based ament CMake and direct `protoc` generation.
- `voxblox/include/kindr/minimal/quat-transformation.h`: added a minimal local
  compatibility implementation for the subset of kindr used by the core
  library.
- `voxblox_ros/package.xml`: converted to ROS 2 package format.
- `voxblox_ros/CMakeLists.txt`: converted to ament and limited the default
  runtime build to the primary mapping path: `transformer`,
  `np_tsdf_server`, `voxfield_server`, and the `np_tsdf_server` /
  `voxfield_server` executables.
- `voxblox_ros/include/ros`, `sensor_msgs`, `std_msgs`, `std_srvs`,
  `geometry_msgs`, `visualization_msgs`, `tf`, `pcl_ros`, and
  `voxblox_msgs`: added temporary ROS 2 compatibility headers for the remaining
  ROS 1-style runtime plumbing while using ROS 2 message/service types.
- `voxblox_ros/src/transformer.cc`: removed `minkindr_conversions` and
  `XmlRpc` use; TF lookup now resolves through ROS 2 TF2 compatibility.
- `voxblox_ros/include/voxblox_ros/*` and `voxblox_ros/src/*`: mechanically
  moved message/service names to ROS 2 namespaces.
- `voxblox_ros/include/ros/ros.h`: fixed ROS 2 publisher, time, and duration
  compatibility needed by the ported server code.
- `voxblox_ros/config/voxfield_server.yaml`: added ROS 2 parameter file for the
  primary Voxfield server.
- `voxblox_ros/launch/voxfield_server.launch.py`: added ROS 2 Python launch
  file.
- `voxblox_rviz_plugin/package.xml`: converted to ROS 2 package format 3 with
  RViz2, pluginlib, Qt, and Voxblox message dependencies.
- `voxblox_rviz_plugin/CMakeLists.txt`: converted from catkin-simple to
  target-based ament CMake, C++17, automoc, shared-library install, RViz2 plugin
  export, and OGRE media export.
- `voxblox_rviz_plugin/plugins_description.xml`: added ROS 2 pluginlib metadata
  for the Voxblox mesh and multi-mesh displays.
- `voxblox_rviz_plugin/include/voxblox_rviz_plugin/*` and
  `voxblox_rviz_plugin/src/*`: ported display classes from RViz 1 APIs to
  RViz2 `rviz_common` APIs and ROS 2 generated message namespaces.

## Features Ported

- `voxblox_msgs` builds as ROS 2 interfaces and validates with
  `ros2 interface show`.
- Core `voxblox` builds with ament and direct protobuf generation.
- `minkindr` core transform usage is preserved through a local compatibility
  header.
- `voxblox_ros` has a ROS 2 ament build path for `np_tsdf_server` and
  `voxfield_server`.
- Point cloud subscriptions use ROS 2 `sensor_msgs::msg::PointCloud2` types.
- Map, mesh, marker, and service types have been moved to ROS 2 namespaces.
- A ROS 2 launch file and YAML parameter file have been added.
- `voxblox_rviz_plugin` builds and exports RViz2 display plugins for
  `voxblox_msgs/msg/Mesh` and `voxblox_msgs/msg/MultiMesh`.
- `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`
  succeeds for `voxblox`, `voxblox_msgs`, `voxblox_ros`, and
  `voxblox_rviz_plugin`.

## RViz2 Plugin Port

Original ROS 1 plugin classes found:

- `voxblox_rviz_plugin::VoxbloxMeshDisplay`, exported as
  `voxblox_rviz_plugin/VoxbloxMesh`, displayed `voxblox_msgs/Mesh`.
- `voxblox_rviz_plugin::VoxbloxMultiMeshDisplay`, exported as
  `voxblox_rviz_plugin/VoxbloxMultiMesh`, displayed `voxblox_msgs/MultiMesh`.
- `voxblox_rviz_plugin::VoxbloxMeshVisual`, shared OGRE manual-object renderer
  for block-wise incremental mesh updates.
- `voxblox_rviz_plugin::MaterialLoader`, custom OGRE material loader.

ROS 2 replacement APIs used:

- `rviz::MessageFilterDisplay<T>` -> `rviz_common::MessageFilterDisplay<T>` to
  preserve TF-aware filtering, topic properties, and QoS controls.
- `rviz::BoolProperty` / `rviz::StatusProperty` ->
  `rviz_common::properties::BoolProperty` /
  `rviz_common::properties::StatusProperty`.
- ROS 1 message headers such as `voxblox_msgs/Mesh.h` ->
  `voxblox_msgs/msg/mesh.hpp` and `voxblox_msgs::msg::*`.
- `ros::Time` -> `rclcpp::Time`.
- `ros::package::getPath` -> `ament_index_cpp::get_package_share_directory`.
- `pluginlib/class_list_macros.h` ->
  `pluginlib/class_list_macros.hpp`, with exports using
  `rviz_common::Display` as the pluginlib base.

Files changed:

- `voxblox_rviz_plugin/package.xml`
- `voxblox_rviz_plugin/CMakeLists.txt`
- `voxblox_rviz_plugin/plugins_description.xml`
- `voxblox_rviz_plugin/include/voxblox_rviz_plugin/voxblox_mesh_display.h`
- `voxblox_rviz_plugin/include/voxblox_rviz_plugin/voxblox_multi_mesh_display.h`
- `voxblox_rviz_plugin/include/voxblox_rviz_plugin/voxblox_mesh_visual.h`
- `voxblox_rviz_plugin/src/material_loader.cc`
- `voxblox_rviz_plugin/src/voxblox_mesh_display.cc`
- `voxblox_rviz_plugin/src/voxblox_multi_mesh_display.cc`
- `voxblox_rviz_plugin/src/voxblox_mesh_visual.cc`

Plugin XML changes:

- Replaced ROS 1 `plugin_description.xml` with
  `plugins_description.xml`.
- Library path is now `voxblox_rviz_plugin`, matching ROS 2 pluginlib
  conventions.
- Base class type is `rviz_common::Display`.
- Message type tags are `voxblox_msgs/msg/Mesh` and
  `voxblox_msgs/msg/MultiMesh`.
- CMake exports the XML with
  `pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)`.

Build instructions:

```bash
cd /workspace/3dsg_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

RViz2 test instructions:

```bash
source /workspace/3dsg_ws/install/setup.bash
ros2 run pluginlib list_plugins rviz_common rviz_common::Display | grep voxblox
rviz2
```

In RViz2, click `Add`, select `voxblox_rviz_plugin/VoxbloxMesh`, set the mesh
topic to `/mesh` or the server's configured mesh topic, and set the fixed frame
to the mapping frame such as `map` or `odom`.

Runtime Voxfield test:

```bash
cd /workspace/3dsg_ws
source install/setup.bash
ros2 launch voxblox_ros voxfield_server.launch.py
```

## ROS 2 Jazzy Memory/Shutdown Fix

Root cause found during the Jazzy port audit:

- The ROS 1 Voxfield launch/config files use bounded ESDF local range offsets
  such as `local_range_offset_x: 20`, `local_range_offset_y: 20`, and
  `local_range_offset_z: 10` for RGB-D data.
- The ROS 2 `voxblox_ros/config/voxfield_server.yaml` had been authored with
  `local_range_offset_x/y/z: 10000`. These offsets are voxel indices, not
  metres. `EsdfVoxfieldIntegrator::setLocalRange()` allocates every ESDF block
  in the expanded update box, so the first ESDF update attempted an effectively
  unbounded allocation while the TSDF `Layer memory` log stayed constant.
- The ROS 2 point cloud compatibility subscriber also used generic reliable QoS
  and the default YAML set `pointcloud_queue_size: 50`, which is inappropriate
  for high-rate 300k-point sensor clouds.

Fixes added:

- `voxblox_ros/config/voxfield_server.yaml` now matches the bounded ROS 1 local
  ESDF range defaults and uses `pointcloud_queue_size: 1`.
- `voxblox_ros/include/ros/ros.h` maps `sensor_msgs/msg/PointCloud2`
  subscriptions to bounded best-effort volatile QoS, while map/layer
  subscriptions remain reliable.
- `EsdfVoxfieldIntegrator` now has `max_esdf_blocks_per_update` and refuses a
  pathological local range allocation before allocating blocks.
- `voxfield_server` and `np_tsdf_server` cancel timers and drop local point cloud
  queues during shutdown; the ESDF integrator checks an abort callback inside
  long update loops so `Ctrl+C` can terminate cleanly.
- Periodic memory diagnostics include process RSS, TSDF/ESDF block counts,
  TSDF/ESDF layer memory, local queue sizes, and ESDF update-list sizes.

Useful verification commands:

```bash
cd /mnt/DATA/repos/phd/voxfield_ros2
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/voxfield_ros2 --packages-select voxblox voxblox_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ros2 launch voxblox_ros voxfield_server.launch.py \
  pointcloud_topic:=/intel_realsense_r200_depth/points \
  world_frame:=odom \
  sensor_frame:=realsense_depth_frame
```

AddressSanitizer/LeakSanitizer build:

```bash
colcon build --base-paths src/voxfield_ros2 --packages-select voxblox voxblox_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DENABLE_ASAN=ON
```

External RSS checks:

```bash
pid=$(pgrep -f 'voxfield_server')
watch -n 1 "grep -E 'VmRSS|VmHWM' /proc/$pid/status"
/usr/bin/time -v ros2 launch voxblox_ros voxfield_server.launch.py \
  pointcloud_topic:=/intel_realsense_r200_depth/points \
  world_frame:=odom sensor_frame:=realsense_depth_frame
```

Then open RViz2 from another sourced terminal, add the Voxblox mesh display, and
select the server's mesh topic.

Known limitations:

- The single-mesh display is the primary Voxfield path and subscribes to
  `voxblox_msgs/msg/Mesh`.
- The multi-mesh display was also ported because `voxblox_msgs/msg/MultiMesh`
  exists in this workspace, but it has not been exercised against a live
  multi-submap ROS 2 publisher in this session.
- GUI RViz2 rendering was not manually validated in this headless session; the
  package builds, the shared library resolves, and pluginlib discovers both
  display classes.
- The visual still reuses the original Voxblox mesh conversion helper, so
  `voxblox_rviz_plugin` depends on the core `voxblox` library in addition to
  the generated message package.

## Features Deferred

- Legacy TSDF, Voxblox ESDF, FIESTA, EDT, intensity, simulation, visualization,
  and evaluation executables are not part of the default `voxblox_ros` build
  yet. Their algorithm files are preserved; bringing each executable fully onto
  native ROS 2 APIs is separate follow-up work after the primary Voxfield path
  builds.

## Build Instructions

Target environment:

```bash
cd /workspace/3dsg_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

This session's workspace path is `/home/crcz/repos/voxfield_ws`, while the
requested deployment path is `/workspace/3dsg_ws`.

On Linux Mint hosts that should resolve dependencies as Ubuntu 24.04/Jazzy,
use:

```bash
rosdep install --from-paths . --ignore-src --rosdistro jazzy --os=ubuntu:noble -r -y
```

## Run Instructions

```bash
source install/setup.bash
ros2 launch voxblox_ros voxfield_server.launch.py
```

The launch file will load `voxblox_ros/config/voxfield_server.yaml` and can
remap the `pointcloud` input topic.

## Smoke Test

1. Start a static transform from `world_frame` to `sensor_frame`.
2. Run `voxfield_server` with the provided YAML file.
3. Publish a small `sensor_msgs/msg/PointCloud2` on `pointcloud`.
4. Confirm the node appears in `ros2 node list`, subscribes to `pointcloud`,
   exposes map services, and publishes at least one output topic.

## Known Limitations

- The port prioritizes a working mapping stack over feature completeness.
- Compatibility of all historical launch files and offline evaluation tools is
  not guaranteed until the primary servers build and run.
- This local session is Linux Mint based and is missing `libprotobuf-dev`,
  `protobuf-compiler`, and `libprotoc-dev`. With
  `--os=ubuntu:noble`, rosdep resolves them correctly, but installation could
  not proceed here because sudo requires an interactive password.
- `voxblox_ros` still uses a temporary compatibility layer for some ROS 1
  NodeHandle-style calls. It is backed by ROS 2 `rclcpp`, TF2, and ROS 2
  messages, but a deeper cleanup can replace those shims with direct native
  `rclcpp::Node` APIs throughout the server classes.

## voxblox_ros Parameters and Launch Configuration

### Files updated

- **`voxblox_ros/config/voxfield_server.yaml`** — Complete parameter file for
  the `voxfield_server` node.  Every parameter read from ROS params by
  `TsdfServer`, `NpTsdfServer`, `VoxfieldServer`, `Transformer`, and all
  integrator config helpers in `ros_params.h` is present.  Safe defaults are
  enabled; advanced, unsafe, or context-dependent parameters are included as
  commented-out examples so they remain visible and can be activated without
  having to search the source code.

- **`voxblox_ros/launch/voxfield_server.launch.py`** — ROS 2 Python launch
  file for the `voxfield_server` executable.  Updated to expose commonly
  overridden parameters as launch arguments and to add the
  `freespace_pointcloud` remapping.

### How to run

```bash
source install/setup.bash
ros2 launch voxblox_ros voxfield_server.launch.py
```

### Overriding parameters from the command line

Launch arguments override the YAML file:

```bash
ros2 launch voxblox_ros voxfield_server.launch.py \
  world_frame:=map \
  pointcloud_topic:=/lidar/points \
  use_sim_time:=true \
  params_file:=/path/to/custom_params.yaml
```

Available launch arguments:

| Argument | Default | Description |
| --- | --- | --- |
| `params_file` | package `config/voxfield_server.yaml` | Full YAML parameter file path |
| `pointcloud_topic` | `pointcloud` | Main input point cloud topic |
| `freespace_pointcloud_topic` | `freespace_pointcloud` | Freespace cloud topic (needs `use_freespace_pointcloud: true` in YAML) |
| `world_frame` | `odom` | Global map frame for TSDF/ESDF integration |
| `sensor_frame` | `""` | Override sensor TF frame; empty = use cloud header frame_id |
| `use_sim_time` | `false` | Use `/clock` from a bag or simulator |

### Parameter inventory summary

All parameters are grounded in the following source files:

| Source | Parameters |
| --- | --- |
| `transformer.cc` | `world_frame`, `sensor_frame`, `use_tf_transforms`, `timestamp_tolerance_sec`, `T_B_D/C/CH`, `invert_T_B_D/C/CH` |
| `ros_params.h` `getTsdfMapConfigFromRosParam` | `tsdf_voxel_size`, `tsdf_voxels_per_side` |
| `ros_params.h` `getNpTsdfIntegratorConfigFromRosParam` | `truncation_distance`, `voxel_carving_enabled`, `max/min_ray_length_m`, `max_weight`, `use_const_weight`, `weight_reduction_exp`, `use_weight_dropoff`, `weight_dropoff_epsilon`, `allow_clear`, `start_voxel_subsampling_factor`, `max_consecutive_ray_collisions`, `clear_checks_every_n_frames`, `max_integration_time_s`, `anti_grazing`, `use_sparsity_compensation_factor`, `sparsity_compensation_factor`, `integration_order_mode`, `integrator_threads`, `merge_with_clear`, `normal_available`, `reliable_band_ratio`, `curve_assumption`, `reliable_normal_ratio_thre` |
| `ros_params.h` `getEsdfVoxfieldIntegratorConfigFromRosParam` | `local_range_offset_x/y/z`, `esdf_max_distance_m`, `esdf_default_distance_m`, `fix_band_distance_m`, `max_behind_surface_m`, `occ_min_weight`, `occ_voxel_size_ratio`, `num_buckets`, `patch_on`, `early_break`, `finer_esdf_on` |
| `ros_params.h` `getMeshIntegratorConfigFromRosParam` | `mesh_min_weight`, `mesh_use_color` |
| `ros_params.h` `getICPConfigFromRosParam` | `icp_min_match_ratio`, `icp_subsample_keep_ratio`, `icp_mini_batch_size`, `icp_refine_roll_pitch`, `icp_inital_translation_weighting`, `icp_inital_rotation_weighting` |
| `np_tsdf_server.cc` `getServerConfigFromRosParam` | `min_time_between_msgs_sec`, `max_block_distance_from_body`, `slice_level`, `publish_pointclouds_on_update`, `publish_slices`, `publish_pointclouds`, `use_freespace_pointcloud`, `pointcloud_queue_size`, `enable_icp`, `accumulate_icp_corrections`, `verbose`, `timing`, `sensor_is_lidar`, `width`, `height`, `smooth_thre_ratio`, `min_z`, `min_dist`, `vx`, `vy`, `fx`, `fy`, `fov_up`, `fov_down`, `publish_robot_model`, `robot_model_file`, `robot_model_scale`, `mesh_filename`, `color_mode`, `intensity_colormap`, `intensity_max_value` |
| `tsdf_server.cc` / `np_tsdf_server.cc` | `method`, `publish_tsdf_map`, `update_mesh_every_n_sec`, `publish_map_every_n_sec`, `icp_corrected_frame`, `pose_corrected_frame` |
| `voxfield_server.cc` `setupRos` | `clear_sphere_for_planning`, `publish_esdf_map`, `publish_traversable`, `traversability_radius`, `update_esdf_every_n_sec`, `eval_esdf_on`, `eval_esdf_every_n_sec` |

### Parameters intentionally commented out and why

| Parameter | Reason commented out |
| --- | --- |
| `sensor_frame` | Empty string is the correct default; setting a wrong value blocks all TF lookups |
| `fov_up`, `fov_down` | Only read when `sensor_is_lidar: true`; included below camera params for visibility |
| `icp_corrected_frame`, `pose_corrected_frame` | Only meaningful when `enable_icp: true` |
| `icp_min_match_ratio` … `icp_inital_rotation_weighting` | Only active when `enable_icp: true`; values shown are source defaults |
| `mesh_filename` | Writing a PLY file is opt-in; an incorrect path silently fails |
| `T_B_D`, `invert_T_B_D`, `T_B_C`, `invert_T_B_C` | Only read when `use_tf_transforms: false`; including them when TF is enabled is harmless but misleading |
| `invert_T_C_CH` | Rarely needed; identity default shown as example |

### Notes on removed YAML entries

The previous `voxfield_server.yaml` contained `voxel_size` and
`voxels_per_side` entries that are **not read by any server code** in this
package.  The correct parameter names are `tsdf_voxel_size` and
`tsdf_voxels_per_side` (as declared in `ros_params.h`).  Those stale entries
have been removed to avoid silent misconfiguration.
