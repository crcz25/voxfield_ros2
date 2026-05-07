#include "voxblox_ros/ros_params.h"

#include <algorithm>
#include <cmath>
#include <thread>

namespace voxblox {

TsdfMap::Config getTsdfMapConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  TsdfMap::Config tsdf_config;

  double voxel_size = tsdf_config.tsdf_voxel_size;
  int voxels_per_side = tsdf_config.tsdf_voxels_per_side;
  nh_private.param("tsdf_voxel_size", voxel_size, voxel_size);
  nh_private.param("tsdf_voxels_per_side", voxels_per_side, voxels_per_side);
  if (!isPowerOfTwo(voxels_per_side)) {
    ROS_ERROR("voxels_per_side must be a power of 2, setting to default value");
    voxels_per_side = tsdf_config.tsdf_voxels_per_side;
  }

  tsdf_config.tsdf_voxel_size = static_cast<FloatingPoint>(voxel_size);
  tsdf_config.tsdf_voxels_per_side = voxels_per_side;
  return tsdf_config;
}

ICP::Config getICPConfigFromRosParam(const ros::NodeHandle& nh_private) {
  ICP::Config icp_config;

  nh_private.param(
      "icp_min_match_ratio", icp_config.min_match_ratio,
      icp_config.min_match_ratio);
  nh_private.param(
      "icp_subsample_keep_ratio", icp_config.subsample_keep_ratio,
      icp_config.subsample_keep_ratio);
  nh_private.param(
      "icp_mini_batch_size", icp_config.mini_batch_size,
      icp_config.mini_batch_size);
  nh_private.param(
      "icp_refine_roll_pitch", icp_config.refine_roll_pitch,
      icp_config.refine_roll_pitch);
  nh_private.param(
      "icp_inital_translation_weighting",
      icp_config.inital_translation_weighting,
      icp_config.inital_translation_weighting);
  nh_private.param(
      "icp_inital_rotation_weighting", icp_config.inital_rotation_weighting,
      icp_config.inital_rotation_weighting);

  return icp_config;
}

TsdfIntegratorBase::Config getTsdfIntegratorConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  TsdfIntegratorBase::Config integrator_config;
  integrator_config.voxel_carving_enabled = true;

  const TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam(nh_private);
  double max_weight = integrator_config.max_weight;
  float truncation_distance = -2.0f;

  nh_private.param(
      "truncation_distance", truncation_distance, truncation_distance);
  integrator_config.default_truncation_distance =
      truncation_distance > 0
          ? truncation_distance
          : -truncation_distance * tsdf_config.tsdf_voxel_size;

  nh_private.param(
      "voxel_carving_enabled", integrator_config.voxel_carving_enabled,
      integrator_config.voxel_carving_enabled);
  nh_private.param(
      "max_ray_length_m", integrator_config.max_ray_length_m,
      integrator_config.max_ray_length_m);
  nh_private.param(
      "min_ray_length_m", integrator_config.min_ray_length_m,
      integrator_config.min_ray_length_m);
  nh_private.param("max_weight", max_weight, max_weight);
  integrator_config.max_weight = static_cast<float>(max_weight);
  nh_private.param(
      "use_const_weight", integrator_config.use_const_weight,
      integrator_config.use_const_weight);
  nh_private.param(
      "use_weight_dropoff", integrator_config.use_weight_dropoff,
      integrator_config.use_weight_dropoff);
  nh_private.param(
      "allow_clear", integrator_config.allow_clear,
      integrator_config.allow_clear);
  nh_private.param(
      "start_voxel_subsampling_factor",
      integrator_config.start_voxel_subsampling_factor,
      integrator_config.start_voxel_subsampling_factor);
  nh_private.param(
      "max_consecutive_ray_collisions",
      integrator_config.max_consecutive_ray_collisions,
      integrator_config.max_consecutive_ray_collisions);
  nh_private.param(
      "clear_checks_every_n_frames",
      integrator_config.clear_checks_every_n_frames,
      integrator_config.clear_checks_every_n_frames);
  nh_private.param(
      "max_integration_time_s", integrator_config.max_integration_time_s,
      integrator_config.max_integration_time_s);
  nh_private.param(
      "anti_grazing", integrator_config.enable_anti_grazing,
      integrator_config.enable_anti_grazing);
  nh_private.param(
      "use_sparsity_compensation_factor",
      integrator_config.use_sparsity_compensation_factor,
      integrator_config.use_sparsity_compensation_factor);
  nh_private.param(
      "sparsity_compensation_factor",
      integrator_config.sparsity_compensation_factor,
      integrator_config.sparsity_compensation_factor);
  nh_private.param(
      "integration_order_mode", integrator_config.integration_order_mode,
      integrator_config.integration_order_mode);
  float integrator_threads = std::thread::hardware_concurrency();
  nh_private.param(
      "integrator_threads", integrator_threads, integrator_threads);
  integrator_config.integrator_threads = static_cast<int>(integrator_threads);
  nh_private.param(
      "merge_with_clear", integrator_config.merge_with_clear,
      integrator_config.merge_with_clear);

  return integrator_config;
}

NpTsdfIntegratorBase::Config getNpTsdfIntegratorConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  NpTsdfIntegratorBase::Config integrator_config;
  integrator_config.voxel_carving_enabled = true;

  const TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam(nh_private);
  double max_weight = integrator_config.max_weight;
  float truncation_distance = -2.0f;

  nh_private.param(
      "truncation_distance", truncation_distance, truncation_distance);
  integrator_config.default_truncation_distance =
      truncation_distance > 0
          ? truncation_distance
          : -truncation_distance * tsdf_config.tsdf_voxel_size;

  nh_private.param(
      "voxel_carving_enabled", integrator_config.voxel_carving_enabled,
      integrator_config.voxel_carving_enabled);
  nh_private.param(
      "max_ray_length_m", integrator_config.max_ray_length_m,
      integrator_config.max_ray_length_m);
  nh_private.param(
      "min_ray_length_m", integrator_config.min_ray_length_m,
      integrator_config.min_ray_length_m);
  nh_private.param("max_weight", max_weight, max_weight);
  integrator_config.max_weight = static_cast<float>(max_weight);
  nh_private.param(
      "use_const_weight", integrator_config.use_const_weight,
      integrator_config.use_const_weight);
  nh_private.param(
      "weight_reduction_exp", integrator_config.weight_reduction_exp,
      integrator_config.weight_reduction_exp);
  nh_private.param(
      "use_weight_dropoff", integrator_config.use_weight_dropoff,
      integrator_config.use_weight_dropoff);
  nh_private.param(
      "weight_dropoff_epsilon", integrator_config.weight_dropoff_epsilon,
      integrator_config.weight_dropoff_epsilon);
  nh_private.param(
      "allow_clear", integrator_config.allow_clear,
      integrator_config.allow_clear);
  nh_private.param(
      "start_voxel_subsampling_factor",
      integrator_config.start_voxel_subsampling_factor,
      integrator_config.start_voxel_subsampling_factor);
  nh_private.param(
      "max_consecutive_ray_collisions",
      integrator_config.max_consecutive_ray_collisions,
      integrator_config.max_consecutive_ray_collisions);
  nh_private.param(
      "clear_checks_every_n_frames",
      integrator_config.clear_checks_every_n_frames,
      integrator_config.clear_checks_every_n_frames);
  nh_private.param(
      "max_integration_time_s", integrator_config.max_integration_time_s,
      integrator_config.max_integration_time_s);
  nh_private.param(
      "anti_grazing", integrator_config.enable_anti_grazing,
      integrator_config.enable_anti_grazing);
  nh_private.param(
      "use_sparsity_compensation_factor",
      integrator_config.use_sparsity_compensation_factor,
      integrator_config.use_sparsity_compensation_factor);
  nh_private.param(
      "sparsity_compensation_factor",
      integrator_config.sparsity_compensation_factor,
      integrator_config.sparsity_compensation_factor);
  nh_private.param(
      "integration_order_mode", integrator_config.integration_order_mode,
      integrator_config.integration_order_mode);
  float integrator_threads = std::thread::hardware_concurrency();
  nh_private.param(
      "integrator_threads", integrator_threads, integrator_threads);
  integrator_config.integrator_threads = static_cast<int>(integrator_threads);
  nh_private.param(
      "merge_with_clear", integrator_config.merge_with_clear,
      integrator_config.merge_with_clear);
  nh_private.param(
      "normal_available", integrator_config.normal_available,
      integrator_config.normal_available);
  nh_private.param(
      "reliable_band_ratio", integrator_config.reliable_band_ratio,
      integrator_config.reliable_band_ratio);
  nh_private.param(
      "curve_assumption", integrator_config.curve_assumption,
      integrator_config.curve_assumption);
  nh_private.param(
      "reliable_normal_ratio_thre",
      integrator_config.reliable_normal_ratio_thre,
      integrator_config.reliable_normal_ratio_thre);

  return integrator_config;
}

EsdfMap::Config getEsdfMapConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  EsdfMap::Config esdf_config;
  const TsdfMap::Config tsdf_config = getTsdfMapConfigFromRosParam(nh_private);
  esdf_config.esdf_voxel_size = tsdf_config.tsdf_voxel_size;
  esdf_config.esdf_voxels_per_side = tsdf_config.tsdf_voxels_per_side;
  return esdf_config;
}

MeshIntegratorConfig getMeshIntegratorConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  MeshIntegratorConfig mesh_integrator_config;

  nh_private.param(
      "mesh_min_weight", mesh_integrator_config.min_weight,
      mesh_integrator_config.min_weight);
  nh_private.param(
      "mesh_use_color", mesh_integrator_config.use_color,
      mesh_integrator_config.use_color);

  return mesh_integrator_config;
}

EsdfVoxfieldIntegrator::Config getEsdfVoxfieldIntegratorConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  EsdfVoxfieldIntegrator::Config esdf_integrator_config;

  int range_boundary_offset_x = esdf_integrator_config.range_boundary_offset(0);
  int range_boundary_offset_y = esdf_integrator_config.range_boundary_offset(1);
  int range_boundary_offset_z = esdf_integrator_config.range_boundary_offset(2);

  nh_private.param(
      "local_range_offset_x", range_boundary_offset_x, range_boundary_offset_x);
  nh_private.param(
      "local_range_offset_y", range_boundary_offset_y, range_boundary_offset_y);
  nh_private.param(
      "local_range_offset_z", range_boundary_offset_z, range_boundary_offset_z);
  nh_private.param(
      "esdf_max_distance_m", esdf_integrator_config.max_distance_m,
      esdf_integrator_config.max_distance_m);
  nh_private.param(
      "esdf_default_distance_m", esdf_integrator_config.default_distance_m,
      esdf_integrator_config.default_distance_m);
  nh_private.param(
      "fix_band_distance_m", esdf_integrator_config.band_distance_m,
      esdf_integrator_config.band_distance_m);
  nh_private.param(
      "max_behind_surface_m", esdf_integrator_config.max_behind_surface_m,
      esdf_integrator_config.max_behind_surface_m);
  nh_private.param(
      "occ_min_weight", esdf_integrator_config.min_weight,
      esdf_integrator_config.min_weight);
  nh_private.param(
      "occ_voxel_size_ratio", esdf_integrator_config.occ_voxel_size_ratio,
      esdf_integrator_config.occ_voxel_size_ratio);
  nh_private.param(
      "num_buckets", esdf_integrator_config.num_buckets,
      esdf_integrator_config.num_buckets);
  nh_private.param(
      "patch_on", esdf_integrator_config.patch_on,
      esdf_integrator_config.patch_on);
  nh_private.param(
      "early_break", esdf_integrator_config.early_break,
      esdf_integrator_config.early_break);
  nh_private.param(
      "finer_esdf_on", esdf_integrator_config.finer_esdf_on,
      esdf_integrator_config.finer_esdf_on);

  int max_blocks_per_update =
      static_cast<int>(esdf_integrator_config.max_blocks_per_update);
  nh_private.param(
      "max_esdf_blocks_per_update", max_blocks_per_update,
      max_blocks_per_update);
  esdf_integrator_config.max_blocks_per_update =
      max_blocks_per_update <= 0 ? 0u : static_cast<size_t>(max_blocks_per_update);

  esdf_integrator_config.range_boundary_offset(0) = range_boundary_offset_x;
  esdf_integrator_config.range_boundary_offset(1) = range_boundary_offset_y;
  esdf_integrator_config.range_boundary_offset(2) = range_boundary_offset_z;
  return esdf_integrator_config;
}

SensorConfig getSensorConfigFromRosParam(
    const ros::NodeHandle& nh_private, const SensorConfig& defaults) {
  SensorConfig config = defaults;
  nh_private.param("sensor_is_lidar", config.sensor_is_lidar,
                   config.sensor_is_lidar);
  nh_private.param("width", config.width, config.width);
  nh_private.param("height", config.height, config.height);
  nh_private.param(
      "smooth_thre_ratio", config.smooth_thre_ratio,
      config.smooth_thre_ratio);
  nh_private.param("min_z", config.min_z, config.min_z);
  nh_private.param("min_dist", config.min_dist, config.min_dist);

  if (config.sensor_is_lidar) {
    nh_private.param("fov_up", config.fov_up, config.fov_up);
    nh_private.param("fov_down", config.fov_down, config.fov_down);
  } else {
    nh_private.param("vx", config.vx, config.vx);
    nh_private.param("vy", config.vy, config.vy);
    nh_private.param("fx", config.fx, config.fx);
    nh_private.param("fy", config.fy, config.fy);
  }
  return config;
}

RuntimeRosConfig getRuntimeRosConfigFromRosParam(
    const ros::NodeHandle& nh_private, const RuntimeRosConfig& defaults) {
  RuntimeRosConfig config = defaults;
  nh_private.param("world_frame", config.world_frame, config.world_frame);
  nh_private.param("sensor_frame", config.sensor_frame, config.sensor_frame);
  nh_private.param(
      "use_tf_transforms", config.use_tf_transforms,
      config.use_tf_transforms);
  nh_private.param(
      "use_freespace_pointcloud", config.use_freespace_pointcloud,
      config.use_freespace_pointcloud);
  nh_private.param("enable_icp", config.enable_icp, config.enable_icp);
  nh_private.param(
      "accumulate_icp_corrections", config.accumulate_icp_corrections,
      config.accumulate_icp_corrections);
  nh_private.param(
      "icp_corrected_frame", config.icp_corrected_frame,
      config.icp_corrected_frame);
  nh_private.param(
      "pose_corrected_frame", config.pose_corrected_frame,
      config.pose_corrected_frame);
  nh_private.param(
      "min_time_between_msgs_sec", config.min_time_between_msgs_sec,
      config.min_time_between_msgs_sec);
  nh_private.param(
      "max_block_distance_from_body", config.max_block_distance_from_body,
      config.max_block_distance_from_body);
  return config;
}

VisualizationConfig getVisualizationConfigFromRosParam(
    const ros::NodeHandle& nh_private, const VisualizationConfig& defaults) {
  VisualizationConfig config = defaults;
  nh_private.param(
      "publish_pointclouds_on_update", config.publish_pointclouds_on_update,
      config.publish_pointclouds_on_update);
  nh_private.param("publish_slices", config.publish_slices,
                   config.publish_slices);
  nh_private.param("publish_pointclouds", config.publish_tsdf_pointcloud,
                   config.publish_tsdf_pointcloud);
  config.publish_esdf_pointcloud = config.publish_tsdf_pointcloud;
  nh_private.param("publish_tsdf_map", config.publish_tsdf_map,
                   config.publish_tsdf_map);
  nh_private.param("publish_esdf_map", config.publish_esdf_map,
                   config.publish_esdf_map);
  nh_private.param("publish_traversable", config.publish_traversable,
                   config.publish_traversable);
  nh_private.param("publish_robot_model", config.publish_robot_model,
                   config.publish_robot_model);
  nh_private.param("robot_model_file", config.robot_model_file,
                   config.robot_model_file);
  nh_private.param("robot_model_scale", config.robot_model_scale,
                   config.robot_model_scale);
  nh_private.param("slice_level", config.slice_level, config.slice_level);
  return config;
}

QueueConfig getQueueConfigFromRosParam(
    const ros::NodeHandle& nh_private, const QueueConfig& defaults) {
  QueueConfig config = defaults;
  nh_private.param(
      "pointcloud_queue_size", config.pointcloud_queue_size,
      config.pointcloud_queue_size);
  nh_private.param(
      "max_transform_queue_size", config.max_transform_queue_size,
      config.max_transform_queue_size);
  nh_private.param(
      "max_transform_queue_age_sec", config.max_transform_queue_age_sec,
      config.max_transform_queue_age_sec);
  nh_private.param(
      "max_pointcloud_queue_size", config.max_pointcloud_queue_size,
      config.max_pointcloud_queue_size);
  nh_private.param(
      "max_pointcloud_queue_age_sec", config.max_pointcloud_queue_age_sec,
      config.max_pointcloud_queue_age_sec);
  return config;
}

}  // namespace voxblox
