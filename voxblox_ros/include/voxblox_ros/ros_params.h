#ifndef VOXBLOX_ROS_ROS_PARAMS_H_
#define VOXBLOX_ROS_ROS_PARAMS_H_

#include <cstddef>
#include <limits>
#include <string>

#include <ros/node_handle.h>

#include <voxblox/alignment/icp.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/core/occupancy_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/esdf_occ_edt_integrator.h>
#include <voxblox/integrator/esdf_occ_fiesta_integrator.h>
#include <voxblox/integrator/esdf_voxfield_integrator.h>
#include <voxblox/integrator/np_tsdf_integrator.h>
#include <voxblox/integrator/occupancy_integrator.h>
#include <voxblox/integrator/occupancy_tsdf_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/mesh/mesh_integrator.h>

namespace voxblox {

struct SensorConfig {
  bool sensor_is_lidar = false;
  int width = 640;
  int height = 480;
  int fx = 566;
  int fy = 566;
  int vx = 320;
  int vy = 240;
  float fov_up = 3.0f;
  float fov_down = -25.0f;
  float max_range = 0.0f;
  float min_range = 0.0f;
  float smooth_thre_ratio = 1.0f;
  float min_dist = 0.1f;
  float min_z = -1000.0f;
};

struct RuntimeRosConfig {
  std::string world_frame = "world";
  std::string sensor_frame;
  std::string icp_corrected_frame = "icp_corrected";
  std::string pose_corrected_frame = "pose_corrected";
  bool use_tf_transforms = true;
  bool use_freespace_pointcloud = false;
  bool enable_icp = false;
  bool accumulate_icp_corrections = true;
  double min_time_between_msgs_sec = 0.0;
  double max_block_distance_from_body =
      std::numeric_limits<FloatingPoint>::max();
};

struct VisualizationConfig {
  bool publish_tsdf_pointcloud = false;
  bool publish_esdf_pointcloud = false;
  bool publish_pointclouds_on_update = false;
  bool publish_slices = false;
  bool publish_mesh = true;
  bool publish_robot_model = false;
  bool publish_traversable = false;
  bool publish_tsdf_map = false;
  bool publish_esdf_map = false;
  std::string robot_model_file;
  float robot_model_scale = 1.0f;
  double slice_level = 0.5;
};

struct QueueConfig {
  int pointcloud_queue_size = 1;
  size_t max_transform_queue_size = 200u;
  double max_transform_queue_age_sec = 5.0;
  size_t max_pointcloud_queue_size = 100u;
  double max_pointcloud_queue_age_sec = 5.0;
};

// Active Voxfield / NP-TSDF config readers live in ros_params.cc so the
// headers expose typed contracts without carrying the parsing implementation.
TsdfMap::Config getTsdfMapConfigFromRosParam(
    const ros::NodeHandle& nh_private);
ICP::Config getICPConfigFromRosParam(const ros::NodeHandle& nh_private);
TsdfIntegratorBase::Config getTsdfIntegratorConfigFromRosParam(
    const ros::NodeHandle& nh_private);
NpTsdfIntegratorBase::Config getNpTsdfIntegratorConfigFromRosParam(
    const ros::NodeHandle& nh_private);
EsdfMap::Config getEsdfMapConfigFromRosParam(
    const ros::NodeHandle& nh_private);
MeshIntegratorConfig getMeshIntegratorConfigFromRosParam(
    const ros::NodeHandle& nh_private);
EsdfVoxfieldIntegrator::Config getEsdfVoxfieldIntegratorConfigFromRosParam(
    const ros::NodeHandle& nh_private);
SensorConfig getSensorConfigFromRosParam(
    const ros::NodeHandle& nh_private, const SensorConfig& defaults);
RuntimeRosConfig getRuntimeRosConfigFromRosParam(
    const ros::NodeHandle& nh_private, const RuntimeRosConfig& defaults);
VisualizationConfig getVisualizationConfigFromRosParam(
    const ros::NodeHandle& nh_private, const VisualizationConfig& defaults);
QueueConfig getQueueConfigFromRosParam(
    const ros::NodeHandle& nh_private, const QueueConfig& defaults);

// Legacy/unbuilt-server readers below remain inline for Phase 2 to keep this
// patch focused. Phase 3 should either port those servers or quarantine them.

inline EsdfIntegrator::Config getEsdfIntegratorConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  EsdfIntegrator::Config esdf_integrator_config;

  TsdfIntegratorBase::Config tsdf_integrator_config =
      getTsdfIntegratorConfigFromRosParam(nh_private);

  esdf_integrator_config.min_distance_m =
      tsdf_integrator_config.default_truncation_distance / 2.0;

  nh_private.param(
      "esdf_euclidean_distance", esdf_integrator_config.full_euclidean_distance,
      esdf_integrator_config.full_euclidean_distance);
  nh_private.param(
      "esdf_max_distance_m", esdf_integrator_config.max_distance_m,
      esdf_integrator_config.max_distance_m);
  nh_private.param(
      "esdf_min_distance_m", esdf_integrator_config.min_distance_m,
      esdf_integrator_config.min_distance_m);
  nh_private.param(
      "esdf_default_distance_m", esdf_integrator_config.default_distance_m,
      esdf_integrator_config.default_distance_m);
  nh_private.param(
      "esdf_min_diff_m", esdf_integrator_config.min_diff_m,
      esdf_integrator_config.min_diff_m);
  nh_private.param(
      "clear_sphere_radius", esdf_integrator_config.clear_sphere_radius,
      esdf_integrator_config.clear_sphere_radius);
  nh_private.param(
      "occupied_sphere_radius", esdf_integrator_config.occupied_sphere_radius,
      esdf_integrator_config.occupied_sphere_radius);
  nh_private.param(
      "esdf_add_occupied_crust", esdf_integrator_config.add_occupied_crust,
      esdf_integrator_config.add_occupied_crust);

  if (esdf_integrator_config.default_distance_m <
      esdf_integrator_config.max_distance_m) {
    esdf_integrator_config.default_distance_m =
        esdf_integrator_config.max_distance_m;
  }

  return esdf_integrator_config;
}

inline OccupancyMap::Config getOccupancyMapConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  OccupancyMap::Config occ_config;

  /**
   * Workaround for OS X on mac mini not having specializations for float
   * for some reason.
   */
  double voxel_size = occ_config.occupancy_voxel_size;
  int voxels_per_side = occ_config.occupancy_voxels_per_side;
  nh_private.param("occ_voxel_size", voxel_size, voxel_size);
  nh_private.param(
      "occ_voxels_per_side", voxels_per_side,
      voxels_per_side);  // block size (unit: voxel)
  if (!isPowerOfTwo(voxels_per_side)) {
    ROS_ERROR("voxels_per_side must be a power of 2, setting to default value");
    voxels_per_side = occ_config.occupancy_voxels_per_side;
  }

  occ_config.occupancy_voxel_size = static_cast<FloatingPoint>(voxel_size);
  occ_config.occupancy_voxels_per_side = voxels_per_side;

  return occ_config;
}

inline OccTsdfIntegrator::Config getOccTsdfIntegratorConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  OccTsdfIntegrator::Config integrator_config;

  nh_private.param(
      "occ_min_weight", integrator_config.min_weight,
      integrator_config.min_weight);

  nh_private.param(
      "occ_voxel_size_ratio", integrator_config.occ_voxel_size_ratio,
      integrator_config.occ_voxel_size_ratio);

  return integrator_config;
}

inline EsdfMap::Config getEsdfMapConfigFromOccMapRosParam(
    const ros::NodeHandle& nh_private) {
  EsdfMap::Config esdf_config;

  const OccupancyMap::Config occ_config =
      getOccupancyMapConfigFromRosParam(nh_private);
  esdf_config.esdf_voxel_size = occ_config.occupancy_voxel_size;
  esdf_config.esdf_voxels_per_side = occ_config.occupancy_voxels_per_side;

  return esdf_config;
}

inline TsdfMap::Config getTsdfMapConfigFromOccMapRosParam(
    const ros::NodeHandle& nh_private) {
  TsdfMap::Config tsdf_config;

  const OccupancyMap::Config occ_config =
      getOccupancyMapConfigFromRosParam(nh_private);
  tsdf_config.tsdf_voxel_size = occ_config.occupancy_voxel_size;
  tsdf_config.tsdf_voxels_per_side = occ_config.occupancy_voxels_per_side;

  return tsdf_config;
}

inline TsdfMap::Config getTsdfMapConfigFromEsdfMapRosParam(
    const ros::NodeHandle& nh_private) {
  TsdfMap::Config tsdf_config;

  const EsdfMap::Config esdf_config = getEsdfMapConfigFromRosParam(nh_private);
  tsdf_config.tsdf_voxel_size = esdf_config.esdf_voxel_size;
  tsdf_config.tsdf_voxels_per_side = esdf_config.esdf_voxels_per_side;

  return tsdf_config;
}

inline EsdfOccFiestaIntegrator::Config
getEsdfOccFiestaIntegratorConfigFromRosParam(
    const ros::NodeHandle& nh_private) {  // NOLINT
  EsdfOccFiestaIntegrator::Config esdf_integrator_config;

  int range_boundary_offset_x = esdf_integrator_config.range_boundary_offset(0);
  int range_boundary_offset_y = esdf_integrator_config.range_boundary_offset(1);
  int range_boundary_offset_z = esdf_integrator_config.range_boundary_offset(2);

  // esdf_integrator_config.min_distance_m =
  //     tsdf_integrator_config.default_truncation_distance / 2.0;

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
      "max_behind_surface_m", esdf_integrator_config.max_behind_surface_m,
      esdf_integrator_config.max_behind_surface_m);
  // max_behind_surface_m should be at least sqrt(3) * truncation_dist

  nh_private.param(
      "num_buckets", esdf_integrator_config.num_buckets,
      esdf_integrator_config.num_buckets);

  nh_private.param(
      "patch_on", esdf_integrator_config.patch_on,
      esdf_integrator_config.patch_on);

  nh_private.param(
      "early_break", esdf_integrator_config.early_break,
      esdf_integrator_config.early_break);

  esdf_integrator_config.range_boundary_offset(0) = range_boundary_offset_x;
  esdf_integrator_config.range_boundary_offset(1) = range_boundary_offset_y;
  esdf_integrator_config.range_boundary_offset(2) = range_boundary_offset_z;

  return esdf_integrator_config;
}

inline EsdfOccEdtIntegrator::Config getEsdfEdtIntegratorConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  EsdfOccEdtIntegrator::Config esdf_integrator_config;

  int range_boundary_offset_x = esdf_integrator_config.range_boundary_offset(0);
  int range_boundary_offset_y = esdf_integrator_config.range_boundary_offset(1);
  int range_boundary_offset_z = esdf_integrator_config.range_boundary_offset(2);

  // esdf_integrator_config.min_distance_m =
  //     tsdf_integrator_config.default_truncation_distance / 2.0;

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
      "max_behind_surface_m", esdf_integrator_config.max_behind_surface_m,
      esdf_integrator_config.max_behind_surface_m);
  // max_behind_surface_m should be at least sqrt(3) * truncation_dist

  nh_private.param(
      "num_buckets", esdf_integrator_config.num_buckets,
      esdf_integrator_config.num_buckets);

  if (esdf_integrator_config.default_distance_m <
      esdf_integrator_config.max_distance_m) {
    esdf_integrator_config.default_distance_m =
        esdf_integrator_config.max_distance_m;
  }

  esdf_integrator_config.range_boundary_offset(0) = range_boundary_offset_x;
  esdf_integrator_config.range_boundary_offset(1) = range_boundary_offset_y;
  esdf_integrator_config.range_boundary_offset(2) = range_boundary_offset_z;

  return esdf_integrator_config;
}

}  // namespace voxblox

#endif  // VOXBLOX_ROS_ROS_PARAMS_H_
