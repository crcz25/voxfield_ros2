#include "voxblox_ros/np_tsdf_server.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

#include "voxblox_ros/conversions.h"
#include "voxblox_ros/ros_params.h"

namespace voxblox {
namespace {

bool startsWith(const std::string& value, const std::string& prefix) {
  return value.rfind(prefix, 0) == 0;
}

bool packageResourceExists(const std::string& resource_uri) {
  constexpr char kPackagePrefix[] = "package://";
  const std::string package_path = resource_uri.substr(std::strlen(kPackagePrefix));
  const size_t separator = package_path.find('/');
  if (separator == std::string::npos || separator == 0 ||
      separator + 1 >= package_path.size()) {
    return false;
  }

  const std::string package_name = package_path.substr(0, separator);
  const std::string relative_path = package_path.substr(separator + 1);
  try {
    const std::filesystem::path absolute_path =
        std::filesystem::path(
            ament_index_cpp::get_package_share_directory(package_name)) /
        relative_path;
    return std::filesystem::exists(absolute_path);
  } catch (const ament_index_cpp::PackageNotFoundError&) {
    return false;
  }
}

std::string resolveRobotModelResource(const std::string& robot_model_file) {
  if (robot_model_file.empty()) {
    return "";
  }

  if (startsWith(robot_model_file, "package://")) {
    if (packageResourceExists(robot_model_file)) {
      return robot_model_file;
    }
    ROS_WARN(
        "Robot model resource '%s' does not resolve to an installed file. "
        "Robot_model marker will not be published.",
        robot_model_file.c_str());
    return "";
  }

  if (startsWith(robot_model_file, "file://")) {
    const std::string absolute_path = robot_model_file.substr(7);
    if (!absolute_path.empty() && std::filesystem::exists(absolute_path)) {
      return robot_model_file;
    }
    ROS_WARN(
        "Robot model file URI '%s' is empty or does not exist. "
        "Robot_model marker will not be published.",
        robot_model_file.c_str());
    return "";
  }

  if (!robot_model_file.empty() && robot_model_file[0] == '/') {
    if (std::filesystem::exists(robot_model_file)) {
      return "file://" + robot_model_file;
    }
    ROS_WARN(
        "Robot model file '%s' does not exist. Robot_model marker will not be "
        "published.",
        robot_model_file.c_str());
    return "";
  }

  ROS_WARN(
      "Robot model path '%s' is not a package:// URI, file:// URI, or absolute "
      "path. Robot_model marker will not be published.",
      robot_model_file.c_str());
  return "";
}

}  // namespace

NpTsdfServer::NpTsdfServer(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : NpTsdfServer(
          nh, nh_private, getTsdfMapConfigFromRosParam(nh_private),
          getNpTsdfIntegratorConfigFromRosParam(nh_private),
          getMeshIntegratorConfigFromRosParam(nh_private)) {}

NpTsdfServer::NpTsdfServer(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    const TsdfMap::Config& config,
    const NpTsdfIntegratorBase::Config& integrator_config,  // NOLINT
    const MeshIntegratorConfig& mesh_config)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(true),
      world_frame_("world"),
      icp_corrected_frame_("icp_corrected"),
      pose_corrected_frame_("pose_corrected"),
      max_block_distance_from_body_(std::numeric_limits<FloatingPoint>::max()),
      slice_level_(0.5),
      use_freespace_pointcloud_(false),
      color_map_(std::make_shared<RainbowColorMap>()),
      publish_pointclouds_on_update_(false),
      publish_slices_(false),
      publish_pointclouds_(false),
      publish_tsdf_map_(false),
      publish_robot_model_(false),
      cache_mesh_(false),
      enable_icp_(false),
      accumulate_icp_corrections_(true),
      pointcloud_queue_size_(1),
      num_subscribers_tsdf_map_(0),
      transformer_(nh, nh_private) {
  last_memory_log_time_ = std::chrono::steady_clock::now();
  getServerConfigFromRosParam(nh_private);
  if (!validateSensorConfig()) {
    shutdown_requested_.store(true);
    throw std::runtime_error("Invalid Voxfield sensor configuration");
  }

  // Advertise topics.
  surface_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
          "surface_pointcloud", 1, true);
  tsdf_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
          "tsdf_pointcloud", 1, true);
  gsdf_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
          "gsdf_pointcloud", 1, true);
  occupancy_marker_pub_ =
      nh_private_.advertise<visualization_msgs::msg::MarkerArray>(
          "occupied_nodes", 1, true);
  tsdf_slice_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "tsdf_slice", 1, true);
  gsdf_slice_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "gsdf_slice", 1, true);

  // Map-mutating pointcloud callbacks assume SingleThreadedExecutor-style
  // mutually exclusive execution; see the class-level Phase 2 note.
  pointcloud_sub_ = nh_.subscribe(
      "pointcloud", pointcloud_queue_size_, &NpTsdfServer::insertPointcloud,
      this);

  mesh_pub_ = nh_private_.advertise<voxblox_msgs::msg::Mesh>("mesh", 1, true);

  // Publishing/subscribing to a layer from another node (when using this as
  // a library, for example within a planner).
  tsdf_map_pub_ =
      nh_private_.advertise<voxblox_msgs::msg::Layer>("tsdf_map_out", 1, false);
  tsdf_map_sub_ = nh_private_.subscribe(
      "tsdf_map_in", 1, &NpTsdfServer::tsdfMapCallback, this);
  robot_model_pub_ =
      nh_private_.advertise<visualization_msgs::msg::Marker>("Robot_model", 100);

  if (use_freespace_pointcloud_) {
    // points that are not inside an object, but may also not be on a surface.
    // These will only be used to mark freespace beyond the truncation distance.
    freespace_pointcloud_sub_ = nh_.subscribe(
        "freespace_pointcloud", pointcloud_queue_size_,
        &NpTsdfServer::insertFreespacePointcloud, this);
  }

  if (enable_icp_) {
    icp_transform_pub_ = nh_private_.advertise<geometry_msgs::msg::TransformStamped>(
        "icp_transform", 1, true);
  }

  // Initialize TSDF Map and integrator.
  tsdf_map_ = std::make_shared<TsdfMap>(config);

  std::string method("merged");
  nh_private_.param("method", method, method);
  if (method.compare("simple") == 0) {
    tsdf_integrator_ = std::make_unique<SimpleNpTsdfIntegrator>(
        integrator_config, tsdf_map_->getTsdfLayerPtr());
  } else if (method.compare("merged") == 0) {
    tsdf_integrator_ = std::make_unique<MergedNpTsdfIntegrator>(
        integrator_config, tsdf_map_->getTsdfLayerPtr());
  } else if (method.compare("fast") == 0) {
    tsdf_integrator_ = std::make_unique<FastNpTsdfIntegrator>(
        integrator_config, tsdf_map_->getTsdfLayerPtr());
  } else {
    tsdf_integrator_ = std::make_unique<SimpleNpTsdfIntegrator>(
        integrator_config, tsdf_map_->getTsdfLayerPtr());
  }

  mesh_layer_ = std::make_shared<MeshLayer>(tsdf_map_->block_size());
  mesh_integrator_ = std::make_unique<MeshIntegrator<TsdfVoxel>>(
      mesh_config, tsdf_map_->getTsdfLayerPtr(), mesh_layer_.get());
  icp_ = std::make_shared<ICP>(getICPConfigFromRosParam(nh_private));

  // Advertise services.
  generate_mesh_srv_ = nh_private_.advertiseService<std_srvs::srv::Empty>(
      "generate_mesh", &NpTsdfServer::generateMeshCallback, this);
  clear_map_srv_ = nh_private_.advertiseService<std_srvs::srv::Empty>(
      "clear_map", &NpTsdfServer::clearMapCallback, this);
  save_map_srv_ = nh_private_.advertiseService<voxblox_msgs::srv::FilePath>(
      "save_map", &NpTsdfServer::saveMapCallback, this);
  load_map_srv_ = nh_private_.advertiseService<voxblox_msgs::srv::FilePath>(
      "load_map", &NpTsdfServer::loadMapCallback, this);
  publish_pointclouds_srv_ = nh_private_.advertiseService<std_srvs::srv::Empty>(
      "publish_pointclouds", &NpTsdfServer::publishPointcloudsCallback, this);
  publish_tsdf_map_srv_ = nh_private_.advertiseService<std_srvs::srv::Empty>(
      "publish_map", &NpTsdfServer::publishTsdfMapCallback, this);

  // If set, use a timer to progressively integrate the mesh.
  double update_mesh_every_n_sec = 1.0f;
  nh_private_.param(
      "update_mesh_every_n_sec", update_mesh_every_n_sec,
      update_mesh_every_n_sec);

  if (update_mesh_every_n_sec > 0.0) {
    // Timer mutates mesh state shared with pointcloud callbacks.
    update_mesh_timer_ = nh_private_.createTimer(
        ros::Duration(update_mesh_every_n_sec), &NpTsdfServer::updateMeshEvent,
        this);
  } else {
    update_mesh_every_n_ = static_cast<int>(-1.0 * update_mesh_every_n_sec);
  }

  double publish_map_every_n_sec = 1.0;
  nh_private_.param(
      "publish_map_every_n_sec", publish_map_every_n_sec,
      publish_map_every_n_sec);

  if (publish_map_every_n_sec > 0.0) {
    // Timer reads map state shared with pointcloud callbacks.
    publish_map_timer_ = nh_private_.createTimer(
        ros::Duration(publish_map_every_n_sec), &NpTsdfServer::publishMapEvent,
        this);
  }
}

NpTsdfServer::~NpTsdfServer() {
  shutdown();
}

void NpTsdfServer::getServerConfigFromRosParam(
    const ros::NodeHandle& nh_private) {
  sensor_config_.sensor_is_lidar = sensor_is_lidar_;
  sensor_config_.width = width_;
  sensor_config_.height = height_;
  sensor_config_.fx = fx_;
  sensor_config_.fy = fy_;
  sensor_config_.vx = vx_;
  sensor_config_.vy = vy_;
  sensor_config_.fov_up = fov_up_;
  sensor_config_.fov_down = fov_down_;
  sensor_config_.max_range = max_range_;
  sensor_config_.min_range = min_range_;
  sensor_config_.smooth_thre_ratio = smooth_thre_ratio_;
  sensor_config_.min_dist = min_dist_;
  sensor_config_.min_z = min_z_;

  runtime_config_.world_frame = world_frame_;
  runtime_config_.sensor_frame = sensor_frame_;
  runtime_config_.icp_corrected_frame = icp_corrected_frame_;
  runtime_config_.pose_corrected_frame = pose_corrected_frame_;
  runtime_config_.use_freespace_pointcloud = use_freespace_pointcloud_;
  runtime_config_.enable_icp = enable_icp_;
  runtime_config_.accumulate_icp_corrections = accumulate_icp_corrections_;
  runtime_config_.min_time_between_msgs_sec = min_time_between_msgs_.toSec();
  runtime_config_.max_block_distance_from_body = max_block_distance_from_body_;

  visualization_config_.publish_tsdf_pointcloud = publish_pointclouds_;
  visualization_config_.publish_esdf_pointcloud = publish_pointclouds_;
  visualization_config_.publish_pointclouds_on_update =
      publish_pointclouds_on_update_;
  visualization_config_.publish_slices = publish_slices_;
  visualization_config_.publish_robot_model = publish_robot_model_;
  visualization_config_.publish_tsdf_map = publish_tsdf_map_;
  visualization_config_.robot_model_file = robot_model_file_;
  visualization_config_.robot_model_scale = robot_model_scale_;
  visualization_config_.slice_level = slice_level_;

  queue_config_.pointcloud_queue_size = pointcloud_queue_size_;

  sensor_config_ = getSensorConfigFromRosParam(nh_private, sensor_config_);
  runtime_config_ = getRuntimeRosConfigFromRosParam(nh_private, runtime_config_);
  visualization_config_ =
      getVisualizationConfigFromRosParam(nh_private, visualization_config_);
  queue_config_ = getQueueConfigFromRosParam(nh_private, queue_config_);

  min_time_between_msgs_.fromSec(runtime_config_.min_time_between_msgs_sec);
  max_block_distance_from_body_ = runtime_config_.max_block_distance_from_body;
  world_frame_ = runtime_config_.world_frame;
  sensor_frame_ = runtime_config_.sensor_frame;
  use_freespace_pointcloud_ = runtime_config_.use_freespace_pointcloud;
  enable_icp_ = runtime_config_.enable_icp;
  accumulate_icp_corrections_ = runtime_config_.accumulate_icp_corrections;
  icp_corrected_frame_ = runtime_config_.icp_corrected_frame;
  pose_corrected_frame_ = runtime_config_.pose_corrected_frame;

  slice_level_ = visualization_config_.slice_level;
  publish_pointclouds_on_update_ =
      visualization_config_.publish_pointclouds_on_update;
  publish_slices_ = visualization_config_.publish_slices;
  publish_pointclouds_ = visualization_config_.publish_tsdf_pointcloud;
  publish_tsdf_map_ = visualization_config_.publish_tsdf_map;
  publish_robot_model_ = visualization_config_.publish_robot_model;
  robot_model_file_ = visualization_config_.robot_model_file;
  robot_model_scale_ = visualization_config_.robot_model_scale;

  pointcloud_queue_size_ = std::max(1, queue_config_.pointcloud_queue_size);

  // Logging
  nh_private.param("verbose", verbose_, verbose_);
  nh_private.param("timing", timing_, timing_);
  nh_private.param(
      "memory_log_interval_sec", memory_log_interval_sec_,
      memory_log_interval_sec_);

  // Sensor specific
  sensor_is_lidar_ = sensor_config_.sensor_is_lidar;
  width_ = sensor_config_.width;
  height_ = sensor_config_.height;
  max_range_ = sensor_config_.max_range;
  min_range_ = sensor_config_.min_range;
  smooth_thre_ratio_ = sensor_config_.smooth_thre_ratio;
  min_z_ = sensor_config_.min_z;
  min_dist_ = sensor_config_.min_dist;

  if (sensor_is_lidar_) {
    fov_up_ = sensor_config_.fov_up;
    fov_down_ = sensor_config_.fov_down;
    float fov = std::abs(fov_down_) + std::abs(fov_up_);
    fov_down_rad_ = fov_down_ / 180.0f * M_PI;
    fov_rad_ = fov / 180.0f * M_PI;
  } else {
    vx_ = sensor_config_.vx;
    vy_ = sensor_config_.vy;
    fx_ = sensor_config_.fx;
    fy_ = sensor_config_.fy;
  }

  // Robot model related
  robot_model_resource_ = resolveRobotModelResource(robot_model_file_);
  if (publish_robot_model_ && robot_model_resource_.empty()) {
    ROS_WARN(
        "publish_robot_model is true, but robot_model_file is empty or "
        "invalid. Robot_model marker will not be published.");
  }

  // Mesh settings.
  nh_private.param("mesh_filename", mesh_filename_, mesh_filename_);
  std::string color_mode("");
  nh_private.param("color_mode", color_mode, color_mode);
  color_mode_ = getColorModeFromString(color_mode);

  // Color map for intensity pointclouds.
  std::string intensity_colormap("rainbow");
  float intensity_max_value = kDefaultMaxIntensity;
  nh_private.param(
      "intensity_colormap", intensity_colormap, intensity_colormap);
  nh_private.param(
      "intensity_max_value", intensity_max_value, intensity_max_value);

  // Default set in constructor.
  if (intensity_colormap == "rainbow") {
    color_map_ = std::make_shared<RainbowColorMap>();
  } else if (intensity_colormap == "inverse_rainbow") {
    color_map_ = std::make_shared<InverseRainbowColorMap>();
  } else if (intensity_colormap == "grayscale") {
    color_map_ = std::make_shared<GrayscaleColorMap>();
  } else if (intensity_colormap == "inverse_grayscale") {
    color_map_ = std::make_shared<InverseGrayscaleColorMap>();
  } else if (intensity_colormap == "ironbow") {
    color_map_ = std::make_shared<IronbowColorMap>();
  } else {
    ROS_ERROR_STREAM("Invalid color map: " << intensity_colormap);
  }
  color_map_->setMaxValue(intensity_max_value);
}

bool NpTsdfServer::validateSensorConfig() const {
  if (width_ <= 0 || height_ <= 0) {
    ROS_ERROR(
        "Invalid sensor configuration: width and height must be positive "
        "(width=%d, height=%d).",
        width_, height_);
    return false;
  }

  if (sensor_is_lidar_) {
    if (fov_up_ <= fov_down_) {
      ROS_ERROR(
          "Invalid LiDAR sensor configuration: fov_up must be greater than "
          "fov_down (fov_up=%.3f, fov_down=%.3f).",
          fov_up_, fov_down_);
      return false;
    }
    if (fov_rad_ <= 0.0f) {
      ROS_ERROR(
          "Invalid LiDAR sensor configuration: vertical FoV must be positive "
          "(fov_rad=%.6f).",
          fov_rad_);
      return false;
    }
    return true;
  }

  if (fx_ <= 0 || fy_ <= 0) {
    ROS_ERROR(
        "Invalid camera sensor configuration: fx and fy must be positive "
        "(fx=%d, fy=%d).",
        fx_, fy_);
    return false;
  }
  return true;
}

void NpTsdfServer::processPointCloudMessageAndInsert(
    const sensor_msgs::msg::PointCloud2::SharedPtr& pointcloud_msg,
    const Transformation& T_G_C, const bool is_freespace_pointcloud) {
  if (shutdown_requested_.load() || !ros::ok()) {
    return;
  }

  const DecodedPointcloud decoded = decodePointcloudMessage(pointcloud_msg);
  const PreprocessedPointcloud preprocessed = projectAndEstimateNormals(decoded);
  const Transformation T_G_C_refined =
      refinePoseWithIcp(T_G_C, preprocessed.points_C, pointcloud_msg->header.stamp);
  integratePreparedPointcloud(
      T_G_C_refined, preprocessed, is_freespace_pointcloud);
  finishPointcloudIntegration(T_G_C);
  publishRobotMesh(T_G_C_refined);

  // Callback for inheriting classes.
  newPoseCallback(T_G_C);
}

NpTsdfServer::DecodedPointcloud NpTsdfServer::decodePointcloudMessage(
    const sensor_msgs::msg::PointCloud2::SharedPtr& pointcloud_msg) const {
  timing::Timer ptcloud_timer("preprocess/input");
  bool color_pointcloud = false;
  bool has_intensity = false;
  bool has_label = false;
  for (size_t d = 0; d < pointcloud_msg->fields.size(); ++d) {
    if (pointcloud_msg->fields[d].name == std::string("rgb")) {
      pointcloud_msg->fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
      color_pointcloud = true;
    } else if (pointcloud_msg->fields[d].name == std::string("intensity")) {
      has_intensity = true;
    } else if (pointcloud_msg->fields[d].name == std::string("label")) {
      has_label = true;
      // ROS_INFO("Found semantic/instance label in the point cloud");
    }
  }

  DecodedPointcloud decoded;

  if (has_label) {
    pcl::PointCloud<pcl::PointXYZRGBL> pointcloud_pcl;
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(
        pointcloud_pcl, color_map_, &decoded.points_C, &decoded.colors,
        &decoded.labels, true);
  } else if (color_pointcloud) {
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl;
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(
        pointcloud_pcl, color_map_, &decoded.points_C, &decoded.colors);
  } else if (has_intensity) {
    pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(
        pointcloud_pcl, color_map_, &decoded.points_C, &decoded.colors);
  } else {
    pcl::PointCloud<pcl::PointXYZ> pointcloud_pcl;
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(
        pointcloud_pcl, color_map_, &decoded.points_C, &decoded.colors);
  }
  ptcloud_timer.Stop();
  return decoded;
}

NpTsdfServer::PreprocessedPointcloud NpTsdfServer::projectAndEstimateNormals(
    const DecodedPointcloud& decoded) const {
  timing::Timer range_pre_timer("preprocess/normal_estimation");
  cv::Mat vertex_map = cv::Mat::zeros(height_, width_, CV_32FC3);
  cv::Mat depth_image(vertex_map.size(), CV_32FC1, -1.0);
  cv::Mat color_image = cv::Mat::zeros(vertex_map.size(), CV_8UC3);
  projectPointCloudToImage(
      decoded.points_C, decoded.colors, vertex_map, depth_image, color_image,
      min_z_, min_dist_);
  cv::Mat normal_image = computeNormalImage(vertex_map, depth_image);

  PreprocessedPointcloud preprocessed;
  preprocessed.points_C = extractPointCloud(vertex_map, depth_image);
  preprocessed.normals_C = extractNormals(normal_image, depth_image);
  preprocessed.colors = extractColors(color_image, depth_image);
  range_pre_timer.Stop();
  return preprocessed;
}

Transformation NpTsdfServer::refinePoseWithIcp(
    const Transformation& T_G_C, const Pointcloud& points_C,
    const builtin_interfaces::msg::Time& stamp) {
  Transformation T_G_C_refined = T_G_C;
  if (!enable_icp_) {
    return T_G_C_refined;
  }

  timing::Timer icp_timer("icp");
  if (!accumulate_icp_corrections_) {
    icp_corrected_transform_.setIdentity();
  }
  const size_t num_icp_updates = icp_->runICP(
      tsdf_map_->getTsdfLayer(), points_C, icp_corrected_transform_ * T_G_C,
      &T_G_C_refined);
  if (verbose_) {
    ROS_INFO(
        "ICP refinement performed %zu successful update steps",
        num_icp_updates);
  }
  icp_corrected_transform_ = T_G_C_refined * T_G_C.inverse();

  if (!icp_->refiningRollPitch()) {
    // Roll/pitch are removed internally; this prevents small accumulated
    // floating point errors when corrections are accumulated.
    Transformation::Vector6 T_vec = icp_corrected_transform_.log();
    T_vec[3] = 0.0;
    T_vec[4] = 0.0;
    icp_corrected_transform_ = Transformation::exp(T_vec);
  }

  tf::Transform icp_tf_msg, pose_tf_msg;
  geometry_msgs::msg::TransformStamped transform_msg;

  tf::transformKindrToTF(
      icp_corrected_transform_.cast<double>(), &icp_tf_msg);
  tf::transformKindrToTF(T_G_C.cast<double>(), &pose_tf_msg);
  tf::transformKindrToMsg(
      icp_corrected_transform_.cast<double>(), &transform_msg.transform);
  tf_broadcaster_.sendTransform(tf::StampedTransform(
      icp_tf_msg, stamp, world_frame_, icp_corrected_frame_));
  tf_broadcaster_.sendTransform(tf::StampedTransform(
      pose_tf_msg, stamp, icp_corrected_frame_, pose_corrected_frame_));

  transform_msg.header.frame_id = world_frame_;
  transform_msg.child_frame_id = icp_corrected_frame_;
  icp_transform_pub_.publish(transform_msg);

  icp_timer.Stop();
  return T_G_C_refined;
}

void NpTsdfServer::integratePreparedPointcloud(
    const Transformation& T_G_C, const PreprocessedPointcloud& pointcloud,
    bool is_freespace_pointcloud) {
  if (verbose_) {
    ROS_INFO(
        "Integrating a pointcloud with %lu points.",
        pointcloud.points_C.size());
  }

  ros::WallTime start = ros::WallTime::now();
  integratePointcloud(
      T_G_C, pointcloud.points_C, pointcloud.normals_C, pointcloud.colors,
      is_freespace_pointcloud);
  ros::WallTime end = ros::WallTime::now();
  if (verbose_) {
    ROS_INFO(
        "Finished integrating in %f seconds, have %lu blocks.",
        (end - start).toSec(),
        tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks());
  }
  logMemoryStatus("pointcloud", pointcloud.points_C.size());
}

void NpTsdfServer::finishPointcloudIntegration(const Transformation& T_G_C) {
  if (update_mesh_every_n_ > 0 && frame_count_ != 0 &&
      frame_count_ % update_mesh_every_n_ == 0) {
    updateMesh();
  }

  tsdf_map_->getTsdfLayerPtr()->removeDistantBlocks(
      T_G_C.getPosition(), max_block_distance_from_body_);
  mesh_layer_->clearDistantMesh(
      T_G_C.getPosition(), max_block_distance_from_body_);
}

void NpTsdfServer::publishRobotMesh(const Transformation& T_G_C) {
  if (!publish_robot_model_ || robot_model_resource_.empty()) {
    return;
  }

  // publish the robot model with the pose
  visualization_msgs::msg::Marker robot_model;
  robot_model.header.frame_id = world_frame_;
  robot_model.header.stamp = ros::Time();
  robot_model.mesh_resource = robot_model_resource_;
  robot_model.mesh_use_embedded_materials = true;
  robot_model.scale.x = robot_model.scale.y = robot_model.scale.z =
      robot_model_scale_;
  robot_model.lifetime = ros::Duration();
  robot_model.action = visualization_msgs::msg::Marker::MODIFY;
  robot_model.color.a = robot_model.color.r = robot_model.color.g =
      robot_model.color.b = 1.;
  robot_model.type = visualization_msgs::msg::Marker::MESH_RESOURCE;

  // Change to horizontal camera frame
  Transformation T_G_CH = T_G_C * transformer_.getModelTransform();
  Eigen::Quaternionf quatrot = T_G_CH.getEigenQuaternion();
  Point quat_vec = quatrot.vec();
  robot_model.pose.orientation.x = quat_vec(0);
  robot_model.pose.orientation.y = quat_vec(1);
  robot_model.pose.orientation.z = quat_vec(2);
  robot_model.pose.orientation.w = quatrot.w();
  Point translation = T_G_CH.getPosition();
  robot_model.pose.position.x = translation(0);
  robot_model.pose.position.y = translation(1);
  robot_model.pose.position.z = translation(2);
  robot_model_pub_.publish(robot_model);
}

// Checks if we can get the next message from queue.
bool NpTsdfServer::getNextPointcloudFromQueue(
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr>* queue,
    sensor_msgs::msg::PointCloud2::SharedPtr* pointcloud_msg, Transformation* T_G_C) {
  const size_t max_queue_size =
      std::max<size_t>(1u, queue_config_.max_pointcloud_queue_size);
  if (queue->empty()) {
    return false;
  }
  *pointcloud_msg = queue->front();

  if (transformer_.lookupTransform(
          sensor_frame_, world_frame_, (*pointcloud_msg)->header.stamp,
          T_G_C)) {
    queue->pop();
    return true;
  } else {
    if (queue->size() >= max_queue_size) {
      ROS_ERROR_THROTTLE(
          60,
          "Input pointcloud queue getting too long! Dropping "
          "some pointclouds. Either unable to look up transform "
          "timestamps or the processing is taking too long. queue_size=%zu "
          "max_queue_size=%zu stamp=%.6f",
          queue->size(), max_queue_size,
          ros::Time((*pointcloud_msg)->header.stamp).toSec());
      while (queue->size() >= max_queue_size) {
        queue->pop();
      }
    }
  }
  return false;
}

void NpTsdfServer::prunePointcloudQueue(
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr>* queue,
    const std::string& queue_name,
    const builtin_interfaces::msg::Time& newest_stamp) {
  CHECK_NOTNULL(queue);
  const size_t max_queue_size =
      std::max<size_t>(1u, queue_config_.max_pointcloud_queue_size);
  size_t dropped_by_count = 0u;
  while (queue->size() > max_queue_size) {
    queue->pop();
    ++dropped_by_count;
  }

  size_t dropped_by_age = 0u;
  if (queue_config_.max_pointcloud_queue_age_sec > 0.0) {
    const ros::Time newest_time(newest_stamp);
    while (!queue->empty() &&
           (newest_time - queue->front()->header.stamp).toSec() >
               queue_config_.max_pointcloud_queue_age_sec) {
      queue->pop();
      ++dropped_by_age;
    }
  }

  if (dropped_by_count > 0u || dropped_by_age > 0u) {
    ROS_WARN_THROTTLE(
        10.0,
        "Pruned %s pointcloud queue: dropped_by_count=%zu dropped_by_age=%zu "
        "queue_size=%zu max_queue_size=%zu max_queue_age_sec=%.3f "
        "newest_stamp=%.6f",
        queue_name.c_str(), dropped_by_count, dropped_by_age, queue->size(),
        max_queue_size, queue_config_.max_pointcloud_queue_age_sec,
        ros::Time(newest_stamp).toSec());
  }
}

void NpTsdfServer::insertPointcloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr& pointcloud_msg_in) {
  if (shutdown_requested_.load() || !ros::ok()) {
    return;
  }

  if (pointcloud_msg_in->header.stamp - last_msg_time_ptcloud_ >
      min_time_between_msgs_) {
    last_msg_time_ptcloud_ = pointcloud_msg_in->header.stamp;
    // So we have to process the queue anyway... Push this back.
    pointcloud_queue_.push(pointcloud_msg_in);
    prunePointcloudQueue(
        &pointcloud_queue_, "input", pointcloud_msg_in->header.stamp);
  }

  Transformation T_G_C;
  sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg;
  bool processed_any = false;
  while (
      getNextPointcloudFromQueue(&pointcloud_queue_, &pointcloud_msg, &T_G_C)) {
    constexpr bool is_freespace_pointcloud = false;
    // main processing entrance
    processPointCloudMessageAndInsert(
        pointcloud_msg, T_G_C, is_freespace_pointcloud);
    processed_any = true;
  }

  if (!processed_any) {
    return;
  }

  if (publish_pointclouds_on_update_) {
    publishPointclouds();
  }

  if (timing_)
    ROS_INFO_STREAM(
        "Frame [" << frame_count_ << "] timings: " << std::endl
                  << timing::Timing::Print());
  if (verbose_)
    ROS_INFO_STREAM(
        "Layer memory: " << tsdf_map_->getTsdfLayer().getMemorySize());
  frame_count_++;
}

void NpTsdfServer::insertFreespacePointcloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr& pointcloud_msg_in) {
  if (shutdown_requested_.load() || !ros::ok()) {
    return;
  }

  if (pointcloud_msg_in->header.stamp - last_msg_time_freespace_ptcloud_ >
      min_time_between_msgs_) {
    last_msg_time_freespace_ptcloud_ = pointcloud_msg_in->header.stamp;
    // So we have to process the queue anyway... Push this back.
    freespace_pointcloud_queue_.push(pointcloud_msg_in);
    prunePointcloudQueue(
        &freespace_pointcloud_queue_, "freespace",
        pointcloud_msg_in->header.stamp);
  }

  Transformation T_G_C;
  sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg;
  while (getNextPointcloudFromQueue(
      &freespace_pointcloud_queue_, &pointcloud_msg, &T_G_C)) {
    constexpr bool is_freespace_pointcloud = true;
    processPointCloudMessageAndInsert(
        pointcloud_msg, T_G_C, is_freespace_pointcloud);
  }
}

void NpTsdfServer::integratePointcloud(
    const Transformation& T_G_C, const Pointcloud& points_C,
    const Pointcloud& normals_C, const Colors& colors,
    const bool is_freespace_pointcloud) {
  CHECK_EQ(points_C.size(), colors.size());
  CHECK_EQ(points_C.size(), normals_C.size());
  tsdf_integrator_->integratePointCloud(
      T_G_C, points_C, normals_C, colors, is_freespace_pointcloud);
}

void NpTsdfServer::publishAllUpdatedTsdfVoxels() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud_d;
  createDistancePointcloudFromTsdfLayer(
      tsdf_map_->getTsdfLayer(), &pointcloud_d);
  pointcloud_d.header.frame_id = world_frame_;
  tsdf_pointcloud_pub_.publish(pointcloud_d);
  // Create a pointcloud with gradient direction = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud_g;
  createGradientPointcloudFromTsdfLayer(
      tsdf_map_->getTsdfLayer(), &pointcloud_g);
  pointcloud_g.header.frame_id = world_frame_;
  gsdf_pointcloud_pub_.publish(pointcloud_g);
}

void NpTsdfServer::publishTsdfSurfacePoints() {
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  const float surface_distance_thresh =
      tsdf_map_->getTsdfLayer().voxel_size() * 0.75;
  createSurfacePointcloudFromTsdfLayer(
      tsdf_map_->getTsdfLayer(), surface_distance_thresh, &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  surface_pointcloud_pub_.publish(pointcloud);
}

void NpTsdfServer::publishTsdfOccupiedNodes() {
  visualization_msgs::msg::MarkerArray marker_array;
  createOccupancyBlocksFromTsdfLayer(
      tsdf_map_->getTsdfLayer(), world_frame_, &marker_array);
  occupancy_marker_pub_.publish(marker_array);
}

// Publish not only the tsdf value but also the signe distance
// gradient direction
void NpTsdfServer::publishSlices() {
  pcl::PointCloud<pcl::PointXYZI> pointcloud_d;
  createDistancePointcloudFromTsdfLayerSlice(
      tsdf_map_->getTsdfLayer(), 2, slice_level_, &pointcloud_d);
  pointcloud_d.header.frame_id = world_frame_;
  tsdf_slice_pub_.publish(pointcloud_d);

  pcl::PointCloud<pcl::PointXYZI> pointcloud_g;
  createGradientPointcloudFromTsdfLayerSlice(
      tsdf_map_->getTsdfLayer(), 2, slice_level_, &pointcloud_g);
  pointcloud_g.header.frame_id = world_frame_;
  gsdf_slice_pub_.publish(pointcloud_g);
}

void NpTsdfServer::publishMap(bool reset_remote_map) {
  if (!publish_tsdf_map_) {
    return;
  }
  int subscribers = this->tsdf_map_pub_.getNumSubscribers();
  if (subscribers > 0) {
    if (num_subscribers_tsdf_map_ < subscribers) {
      // Always reset the remote map and send all when a new subscriber
      // subscribes. A bit of overhead for other subscribers, but better than
      // inconsistent map states.
      reset_remote_map = true;
    }
    const bool only_updated = !reset_remote_map;
    timing::Timer publish_map_timer("map/publish_tsdf");
    voxblox_msgs::msg::Layer layer_msg;
    serializeLayerAsMsg<TsdfVoxel>(
        this->tsdf_map_->getTsdfLayer(), only_updated, &layer_msg);
    if (reset_remote_map) {
      layer_msg.action = static_cast<uint8_t>(MapDerializationAction::kReset);
    }
    this->tsdf_map_pub_.publish(layer_msg);
    publish_map_timer.Stop();
  }
  num_subscribers_tsdf_map_ = subscribers;
}

void NpTsdfServer::publishPointclouds() {
  // Combined function to publish all possible pointcloud messages -- surface
  // pointclouds, updated points, and occupied points.
  publishAllUpdatedTsdfVoxels();
  publishTsdfSurfacePoints();
  publishTsdfOccupiedNodes();
  if (publish_slices_) {
    publishSlices();
  }
}

void NpTsdfServer::updateMesh() {
  if (verbose_) {
    ROS_INFO("Updating mesh.");
  }

  timing::Timer generate_mesh_timer("mesh/update");
  constexpr bool only_mesh_updated_blocks = true;
  constexpr bool clear_updated_flag = true;
  mesh_integrator_->generateMesh(only_mesh_updated_blocks, clear_updated_flag);
  generate_mesh_timer.Stop();

  timing::Timer publish_mesh_timer("mesh/publish");

  voxblox_msgs::msg::Mesh mesh_msg;
  generateVoxbloxMeshMsg(mesh_layer_, color_mode_, &mesh_msg);
  mesh_msg.header.frame_id = world_frame_;
  mesh_pub_.publish(mesh_msg);

  if (cache_mesh_) {
    cached_mesh_msg_ = mesh_msg;
  }

  publish_mesh_timer.Stop();

  if (publish_pointclouds_ && !publish_pointclouds_on_update_) {
    publishPointclouds();
  }
}

bool NpTsdfServer::generateMesh() {
  timing::Timer generate_mesh_timer("mesh/generate");
  const bool clear_mesh = true;
  if (clear_mesh) {
    constexpr bool only_mesh_updated_blocks = false;
    constexpr bool clear_updated_flag = true;
    mesh_integrator_->generateMesh(
        only_mesh_updated_blocks, clear_updated_flag);
  } else {
    constexpr bool only_mesh_updated_blocks = true;
    constexpr bool clear_updated_flag = true;
    mesh_integrator_->generateMesh(
        only_mesh_updated_blocks, clear_updated_flag);
  }
  generate_mesh_timer.Stop();

  timing::Timer publish_mesh_timer("mesh/publish");
  voxblox_msgs::msg::Mesh mesh_msg;
  generateVoxbloxMeshMsg(mesh_layer_, color_mode_, &mesh_msg);
  mesh_msg.header.frame_id = world_frame_;
  mesh_pub_.publish(mesh_msg);

  publish_mesh_timer.Stop();

  if (!mesh_filename_.empty()) {
    timing::Timer output_mesh_timer("mesh/output");
    const bool success = outputMeshLayerAsPly(mesh_filename_, *mesh_layer_);
    output_mesh_timer.Stop();
    if (success) {
      ROS_INFO("Output file as PLY: %s", mesh_filename_.c_str());
    } else {
      ROS_INFO("Failed to output mesh as PLY: %s", mesh_filename_.c_str());
    }
  }

  if (timing_)
    ROS_INFO_STREAM("Mesh Timings: " << std::endl << timing::Timing::Print());
  return true;
}

bool NpTsdfServer::saveMap(const std::string& file_path) {
  // Inheriting classes should add saving other layers to this function.
  return io::SaveLayer(tsdf_map_->getTsdfLayer(), file_path);
}

bool NpTsdfServer::loadMap(const std::string& file_path) {
  // Inheriting classes should add other layers to load, as this will only
  // load
  // the TSDF layer.
  constexpr bool kMulitpleLayerSupport = true;
  bool success = io::LoadBlocksFromFile(
      file_path, Layer<TsdfVoxel>::BlockMergingStrategy::kReplace,
      kMulitpleLayerSupport, tsdf_map_->getTsdfLayerPtr());
  if (success) {
    LOG(INFO) << "Successfully loaded TSDF layer.";
  }
  return success;
}

bool NpTsdfServer::clearMapCallback(
    std_srvs::srv::Empty::Request& /*request*/, std_srvs::srv::Empty::Response&
    /*response*/) {  // NOLINT
  clear();
  return true;
}

bool NpTsdfServer::generateMeshCallback(
    std_srvs::srv::Empty::Request& /*request*/, std_srvs::srv::Empty::Response&
    /*response*/) {  // NOLINT
  return generateMesh();
}

bool NpTsdfServer::saveMapCallback(
    voxblox_msgs::srv::FilePath::Request& request, voxblox_msgs::srv::FilePath::Response&
    /*response*/) {  // NOLINT
  return saveMap(request.file_path);
}

bool NpTsdfServer::loadMapCallback(
    voxblox_msgs::srv::FilePath::Request& request, voxblox_msgs::srv::FilePath::Response&
    /*response*/) {  // NOLINT
  bool success = loadMap(request.file_path);
  return success;
}

bool NpTsdfServer::publishPointcloudsCallback(
    std_srvs::srv::Empty::Request& /*request*/, std_srvs::srv::Empty::Response&
    /*response*/) {  // NOLINT
  publishPointclouds();
  return true;
}

bool NpTsdfServer::publishTsdfMapCallback(
    std_srvs::srv::Empty::Request& /*request*/, std_srvs::srv::Empty::Response&
    /*response*/) {  // NOLINT
  publishMap();
  return true;
}

void NpTsdfServer::updateMeshEvent(const ros::TimerEvent& /*event*/) {
  if (shutdown_requested_.load() || !ros::ok()) {
    return;
  }
  updateMesh();
}

void NpTsdfServer::publishMapEvent(const ros::TimerEvent& /*event*/) {
  if (shutdown_requested_.load() || !ros::ok()) {
    return;
  }
  publishMap();
}

void NpTsdfServer::shutdown() {
  shutdown_requested_.store(true);
  if (update_mesh_timer_) {
    update_mesh_timer_->cancel();
    update_mesh_timer_.reset();
  }
  if (publish_map_timer_) {
    publish_map_timer_->cancel();
    publish_map_timer_.reset();
  }
  pointcloud_sub_.reset();
  freespace_pointcloud_sub_.reset();
  tsdf_map_sub_.reset();
  generate_mesh_srv_.reset();
  clear_map_srv_.reset();
  save_map_srv_.reset();
  load_map_srv_.reset();
  publish_pointclouds_srv_.reset();
  publish_tsdf_map_srv_.reset();
  std::queue<sensor_msgs::msg::PointCloud2::SharedPtr>().swap(pointcloud_queue_);
  std::queue<sensor_msgs::msg::PointCloud2::SharedPtr>().swap(
      freespace_pointcloud_queue_);
}

bool NpTsdfServer::shouldLogMemoryStatus() {
  if (memory_log_interval_sec_ < 0.0) {
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  const double elapsed =
      std::chrono::duration<double>(now - last_memory_log_time_).count();
  if (elapsed < memory_log_interval_sec_) {
    return false;
  }
  last_memory_log_time_ = now;
  return true;
}

double NpTsdfServer::getProcessRssMb() const {
  std::ifstream status_file("/proc/self/status");
  std::string line;
  while (std::getline(status_file, line)) {
    if (line.rfind("VmRSS:", 0) == 0) {
      std::istringstream stream(line);
      std::string label;
      double rss_kb = 0.0;
      std::string unit;
      stream >> label >> rss_kb >> unit;
      return rss_kb / 1024.0;
    }
  }
  return -1.0;
}

void NpTsdfServer::logMemoryStatus(
    const std::string& context, size_t cloud_points, size_t updated_blocks) {
  if (!shouldLogMemoryStatus()) {
    return;
  }

  const auto& tsdf_layer = tsdf_map_->getTsdfLayer();
  ROS_INFO(
      "[voxfield][mem] context=%s rss_mb=%.1f tsdf_blocks=%zu "
      "tsdf_layer_mb=%.2f updated_blocks=%zu cloud_points=%zu "
      "pointcloud_queue=%zu freespace_queue=%zu",
      context.c_str(), getProcessRssMb(),
      tsdf_layer.getNumberOfAllocatedBlocks(),
      static_cast<double>(tsdf_layer.getMemorySize()) / (1024.0 * 1024.0),
      updated_blocks, cloud_points, pointcloud_queue_.size(),
      freespace_pointcloud_queue_.size());
}

void NpTsdfServer::clear() {
  tsdf_map_->getTsdfLayerPtr()->removeAllBlocks();
  mesh_layer_->clear();

  // Publish a message to reset the map to all subscribers.
  if (publish_tsdf_map_) {
    constexpr bool kResetRemoteMap = true;
    publishMap(kResetRemoteMap);
  }
}

void NpTsdfServer::tsdfMapCallback(const voxblox_msgs::msg::Layer& layer_msg) {
  timing::Timer receive_map_timer("map/receive_tsdf");

  bool success =
      deserializeMsgToLayer<TsdfVoxel>(layer_msg, tsdf_map_->getTsdfLayerPtr());

  if (!success) {
    ROS_ERROR_THROTTLE(10, "Got an invalid TSDF map message!");
  } else {
    ROS_INFO_ONCE("Got an TSDF map from ROS topic!");
    if (publish_pointclouds_on_update_) {
      publishPointclouds();
    }
  }
}

bool NpTsdfServer::projectPointCloudToImage(
    const Pointcloud& points_C, const Colors& colors,
    cv::Mat& vertex_map,   // corresponding point // NOLINT
    cv::Mat& depth_image,  // Float depth image (CV_32FC1). // NOLINT
    cv::Mat& color_image,
    float min_z, 
    float min_d) const {
  // TODO(py): consider to calculate in parallel to speed up
  for (size_t i = 0; i < points_C.size(); i++) {
    const ProjectionResult projection =
        sensor_is_lidar_ ? projectPointToImageLiDAR(points_C[i])
                         : projectPointToImageCamera(points_C[i]);
    if (projection.valid && projection.depth > min_d &&
        points_C[i].z() > min_z) {
      float old_depth = depth_image.at<float>(projection.v, projection.u);
      // save only nearest point for each pixel
      if (old_depth <= 0.0 || old_depth > projection.depth) {
        for (int k = 0; k <= 2; k++) {
          vertex_map.at<cv::Vec3f>(projection.v, projection.u)[k] =
              points_C[i](k);
        }
        depth_image.at<float>(projection.v, projection.u) = projection.depth;
        // BGR default order
        color_image.at<cv::Vec3b>(projection.v, projection.u)[0] = colors[i].b;
        color_image.at<cv::Vec3b>(projection.v, projection.u)[1] = colors[i].g;
        color_image.at<cv::Vec3b>(projection.v, projection.u)[2] = colors[i].r;
      }
    }
  }
  return false;
}

// point should be in the LiDAR's coordinate system
NpTsdfServer::ProjectionResult NpTsdfServer::projectPointToImageLiDAR(
    const Point& p_C) const {
  ProjectionResult result;
  // All values are ceiled and floored to guarantee that the resulting points
  // will be valid for any integer conversion.
  float depth =
      std::sqrt(p_C.x() * p_C.x() + p_C.y() * p_C.y() + p_C.z() * p_C.z());
  if (depth <= 0.0f) {
    return result;
  }
  float yaw = std::atan2(p_C.y(), p_C.x());
  float pitch = std::asin(p_C.z() / depth);
  // projections in image coordinates (percentage)
  float proj_x = 0.5 * (yaw / M_PI + 1.0);
  float proj_y = 1.0 - (pitch - fov_down_rad_) / fov_rad_;
  // scale to image size
  proj_x *= width_;
  proj_y *= height_;
  // round for integer index
  result.u = std::round(proj_x);
  if (result.u == width_)
    result.u = 0;

  result.v = std::round(proj_y);
  if (std::ceil(proj_y) > height_ - 1 || std::floor(proj_y) < 0) {
    return result;
  }
  result.depth = depth;
  result.valid = true;
  return result;
}

NpTsdfServer::ProjectionResult NpTsdfServer::projectPointToImageCamera(
    const Point& p_C) const {
  ProjectionResult result;
  if (p_C.z() <= 0.0f) {
    return result;
  }
  result.u = std::round(p_C.x() * fx_ / p_C.z() + vx_);
  if (result.u >= width_ || result.u < 0) {
    return result;
  }
  result.v = std::round(p_C.y() * fy_ / p_C.z() + vy_);
  if (result.v >= height_ || result.v < 0) {
    return result;
  }
  result.depth = p_C.z();
  result.valid = true;
  return result;
}

cv::Mat NpTsdfServer::computeNormalImage(
    const cv::Mat& vertex_map, const cv::Mat& depth_image) const {
  cv::Mat normal_image(depth_image.size(), CV_32FC3, 0.0);
  for (int u = 0; u < width_; u++) {
    for (int v = 0; v < height_; v++) {
      if (v == 0 || v == height_ - 1) {
        continue;
      }
      Point p;
      p << vertex_map.at<cv::Vec3f>(v, u)[0], vertex_map.at<cv::Vec3f>(v, u)[1],
          vertex_map.at<cv::Vec3f>(v, u)[2];

      float d_p = depth_image.at<float>(v, u);

      if (d_p > 0) {
        // neighbor x (in ring)
        int n_x_u;
        if (u == width_ - 1)
          n_x_u = 0;
        else
          n_x_u = u + 1;
        Point n_x;
        n_x << vertex_map.at<cv::Vec3f>(v, n_x_u)[0],
            vertex_map.at<cv::Vec3f>(v, n_x_u)[1],
            vertex_map.at<cv::Vec3f>(v, n_x_u)[2];
        float d_n_x = depth_image.at<float>(v, n_x_u);
        if (d_n_x < 0)
          continue;
        // on the boundary, not continous
        if (std::abs(d_n_x - d_p) > smooth_thre_ratio_ * d_p)
          continue;

        // neighbor y
        const int n_y_v = v + 1;
        Point n_y;
        n_y << vertex_map.at<cv::Vec3f>(n_y_v, u)[0],
            vertex_map.at<cv::Vec3f>(n_y_v, u)[1],
            vertex_map.at<cv::Vec3f>(n_y_v, u)[2];

        float d_n_y = depth_image.at<float>(n_y_v, u);
        if (d_n_y < 0)
          continue;
        // on the boundary, not continous
        if (std::abs(d_n_y - d_p) > smooth_thre_ratio_ * d_p)
          continue;
        Point dx = n_x - p;
        Point dy = n_y - p;

        Point normal = (dx.cross(dy)).normalized();
        cv::Vec3f& normals = normal_image.at<cv::Vec3f>(v, u);
        for (int k = 0; k <= 2; k++)
          normals[k] = normal(k);
      }
    }
  }
  return normal_image;
}

Pointcloud NpTsdfServer::extractPointCloud(
    const cv::Mat& vertex_map, const cv::Mat& depth_image) const {
  Pointcloud points_C;
  for (int v = 0; v < vertex_map.rows; v++) {
    for (int u = 0; u < vertex_map.cols; u++) {
      cv::Vec3f vertex = vertex_map.at<cv::Vec3f>(v, u);
      if (depth_image.at<float>(v, u) > 0) {
        Point p_C(vertex[0], vertex[1], vertex[2]);
        points_C.push_back(p_C);
      }
    }
  }
  return points_C;
}

Colors NpTsdfServer::extractColors(
    const cv::Mat& color_image, const cv::Mat& depth_image) const {
  Colors colors;
  for (int v = 0; v < color_image.rows; v++) {
    for (int u = 0; u < color_image.cols; u++) {
      // BGR
      cv::Vec3b color = color_image.at<cv::Vec3b>(v, u);
      if (depth_image.at<float>(v, u) > 0) {
        // RGB
        Color c_C(color[2], color[1], color[0]);
        colors.push_back(c_C);
      }
    }
  }
  return colors;
}

Pointcloud NpTsdfServer::extractNormals(
    const cv::Mat& normal_image, const cv::Mat& depth_image) const {
  Pointcloud normals_C;
  for (int v = 0; v < normal_image.rows; v++) {
    for (int u = 0; u < normal_image.cols; u++) {
      cv::Vec3f vertex = normal_image.at<cv::Vec3f>(v, u);
      if (depth_image.at<float>(v, u) > 0) {
        Ray n_C(vertex[0], vertex[1], vertex[2]);
        normals_C.push_back(n_C);
      }
    }
  }
  return normals_C;
}

}  // namespace voxblox
