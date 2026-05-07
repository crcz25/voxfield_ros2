#include "voxblox_ros/transformer.h"

#include <algorithm>
#include <vector>

#include "voxblox_ros/ros_params.h"

namespace voxblox {
namespace {

bool getTransformParam(
    const ros::NodeHandle& nh_private, const std::string& name,
    Transformation* transform) {
  CHECK_NOTNULL(transform);
  std::vector<double> values;
  if (!nh_private.getParam(name, values) || values.size() != 16u) {
    return false;
  }

  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  for (size_t row = 0; row < 4; ++row) {
    for (size_t col = 0; col < 4; ++col) {
      matrix(row, col) = static_cast<float>(values[row * 4 + col]);
    }
  }
  *transform = Transformation(matrix);
  return true;
}

}  // namespace

// TODO(py): change the name of these transformation, current name
// is hard to understand
Transformer::Transformer(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      world_frame_("world"),
      sensor_frame_(""),
      use_tf_transforms_(true),
      timestamp_tolerance_ns_(1000000) {
  nh_private_.param("world_frame", world_frame_, world_frame_);
  nh_private_.param("sensor_frame", sensor_frame_, sensor_frame_);

  const double kNanoSecondsInSecond = 1.0e9;
  double timestamp_tolerance_sec =
      timestamp_tolerance_ns_ / kNanoSecondsInSecond;
  nh_private_.param(
      "timestamp_tolerance_sec", timestamp_tolerance_sec,
      timestamp_tolerance_sec);
  timestamp_tolerance_ns_ =
      static_cast<int64_t>(timestamp_tolerance_sec * kNanoSecondsInSecond);

  // Transform settings.
  nh_private_.param(
      "use_tf_transforms", use_tf_transforms_, use_tf_transforms_);
  QueueConfig queue_config;
  queue_config.max_transform_queue_size = max_transform_queue_size_;
  queue_config.max_transform_queue_age_sec = max_transform_queue_age_sec_;
  queue_config = getQueueConfigFromRosParam(nh_private_, queue_config);
  max_transform_queue_size_ = queue_config.max_transform_queue_size;
  max_transform_queue_age_sec_ = queue_config.max_transform_queue_age_sec;
  // If we use topic transforms, we have 2 parts: a dynamic transform from a
  // topic and a static transform from parameters (calibration).
  // Dynamic transform should be T_G_D (where D is whatever sensor the
  // dynamic coordinate frame is in) and the static should be T_D_C (where
  // C is the sensor frame that produces the depth data). It is possible to
  // specify T_C_D and set invert_static_tranform to true.
  if (!use_tf_transforms_) {
    transform_sub_ =
        nh_.subscribe("transform", 40, &Transformer::transformCallback, this);
    // TODO(helenol): split out into a function to avoid duplication.
    if (getTransformParam(nh_private_, "T_B_D", &T_B_D_)) {
      // See if we need to invert it.
      bool invert_static_tranform = false;
      nh_private_.param(
          "invert_T_B_D", invert_static_tranform, invert_static_tranform);
      if (invert_static_tranform) {
        T_B_D_ = T_B_D_.inverse();
      }
    }
    if (getTransformParam(nh_private_, "T_B_C", &T_B_C_)) {
      // See if we need to invert it.
      bool invert_static_tranform = false;
      nh_private_.param(
          "invert_T_B_C", invert_static_tranform, invert_static_tranform);
      if (invert_static_tranform) {
        T_B_C_ = T_B_C_.inverse();
      }
    }
  }
  T_D_C_ = T_B_D_.inverse() * T_B_C_;
  // Or we will use tf_transform, we do not need the calibration parameters
  // lookupTransformTf
  // Model transformation
  if (getTransformParam(nh_private_, "T_C_CH", &T_C_CH_)) {
    // See if we need to invert it.
    bool invert_static_tranform = false;
    nh_private_.param(
        "invert_T_C_CH", invert_static_tranform, invert_static_tranform);
    if (invert_static_tranform) {
      T_C_CH_ = T_C_CH_.inverse();
    }
  }
}

void Transformer::transformCallback(
    const geometry_msgs::msg::TransformStamped& transform_msg) {
  transform_queue_.push_back(transform_msg);
  pruneTransformQueue(transform_msg.header.stamp);
}

void Transformer::pruneTransformQueue(const ros::Time& newest_stamp) {
  const size_t max_queue_size = std::max<size_t>(1u, max_transform_queue_size_);
  size_t dropped_by_count = 0u;
  while (transform_queue_.size() > max_queue_size) {
    transform_queue_.pop_front();
    ++dropped_by_count;
  }

  size_t dropped_by_age = 0u;
  if (max_transform_queue_age_sec_ > 0.0) {
    while (!transform_queue_.empty() &&
           (newest_stamp - transform_queue_.front().header.stamp).toSec() >
               max_transform_queue_age_sec_) {
      transform_queue_.pop_front();
      ++dropped_by_age;
    }
  }

  if (dropped_by_count > 0u || dropped_by_age > 0u) {
    ROS_WARN_STREAM_THROTTLE(
        10.0,
        "Pruned transform queue: dropped_by_count="
            << dropped_by_count << " dropped_by_age=" << dropped_by_age
            << " queue_size=" << transform_queue_.size()
            << " max_queue_size=" << max_queue_size
            << " max_queue_age_sec=" << max_transform_queue_age_sec_
            << " newest_stamp=" << newest_stamp);
  }
}

Transformation Transformer::getStaticTransform() const {
  return T_B_C_;
}

Transformation Transformer::getModelTransform() const {
  return T_C_CH_;
}

bool Transformer::lookupTransform(
    const std::string& from_frame, const std::string& to_frame,
    const ros::Time& timestamp, Transformation* transform) {
  CHECK_NOTNULL(transform);
  if (use_tf_transforms_) {
    return lookupTransformTf(from_frame, to_frame, timestamp, transform);
  } else {
    return lookupTransformQueue(timestamp, transform);
  }
}

// Stolen from octomap_manager
bool Transformer::lookupTransformTf(
    const std::string& from_frame, const std::string& to_frame,
    const ros::Time& timestamp, Transformation* transform) {
  CHECK_NOTNULL(transform);
  geometry_msgs::msg::TransformStamped tf_transform;
  ros::Time time_to_lookup = timestamp;

  // Allow overwriting the TF frame for the sensor.
  std::string from_frame_modified = from_frame;
  if (!sensor_frame_.empty()) {
    from_frame_modified = sensor_frame_;
  }

  // Previous behavior was just to use the latest transform if the time is in
  // the future. Now we will just wait.
  if (!tf_listener_.canTransform(
          to_frame, from_frame_modified, time_to_lookup)) {
    return false;
  }

  try {
    tf_listener_.lookupTransform(
        to_frame, from_frame_modified, time_to_lookup, tf_transform);
  } catch (tf::TransformException& ex) {  // NOLINT
    ROS_ERROR_STREAM(
        "Error getting TF transform from sensor data: " << ex.what());
    return false;
  }

  tf::transformTFToKindr(tf_transform, transform);
  return true;
}

bool Transformer::lookupTransformQueue(
    const ros::Time& timestamp, Transformation* transform) {
  CHECK_NOTNULL(transform);
  if (transform_queue_.empty()) {
    ROS_WARN_STREAM_THROTTLE(
        30, "No match found for transform timestamp: "
                << timestamp << " as transform queue is empty.");
    return false;
  }
  // Try to match the transforms in the queue.
  bool match_found = false;
  std::deque<geometry_msgs::msg::TransformStamped>::iterator it =
      transform_queue_.begin();
  for (; it != transform_queue_.end(); ++it) {
    // If the current transform is newer than the requested timestamp, we need
    // to break.
    if (it->header.stamp > timestamp) {
      if ((it->header.stamp - timestamp).toNSec() < timestamp_tolerance_ns_) {
        match_found = true;
      }
      break;
    }

    if ((timestamp - it->header.stamp).toNSec() < timestamp_tolerance_ns_) {
      match_found = true;
      break;
    }
  }

  // Match found basically means an exact match.
  Transformation T_G_D;
  if (match_found) {
    tf::transformMsgToKindr(it->transform, &T_G_D);
  } else {
    // If we think we have an inexact match, have to check that we're still
    // within bounds and interpolate.
    if (it == transform_queue_.begin() || it == transform_queue_.end()) {
      ROS_WARN_STREAM_THROTTLE(
          30, "No match found for transform timestamp: "
                  << timestamp
                  << " Queue front: " << transform_queue_.front().header.stamp
                  << " back: " << transform_queue_.back().header.stamp);
      return false;
    }
    // Newest should be 1 past the requested timestamp, oldest should be one
    // before the requested timestamp.
    Transformation T_G_D_newest;
    tf::transformMsgToKindr(it->transform, &T_G_D_newest);
    int64_t offset_newest_ns = (it->header.stamp - timestamp).toNSec();
    // We already checked that this is not the beginning.
    it--;
    Transformation T_G_D_oldest;
    tf::transformMsgToKindr(it->transform, &T_G_D_oldest);
    int64_t offset_oldest_ns = (timestamp - it->header.stamp).toNSec();

    // Interpolate between the two transformations using the exponential map.
    FloatingPoint t_diff_ratio =
        static_cast<FloatingPoint>(offset_oldest_ns) /
        static_cast<FloatingPoint>(offset_newest_ns + offset_oldest_ns);

    Transformation::Vector6 diff_vector =
        (T_G_D_oldest.inverse() * T_G_D_newest).log();
    T_G_D = T_G_D_oldest * Transformation::exp(t_diff_ratio * diff_vector);
  }

  // If we have a static transform, apply it too.
  // Transform should actually be T_G_C. So need to take it through the full
  // chain. transform is T_G_C
  *transform = T_G_D * T_D_C_;

  // And also clear the queue up to this point. This leaves the current
  // message in place.
  transform_queue_.erase(transform_queue_.begin(), it);
  return true;
}

}  // namespace voxblox
