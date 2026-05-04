#ifndef VOXBLOX_ROS_COMPAT_TF_TRANSFORM_LISTENER_H_
#define VOXBLOX_ROS_COMPAT_TF_TRANSFORM_LISTENER_H_

#include <memory>
#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf/transform_broadcaster.h>

namespace tf {

using TransformException = tf2::TransformException;

class TransformListener {
 public:
  TransformListener()
      : buffer_(std::make_shared<tf2_ros::Buffer>(ros::globalNode()->get_clock())),
        listener_(std::make_shared<tf2_ros::TransformListener>(*buffer_)) {}

  bool canTransform(
      const std::string& target_frame,
      const std::string& source_frame,
      const ros::Time& time) const {
    return buffer_->canTransform(
        target_frame, source_frame, time.rclcppTime(),
        tf2::durationFromSec(0.0));
  }

  void lookupTransform(
      const std::string& target_frame,
      const std::string& source_frame,
      const ros::Time& time,
      geometry_msgs::msg::TransformStamped& transform) const {
    transform = buffer_->lookupTransform(
        target_frame, source_frame, time.rclcppTime(),
        tf2::durationFromSec(0.0));
  }

 private:
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
};

}  // namespace tf

#endif  // VOXBLOX_ROS_COMPAT_TF_TRANSFORM_LISTENER_H_
