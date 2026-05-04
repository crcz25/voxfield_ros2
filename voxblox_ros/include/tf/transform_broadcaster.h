#ifndef VOXBLOX_ROS_COMPAT_TF_TRANSFORM_BROADCASTER_H_
#define VOXBLOX_ROS_COMPAT_TF_TRANSFORM_BROADCASTER_H_

#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <ros/ros.h>
#include <voxblox/core/common.h>

namespace tf {

inline void transformKindrToMsg(
    const voxblox::Transformation& transform,
    geometry_msgs::msg::Transform* msg);

class Transform {
 public:
  Transform() : transform_(voxblox::Transformation()) {}
  explicit Transform(const voxblox::Transformation& transform)
      : transform_(transform) {}

  const voxblox::Transformation& asKindr() const {
    return transform_;
  }

 private:
  voxblox::Transformation transform_;
};

class StampedTransform {
 public:
  StampedTransform(
      const Transform& transform,
      const builtin_interfaces::msg::Time& stamp,
      const std::string& frame_id,
      const std::string& child_frame_id)
      : transform_(transform),
        stamp_(stamp),
        frame_id_(frame_id),
        child_frame_id_(child_frame_id) {}

  geometry_msgs::msg::TransformStamped toMsg() const {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp_;
    msg.header.frame_id = frame_id_;
    msg.child_frame_id = child_frame_id_;
    transformKindrToMsg(transform_.asKindr(), &msg.transform);
    return msg;
  }

 private:
  Transform transform_;
  builtin_interfaces::msg::Time stamp_;
  std::string frame_id_;
  std::string child_frame_id_;
};

inline void transformKindrToMsg(
    const voxblox::Transformation& transform,
    geometry_msgs::msg::Transform* msg) {
  const Eigen::Quaternionf q = transform.getEigenQuaternion();
  const voxblox::Point p = transform.getPosition();
  msg->translation.x = p.x();
  msg->translation.y = p.y();
  msg->translation.z = p.z();
  msg->rotation.x = q.x();
  msg->rotation.y = q.y();
  msg->rotation.z = q.z();
  msg->rotation.w = q.w();
}

template <typename Scalar>
inline void transformKindrToMsg(
    const kindr::minimal::QuatTransformationTemplate<Scalar>& transform,
    geometry_msgs::msg::Transform* msg) {
  auto cast_transform = transform.template cast<voxblox::FloatingPoint>();
  transformKindrToMsg(cast_transform, msg);
}

inline void transformKindrToTF(
    const voxblox::Transformation& transform, Transform* tf_transform) {
  *tf_transform = Transform(transform);
}

template <typename Scalar>
inline void transformKindrToTF(
    const kindr::minimal::QuatTransformationTemplate<Scalar>& transform,
    Transform* tf_transform) {
  *tf_transform = Transform(transform.template cast<voxblox::FloatingPoint>());
}

inline void transformMsgToKindr(
    const geometry_msgs::msg::Transform& msg,
    voxblox::Transformation* transform) {
  Eigen::Quaternionf q(
      static_cast<float>(msg.rotation.w), static_cast<float>(msg.rotation.x),
      static_cast<float>(msg.rotation.y), static_cast<float>(msg.rotation.z));
  q.normalize();
  voxblox::Point p(
      static_cast<float>(msg.translation.x),
      static_cast<float>(msg.translation.y),
      static_cast<float>(msg.translation.z));
  *transform = voxblox::Transformation(voxblox::Rotation(q), p);
}

inline void transformTFToKindr(
    const geometry_msgs::msg::TransformStamped& msg,
    voxblox::Transformation* transform) {
  transformMsgToKindr(msg.transform, transform);
}

inline void pointEigenToMsg(
    const Eigen::Vector3d& point, geometry_msgs::msg::Point& msg) {
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
}

class TransformBroadcaster {
 public:
  TransformBroadcaster() = default;

  void sendTransform(const StampedTransform& transform) {
    ensureBroadcaster();
    broadcaster_->sendTransform(transform.toMsg());
  }

  void sendTransform(const geometry_msgs::msg::TransformStamped& transform) {
    ensureBroadcaster();
    broadcaster_->sendTransform(transform);
  }

 private:
  void ensureBroadcaster() {
    if (!broadcaster_) {
      broadcaster_ =
          std::make_shared<tf2_ros::TransformBroadcaster>(ros::globalNode());
    }
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

}  // namespace tf

#endif  // VOXBLOX_ROS_COMPAT_TF_TRANSFORM_BROADCASTER_H_
