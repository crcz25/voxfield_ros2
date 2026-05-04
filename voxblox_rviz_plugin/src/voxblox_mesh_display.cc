#include "voxblox_rviz_plugin/voxblox_mesh_display.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <voxblox_rviz_plugin/material_loader.h>

namespace voxblox_rviz_plugin {

VoxbloxMeshDisplay::VoxbloxMeshDisplay()
    : visible_property_(
          "Visible", true,
          "Show or hide the mesh. If the mesh is hidden but not disabled, it "
          "will persist and is incrementally built in the background.",
          this, SLOT(visibleSLOT())) {
  voxblox_rviz_plugin::MaterialLoader::loadMaterials();
}

void VoxbloxMeshDisplay::reset() {
  MFDClass::reset();
  visual_.reset();
}

void VoxbloxMeshDisplay::visibleSLOT() {
  if (visual_) {
    // Set visibility and update the pose if visibility is turned on.
    visual_->setEnabled(visible_property_.getBool());
    if (visible_property_.getBool()) {
      updateTransformation();
    }
  }
}

void VoxbloxMeshDisplay::processMessage(
    voxblox_msgs::msg::Mesh::ConstSharedPtr msg) {
  if (!visual_) {
    visual_.reset(
        new VoxbloxMeshVisual(context_->getSceneManager(), scene_node_));
    visual_->setEnabled(visible_property_.getBool());
  }

  // update the frame, pose and mesh of the visual
  visual_->setFrameId(msg->header.frame_id);
  if (updateTransformation(rclcpp::Time(msg->header.stamp, RCL_ROS_TIME))) {
    visual_->setMessage(msg);
  }
}

bool VoxbloxMeshDisplay::updateTransformation() {
  if (!visual_) {
    // can not get the transform if we don't have a visual
    return false;
  }
  // Look up the transform from tf. If it doesn't work we have to skip.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          visual_->getFrameId(), position, orientation)) {
    setStatus(
        rviz_common::properties::StatusProperty::Error, "Transform",
        QString("Error transforming from frame '%1' to frame '%2'")
            .arg(QString::fromStdString(visual_->getFrameId()), fixed_frame_));
    return false;
  }
  visual_->setPose(position, orientation);
  setStatus(rviz_common::properties::StatusProperty::Ok, "Transform", "OK");
  return true;
}

bool VoxbloxMeshDisplay::updateTransformation(rclcpp::Time stamp) {
  if (!visual_) {
    // can not get the transform if we don't have a visual
    return false;
  }
  // Look up the transform from tf. If it doesn't work we have to skip.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          visual_->getFrameId(), stamp, position, orientation)) {
    setStatus(
        rviz_common::properties::StatusProperty::Error, "Transform",
        QString("Error transforming from frame '%1' to frame '%2'")
            .arg(QString::fromStdString(visual_->getFrameId()), fixed_frame_));
    return false;
  }
  visual_->setPose(position, orientation);
  setStatus(rviz_common::properties::StatusProperty::Ok, "Transform", "OK");
  return true;
}

void VoxbloxMeshDisplay::fixedFrameChanged() {
  if (tf_filter_) {
    tf_filter_->setTargetFrame(fixed_frame_.toStdString());
  }
  // update the transformation of the visual w.r.t fixed frame
  updateTransformation();
}

}  // namespace voxblox_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    voxblox_rviz_plugin::VoxbloxMeshDisplay, rviz_common::Display)
