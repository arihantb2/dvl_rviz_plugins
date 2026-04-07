#include "dvl_rviz_plugins/dvl_display.hpp"

#include <QColor>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/status_property.hpp>

#include "dvl_rviz_plugins/dvl_visual.hpp"

namespace dvl_rviz_plugins
{

DvlDisplay::DvlDisplay()
{
  velocity_color_prop_ = new rviz_common::properties::ColorProperty(
    "Velocity Color", QColor(0, 255, 0),
    "Color of the velocity arrow.",
    this);

  velocity_scale_prop_ = new rviz_common::properties::FloatProperty(
    "Velocity Scale", 1.0f,
    "Scale multiplier applied to the velocity arrow length (m/s -> metres).",
    this);
  velocity_scale_prop_->setMin(0.0f);

  show_beams_prop_ = new rviz_common::properties::BoolProperty(
    "Show Beams", true,
    "Show the four acoustic beam lines.",
    this);

  beam_color_prop_ = new rviz_common::properties::ColorProperty(
    "Beam Color", QColor(255, 165, 0),
    "Color of the acoustic beam lines.",
    this);

  show_covariance_prop_ = new rviz_common::properties::BoolProperty(
    "Show Covariance", true,
    "Show the velocity covariance ellipsoid.",
    this);

  covariance_color_prop_ = new rviz_common::properties::ColorProperty(
    "Covariance Color", QColor(200, 200, 0),
    "Color of the covariance ellipsoid.",
    this);

  covariance_scale_prop_ = new rviz_common::properties::FloatProperty(
    "Covariance Scale", 1.0f,
    "Scale multiplier applied to each ellipsoid semi-axis (sqrt of eigenvalue).",
    this);
  covariance_scale_prop_->setMin(0.0f);
}

void DvlDisplay::onInitialize()
{
  MFDClass::onInitialize();
  visual_ = std::make_shared<DvlVisual>(context_->getSceneManager(), scene_node_);
}

void DvlDisplay::reset()
{
  MFDClass::reset();
  visual_ = std::make_shared<DvlVisual>(context_->getSceneManager(), scene_node_);
}

void DvlDisplay::processMessage(marine_acoustic_msgs::msg::Dvl::ConstSharedPtr msg)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Transform",
      "Error transforming from frame '" +
      QString::fromStdString(msg->header.frame_id) + "' to fixed frame.");
    return;
  }

  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);

  visual_->setVelocityColor(velocity_color_prop_->getOgreColor());
  visual_->setVelocityScale(velocity_scale_prop_->getFloat());
  visual_->setShowBeams(show_beams_prop_->getBool());
  visual_->setBeamColor(beam_color_prop_->getOgreColor());
  visual_->setShowCovariance(show_covariance_prop_->getBool());
  visual_->setCovarianceColor(covariance_color_prop_->getOgreColor());
  visual_->setCovarianceScale(covariance_scale_prop_->getFloat());

  visual_->setMessage(msg);
}

}  // namespace dvl_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(dvl_rviz_plugins::DvlDisplay, rviz_common::Display)
