#include "dvl_rviz_plugins/dvl_visual.hpp"

#include <algorithm>
#include <cmath>

#include <OgreMatrix3.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector.h>

#include <Eigen/Dense>

#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/shape.hpp>

namespace dvl_rviz_plugins
{

DvlVisual::DvlVisual(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node)
: scene_manager_(scene_manager),
  velocity_color_(0.0f, 1.0f, 0.0f, 1.0f),   // green
  beam_color_(1.0f, 0.65f, 0.0f, 1.0f)         // orange
{
  frame_node_ = parent_node->createChildSceneNode();

  velocity_arrow_ = std::make_unique<rviz_rendering::Arrow>(scene_manager_, frame_node_);

  for (auto & line : beam_lines_) {
    line = std::make_unique<rviz_rendering::BillboardLine>(scene_manager_, frame_node_);
    line->setMaxPointsPerLine(2);
    line->setNumLines(1);
    line->setLineWidth(0.02f);
  }

  covariance_sphere_ = std::make_unique<rviz_rendering::Shape>(
    rviz_rendering::Shape::Sphere, scene_manager_, frame_node_);
  covariance_sphere_->setColor(0.8f, 0.8f, 0.0f, 0.4f);
}

DvlVisual::~DvlVisual()
{
  // Unique_ptrs destroy Ogre objects; then remove the scene node.
  velocity_arrow_.reset();
  for (auto & line : beam_lines_) {
    line.reset();
  }
  covariance_sphere_.reset();
  scene_manager_->destroySceneNode(frame_node_);
}

void DvlVisual::setFramePosition(const Ogre::Vector3 & position)
{
  frame_node_->setPosition(position);
}

void DvlVisual::setFrameOrientation(const Ogre::Quaternion & orientation)
{
  frame_node_->setOrientation(orientation);
}

void DvlVisual::setVelocityColor(const Ogre::ColourValue & color)
{
  velocity_color_ = color;
}

void DvlVisual::setVelocityScale(float scale)
{
  velocity_scale_ = scale;
}

void DvlVisual::setShowBeams(bool show)
{
  show_beams_ = show;
}

void DvlVisual::setBeamColor(const Ogre::ColourValue & color)
{
  beam_color_ = color;
}

void DvlVisual::setShowCovariance(bool show)
{
  show_covariance_ = show;
}

void DvlVisual::setCovarianceColor(const Ogre::ColourValue & color)
{
  covariance_color_ = color;
}

void DvlVisual::setCovarianceScale(float scale)
{
  covariance_scale_ = scale;
}

void DvlVisual::setMessage(const marine_acoustic_msgs::msg::Dvl::ConstSharedPtr & msg)
{
  // --- Velocity arrow ---
  {
    const double vx = msg->velocity.x;
    const double vy = msg->velocity.y;
    const double vz = msg->velocity.z;
    const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);

    if (speed > 1e-9) {
      const float shaft_len = static_cast<float>(speed * velocity_scale_) * 0.8f;
      const float head_len  = static_cast<float>(speed * velocity_scale_) * 0.2f;
      velocity_arrow_->set(shaft_len, 0.05f, head_len, 0.1f);
      velocity_arrow_->setDirection(
        Ogre::Vector3(
          static_cast<float>(vx / speed),
          static_cast<float>(vy / speed),
          static_cast<float>(vz / speed)));
      velocity_arrow_->setColor(velocity_color_);
      velocity_arrow_->getSceneNode()->setVisible(true);
    } else {
      velocity_arrow_->getSceneNode()->setVisible(false);
    }
  }

  // --- Acoustic beam lines ---
  for (int i = 0; i < 4; i++) {
    beam_lines_[i]->clear();

    if (show_beams_ && msg->beam_ranges_valid) {
      const Ogre::Vector3 endpoint(
        static_cast<float>(msg->beam_unit_vec[i].x * msg->range[i]),
        static_cast<float>(msg->beam_unit_vec[i].y * msg->range[i]),
        static_cast<float>(msg->beam_unit_vec[i].z * msg->range[i]));

      beam_lines_[i]->setColor(
        beam_color_.r, beam_color_.g, beam_color_.b, beam_color_.a);
      beam_lines_[i]->addPoint(Ogre::Vector3::ZERO);
      beam_lines_[i]->addPoint(endpoint);
      beam_lines_[i]->finishLine();
    }
  }

  // --- Covariance ellipsoid ---
  if (!show_covariance_ || msg->velocity_covar[0] < 0.0) {
    covariance_sphere_->getRootNode()->setVisible(false);
    return;
  }

  Eigen::Matrix3d cov;
  cov << msg->velocity_covar[0], msg->velocity_covar[1], msg->velocity_covar[2],
         msg->velocity_covar[3], msg->velocity_covar[4], msg->velocity_covar[5],
         msg->velocity_covar[6], msg->velocity_covar[7], msg->velocity_covar[8];

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
  if (solver.info() != Eigen::Success) {
    covariance_sphere_->getRootNode()->setVisible(false);
    return;
  }

  const Eigen::Vector3d & evals = solver.eigenvalues();
  const Eigen::Matrix3d & evecs = solver.eigenvectors();

  const float sx = static_cast<float>(std::sqrt(std::max(0.0, evals(0)))) * covariance_scale_;
  const float sy = static_cast<float>(std::sqrt(std::max(0.0, evals(1)))) * covariance_scale_;
  const float sz = static_cast<float>(std::sqrt(std::max(0.0, evals(2)))) * covariance_scale_;

  // Convert eigenvector matrix (columns = principal axes) to Ogre quaternion.
  Ogre::Matrix3 ogre_rot;
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      ogre_rot[row][col] = static_cast<float>(evecs(row, col));
    }
  }

  covariance_sphere_->setColor(covariance_color_);
  covariance_sphere_->setScale(Ogre::Vector3(sx, sy, sz));
  covariance_sphere_->setOrientation(Ogre::Quaternion(ogre_rot));
  covariance_sphere_->getRootNode()->setVisible(true);
}

}  // namespace dvl_rviz_plugins
