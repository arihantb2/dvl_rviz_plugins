#ifndef DVL_RVIZ_PLUGINS__DVL_VISUAL_HPP_
#define DVL_RVIZ_PLUGINS__DVL_VISUAL_HPP_

#include <array>
#include <memory>

#include <OgreColourValue.h>
#include <OgreQuaternion.h>
#include <OgreVector.h>

#include <marine_acoustic_msgs/msg/dvl.hpp>

namespace Ogre
{
class SceneManager;
class SceneNode;
}

namespace rviz_rendering
{
class Arrow;
class BillboardLine;
class Shape;
}

namespace dvl_rviz_plugins
{

/// Manages the Ogre3D scene objects that render one DVL message.
class DvlVisual
{
public:
  DvlVisual(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node);
  ~DvlVisual();

  /// Update all visual elements from a new DVL message.
  void setMessage(const marine_acoustic_msgs::msg::Dvl::ConstSharedPtr & msg);

  /// Translate the visual to match the sensor frame position in the fixed frame.
  void setFramePosition(const Ogre::Vector3 & position);

  /// Rotate the visual to match the sensor frame orientation in the fixed frame.
  void setFrameOrientation(const Ogre::Quaternion & orientation);

  // --- Property setters called from DvlDisplay before setMessage() ---
  void setVelocityColor(const Ogre::ColourValue & color);
  void setVelocityScale(float scale);
  void setShowBeams(bool show);
  void setBeamColor(const Ogre::ColourValue & color);
  void setShowCovariance(bool show);
  void setCovarianceColor(const Ogre::ColourValue & color);
  void setCovarianceScale(float scale);

private:
  Ogre::SceneNode * frame_node_;
  Ogre::SceneManager * scene_manager_;

  std::unique_ptr<rviz_rendering::Arrow> velocity_arrow_;
  std::array<std::unique_ptr<rviz_rendering::BillboardLine>, 4> beam_lines_;
  std::unique_ptr<rviz_rendering::Shape> covariance_sphere_;

  // Cached property values
  Ogre::ColourValue velocity_color_;
  float velocity_scale_{1.0f};
  bool show_beams_{true};
  Ogre::ColourValue beam_color_;
  bool show_covariance_{true};
  Ogre::ColourValue covariance_color_;
  float covariance_scale_{1.0f};
};

}  // namespace dvl_rviz_plugins

#endif  // DVL_RVIZ_PLUGINS__DVL_VISUAL_HPP_
