#ifndef DVL_RVIZ_PLUGINS__DVL_DISPLAY_HPP_
#define DVL_RVIZ_PLUGINS__DVL_DISPLAY_HPP_

#include <memory>

#include <marine_acoustic_msgs/msg/dvl.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>

namespace dvl_rviz_plugins
{
class DvlVisual;

/// RViz2 Display for marine_acoustic_msgs/msg/Dvl.
///
/// Renders a velocity arrow, four acoustic beam lines, and a covariance
/// ellipsoid.  All visuals are placed in the sensor frame via tf2.
class DvlDisplay
  : public rviz_common::MessageFilterDisplay<marine_acoustic_msgs::msg::Dvl>
{
  Q_OBJECT

public:
  DvlDisplay();
  ~DvlDisplay() override = default;

  void onInitialize() override;
  void reset() override;

protected:
  void processMessage(
    marine_acoustic_msgs::msg::Dvl::ConstSharedPtr msg) override;

private:
  std::shared_ptr<DvlVisual> visual_;

  rviz_common::properties::ColorProperty * velocity_color_prop_;
  rviz_common::properties::FloatProperty * velocity_scale_prop_;
  rviz_common::properties::BoolProperty * show_beams_prop_;
  rviz_common::properties::ColorProperty * beam_color_prop_;
  rviz_common::properties::BoolProperty * show_covariance_prop_;
  rviz_common::properties::ColorProperty * covariance_color_prop_;
  rviz_common::properties::FloatProperty * covariance_scale_prop_;
};

}  // namespace dvl_rviz_plugins

#endif  // DVL_RVIZ_PLUGINS__DVL_DISPLAY_HPP_
