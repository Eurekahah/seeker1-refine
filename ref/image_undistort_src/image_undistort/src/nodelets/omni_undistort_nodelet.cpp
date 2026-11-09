#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "image_undistort/omni_undistort.h"

namespace image_undistort {

class OmniUndistortNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      omni_undistort_ = std::make_shared<OmniUndistort>(
          getNodeHandle(), getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<OmniUndistort> omni_undistort_;
};
}  // namespace image_undistort

PLUGINLIB_EXPORT_CLASS(image_undistort::OmniUndistortNodelet,
                       nodelet::Nodelet);
