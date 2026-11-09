#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "image_undistort/image_concat.h"

namespace image_undistort {

class ImageConcatNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      concat_undistort_ = std::make_shared<ImageConcat>(
          getNodeHandle(), getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<ImageConcat> concat_undistort_;
};
}  // namespace image_undistort

PLUGINLIB_EXPORT_CLASS(image_undistort::ImageConcatNodelet,
                       nodelet::Nodelet);
