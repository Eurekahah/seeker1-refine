#include "image_undistort/omni_undistort.h"
#include "image_undistort/camera_parameters.h"

//   camm0         camm1         camm2          camm3
// cam7 cam0     cam1 cam2     cam3 cam4      cam5 cam6
// s3r  s0l      s0r   s1l     s1r  s2l       s2r  s3l
// stereo index + output side

// static std::map<std::string, std::vector<std::tuple<int, CameraSide, std::string>>> OmniCamToIndex = {
//         {"camm0", {{0, CameraSide::FIRST, "cam0"}, {3, CameraSide::SECOND, "cam7"}}},
//         {"camm1", {{1, CameraSide::FIRST, "cam2"}, {0, CameraSide::SECOND, "cam1"}}},
//         {"camm2", {{2, CameraSide::FIRST, "cam4"}, {1, CameraSide::SECOND, "cam3"}}},
//         {"camm3", {{3, CameraSide::FIRST, "cam6"}, {2, CameraSide::SECOND, "cam5"}}},
//     };

static std::vector<std::vector<std::pair<int, int>>> OmniCamToIndex = {
    {{0, 0}, {1, 1}},
    {{1, 2}, {2, 3}},
    {{2, 4}, {3, 5}},
    {{3, 6}, {0, 7}},
    // input cam index,  stereo index, output cam index
    // OmniCamToIndex[]
};
// static std::vector<std::vector<std::pair<int, int>>> OmniCamToIndex = {
//     {{0, 0}, {3, 7}},
//     {{1, 2}, {0, 1}},
//     {{2, 4}, {1, 3}},
//     {{3, 6}, {2, 5}},
//     // input cam index,  stereo index, output cam index
//     // OmniCamToIndex[]
// };
static std::map<std::string, std::vector<std::pair<int, int>>> OmniStrCamToIndex = {
    {{"cam0"}, {{0, 0}, {3, 7}}},
    {{"cam1"}, {{1, 2}, {0, 1}}},
    {{"cam2"}, {{2, 4}, {1, 3}}},
    {{"cam3"}, {{3, 6}, {2, 5}}},
    // input cam index,  stereo index, output cam index
};
// (stereo index, output cam index) = OmniCamToIndex[input cam index, cam_side];

static std::vector<std::pair<std::string, std::string>> stereo_input_cam_map = {
    {"cam0", "cam1"},
    {"cam1", "cam2"},
    {"cam2", "cam3"},
    {"cam3", "cam0"},
};

// // cam in cam out
// std::map<std::pair<int, CameraSide>, std::pair<int, int>> {
//   {{0, CameraSide::FIRST}, {0, 0}},
//   {{0, CameraSide::SECOND}, {1, 1}},
//   {}
// };

namespace image_undistort {

// needed so that topic filters can be initalized in the constructor
int OmniUndistort::getQueueSize() const {
  int queue_size;
  nh_private_.param("queue_size", queue_size, kQueueSize);
  if (queue_size < 1) {
    ROS_ERROR("Queue size must be >= 1, setting to 1");
    queue_size = 1;
  }
  return queue_size;
}

OmniUndistort::OmniUndistort(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      it_(nh_),
      queue_size_(getQueueSize()),
      camx_undistorter_ptr_(8),
      // second_undistorter_ptr_(nullptr),
      frame_counter_(0) {
  // set parameters from ros
  nh_private_.param("input_camera_info_from_ros_params",
                    input_camera_info_from_ros_params_,
                    kDefaultInputCameraInfoFromROSParams);

  nh_private_.param("rename_radtan_plumb_bob", rename_radtan_plumb_bob_,
                    kDefaultRenameRadtanPlumbBob);

  bool invert_T;
  nh_private_.param("invert_T", invert_T, kDefaultInvertT);

  nh_private_.param("queue_size", queue_size_, kQueueSize);
  if (queue_size_ < 1) {
    ROS_ERROR("Queue size must be >= 1, setting to 1");
    queue_size_ = 1;
  }

  double scale;
  nh_private_.param("scale", scale, kDefaultScale);

  omni_camera_parameters_ptr_.resize(4);
  for (int i = 0; i < CONFIG_MAX_INPUT_CAM_NUM; i++) {
    omni_camera_parameters_ptr_[i] = std::make_shared<StereoCameraParameters>(scale);
  }
  // stereo_camera_parameters_ptr_ =
  //     std::make_shared<StereoCameraParameters>(scale);

  // nh_private_.param("process_every_nth_frame", process_every_nth_frame_,
  //                   kDefaultProcessEveryNthFrame);
  nh_private_.param("output_image_type", output_image_type_,
                    kDefaultOutputImageType);
  // check output type string is correctly formatted
  if (!output_image_type_.empty()) {
    try {
      cv_bridge::getCvType(output_image_type_);
    } catch (const cv_bridge::Exception& e) {
      ROS_ERROR_STREAM(
          "cv_bridge error while setting output_image_type, output will match "
          "input type. "
          << e.what());
      output_image_type_ = "";
    }
  }

  nh_private_.param("publish_tf", publish_tf_, kDefaultPublishTF);
  nh_private_.param("rename_input_frame", rename_input_frame_,
                    kDefaultRenameInputFrame);

  // setup publishers
  camera_info_output_pub_.resize(CONFIG_MAX_OUTPUT_CAM_NUM);
  for (int i = 0; i < CONFIG_STEREO_NUM; i++) {
    int cam_index = OmniCamToIndex[i][0].second;
    std::string camera_info_topic1 = "rect/image"+std::to_string(cam_index)+"/camera_info";
    camera_info_output_pub_[cam_index] = nh_.advertise<sensor_msgs::CameraInfo>(
      camera_info_topic1, queue_size_);
    cam_index = OmniCamToIndex[i][1].second;
    std::string camera_info_topic2 = "rect/image"+std::to_string(cam_index)+"/camera_info";
    camera_info_output_pub_[cam_index] = nh_.advertise<sensor_msgs::CameraInfo>(
      camera_info_topic2, queue_size_);
  }

  // first_camera_info_output_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
  //     "rect/first/camera_info", queue_size_);
  // second_camera_info_output_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
  //     "rect/second/camera_info", queue_size_);

  image_pub_.resize(CONFIG_MAX_OUTPUT_CAM_NUM);
  for (int stereo = 0; stereo < CONFIG_STEREO_NUM; stereo++) {
    {
      int cam_index = OmniCamToIndex[stereo][0].second;
      std::string topic = "rect/image" + std::to_string(cam_index);
      image_pub_[cam_index] = it_.advertise(topic, queue_size_);
    }{
      int cam_index = OmniCamToIndex[stereo][1].second;
      std::string topic = "rect/image" + std::to_string(cam_index);
      image_pub_[cam_index] = it_.advertise(topic, queue_size_);
    }
  }

  // setup subscribers (must be done last as it appears image filters allow a
  // subscriber to be called as soon as it is created, even if this constructor
  // is not finished)
  if (input_camera_info_from_ros_params_) {
    for (int stereo = 0; stereo < CONFIG_MAX_INPUT_CAM_NUM; stereo++) {
      if (!omni_camera_parameters_ptr_[stereo]->setInputCameraParameters(
              nh_private_, stereo_input_cam_map[stereo].first, CameraSide::FIRST, invert_T) ||
          !omni_camera_parameters_ptr_[stereo]->setInputCameraParameters(
              nh_private_, stereo_input_cam_map[stereo].second, CameraSide::SECOND, invert_T)) {
        ROS_FATAL("Loading of input camera parameters failed, exiting");
        ros::shutdown();
        exit(EXIT_FAILURE);
      }
    }

    // first_camera_info_input_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
    //     "raw/first/camera_info", queue_size_);
    // second_camera_info_input_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
    //     "raw/second/camera_info", queue_size_);
    updateUndistorter(0, CameraSide::FIRST);
    updateUndistorter(0, CameraSide::SECOND);
    updateUndistorter(1, CameraSide::FIRST);
    updateUndistorter(1, CameraSide::SECOND);
    updateUndistorter(2, CameraSide::FIRST);
    updateUndistorter(2, CameraSide::SECOND);
    updateUndistorter(3, CameraSide::FIRST);
    updateUndistorter(3, CameraSide::SECOND);

    image_sub_.resize(CONFIG_MAX_INPUT_CAM_NUM);
    for (int i = 0; i < CONFIG_MAX_INPUT_CAM_NUM; i++) {
      image_sub_[i] = it_.subscribe("raw/image"+std::to_string(i), queue_size_,
                                &OmniUndistort::imagesCallback, this);
    }

    // image_sub_.resize(CONFIG_MAX_INPUT_CAM_NUM*2);
    // for (int i = 0; i < CONFIG_MAX_INPUT_CAM_NUM; i++) {
    //   image_sub_[i*2] = it_.subscribe("raw/image"+std::to_string(i), queue_size_,
    //                             boost::bind(&OmniUndistort::imagesCallback, this, _1, 0));
    //   image_sub_[i*2+1] = it_.subscribe("raw/image"+std::to_string(i), queue_size_,
    //                             boost::bind(&OmniUndistort::imagesCallback, this, _1, 1));
    // }
  } else {
  }
}

void OmniUndistort::updateUndistorter(const int stereo_index, const CameraSide& side) {
  std::shared_ptr<Undistorter>* undistorter_ptr_ptr;
  CameraParametersPair camera_parameters_pair;

  if (side == CameraSide::FIRST) {
    undistorter_ptr_ptr = &camx_undistorter_ptr_[OmniCamToIndex[stereo_index][0].second];
    camera_parameters_pair = omni_camera_parameters_ptr_[stereo_index]->getFirst();
  } else {
    undistorter_ptr_ptr = &camx_undistorter_ptr_[OmniCamToIndex[stereo_index][1].second];
    camera_parameters_pair = omni_camera_parameters_ptr_[stereo_index]->getSecond();
  }

  // if undistorter not built or built using old data update it
  if (!(*undistorter_ptr_ptr) ||
      ((*undistorter_ptr_ptr)->getCameraParametersPair() !=
       camera_parameters_pair)) {
    try {
      *undistorter_ptr_ptr =
          std::make_shared<Undistorter>(camera_parameters_pair);
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
      return;
    }
  }
}

void OmniUndistort::sendCameraInfo(const std_msgs::Header& header,
                                     const int stereo,
                                     const CameraSide& side,
                                     const CameraIO& io) {
  sensor_msgs::CameraInfo camera_info;
  camera_info.header = header;

  try {
    omni_camera_parameters_ptr_[stereo]->generateCameraInfoMessage(side, io,
                                                             &camera_info);
  } catch (std::runtime_error e) {
    ROS_ERROR("%s", e.what());
    return;
  }

  if (rename_radtan_plumb_bob_ && camera_info.distortion_model == "radtan") {
    camera_info.distortion_model = "plumb_bob";
  }

  if (side == CameraSide::FIRST) {
    if (io == CameraIO::INPUT) {
      // first_camera_info_input_pub_.publish(camera_info);
    } else {
      camera_info_output_pub_[OmniCamToIndex[stereo][0].second].publish(camera_info);
    }
  } else {
    if (io == CameraIO::INPUT) {
      // second_camera_info_input_pub_.publish(camera_info);
    } else {
      camera_info_output_pub_[OmniCamToIndex[stereo][1].second].publish(camera_info);
    }
  }
}

void OmniUndistort::processAndSendImage( 
    const sensor_msgs::ImageConstPtr& image_msg_in) {
  cv_bridge::CvImageConstPtr image_in_ptr =
      cv_bridge::toCvShare(image_msg_in, output_image_type_);

  std::string encoding = image_in_ptr->encoding;
  if (encoding == "8UC1") {
    // ros does not recognize U8C1 and it will crash the disparity node
    encoding = "mono8";
  }

  cv::UMat uimage = image_in_ptr->image.getUMat(cv::ACCESS_READ);  // 将输入的 cv::Mat 转换为 cv::UMat
  // side first
  {
    cv_bridge::CvImagePtr image_out_ptr1(
        new cv_bridge::CvImage(image_in_ptr->header, encoding));
    const int stereo1 = OmniStrCamToIndex[image_in_ptr->header.frame_id][0].first;
    const int out_index1 = OmniStrCamToIndex[image_in_ptr->header.frame_id][0].second;
    std::string output_frame_1 = "cam" + std::to_string(out_index1);
    const CameraSide side1 = CameraSide::FIRST;
    // updateUndistorter(stereo1, side1);
    image_out_ptr1->header.frame_id = output_frame_1;

    cv_bridge::CvImagePtr image_out_ptr2(
        new cv_bridge::CvImage(image_in_ptr->header, encoding));
    const int stereo2 = OmniStrCamToIndex[image_in_ptr->header.frame_id][1].first; 
    const int out_index2 = OmniStrCamToIndex[image_in_ptr->header.frame_id][1].second;
    std::string output_frame_2 = "cam" + std::to_string(out_index2);
    CameraSide side2 = CameraSide::SECOND;
    // updateUndistorter(stereo2, side2);
    image_out_ptr2->header.frame_id = output_frame_2;
    // ros::Time t1 = ros::Time::now();
    camx_undistorter_ptr_[out_index1]->undistortImage(uimage, &(image_out_ptr1->image));
    // ros::Time t2 = ros::Time::now();
    camx_undistorter_ptr_[out_index2]->undistortImage(uimage, &(image_out_ptr2->image));
    // ros::Time t3 = ros::Time::now();
    // printf("time1=%f time2=%f\n", (t3-t2).toSec(), (t2-t1).toSec() );

    image_pub_[out_index1].publish(*(image_out_ptr1->toImageMsg()));
    image_pub_[out_index2].publish(*(image_out_ptr2->toImageMsg()));
  }
}

void OmniUndistort::imagesCallback(const sensor_msgs::ImageConstPtr& first_image_msg_in) {
  if (!omni_camera_parameters_ptr_[0]->valid() || !omni_camera_parameters_ptr_[1]->valid() || 
       !omni_camera_parameters_ptr_[2]->valid() ||!omni_camera_parameters_ptr_[3]->valid()) {
    ROS_ERROR("Camera parameters invalid, undistortion failed");
    return;
  }

  // if (++frame_counter_ < process_every_nth_frame_) {
  //   return;
  // }
  // frame_counter_ = 0;

  processAndSendImage(first_image_msg_in);
  // processAndSendImage(second_image_msg_in, CameraSide::SECOND);

  // if (input_camera_info_from_ros_params_) {
  //   std_msgs::Header header = first_image_msg_in->header;
  //   if (rename_input_frame_) {
  //     header.frame_id = first_input_frame_;
  //   }
  //   sendCameraInfo(header, CameraSide::FIRST, CameraIO::INPUT);

  //   header = second_image_msg_in->header;
  //   if (rename_input_frame_) {
  //     header.frame_id = second_input_frame_;
  //   }
  //   sendCameraInfo(header, CameraSide::SECOND, CameraIO::INPUT);
  // }
  std_msgs::Header header = first_image_msg_in->header;
  header.frame_id = first_output_frame_;
  const int stereol = OmniStrCamToIndex[first_image_msg_in->header.frame_id][0].first;
  sendCameraInfo(header, stereol, CameraSide::FIRST, CameraIO::OUTPUT);

  header = first_image_msg_in->header;
  header.frame_id = second_output_frame_;
  const int stereor = OmniStrCamToIndex[first_image_msg_in->header.frame_id][1].first;
  sendCameraInfo(header, stereor, CameraSide::SECOND, CameraIO::OUTPUT);
}

// void OmniUndistort::camerasCallback(
//     const sensor_msgs::ImageConstPtr& first_image_msg_in,
//     const sensor_msgs::ImageConstPtr& second_image_msg_in,
//     const sensor_msgs::CameraInfoConstPtr& first_camera_info_msg_in,
//     const sensor_msgs::CameraInfoConstPtr& second_camera_info_msg_in) {
//   if (!stereo_camera_parameters_ptr_->setInputCameraParameters(
//           *first_camera_info_msg_in, CameraSide::FIRST) ||
//       !stereo_camera_parameters_ptr_->setInputCameraParameters(
//           *second_camera_info_msg_in, CameraSide::SECOND)) {
//     ROS_ERROR("Setting camera info failed, dropping frame");
//     return;
//   }

//   imagesCallback(first_image_msg_in, second_image_msg_in);
// }
}  // namespace image_undistort
