#ifndef IMAGE_UNDISTORT_IMAGE_CONCAT_H
#define IMAGE_UNDISTORT_IMAGE_CONCAT_H

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <ros/callback_queue.h>
#include <mutex>
#include <algorithm>
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <cv_bridge/cv_bridge.h>

#include "image_undistort/camera_parameters.h"
#include "image_undistort/undistorter.h"

namespace image_undistort {

typedef struct {
  int concat_method;

  int concat_width;
  int concat_height;

  double blend_weight;

  double pano_radius;
  double pano_vfov_deg;
  
  double bev_ground_height;
  double bev_fx;
} ConcatParam_t;

class ConcatCameraParameters {
 public:
  ConcatCameraParameters(const int camera_num = 4);
  bool setInputCameraParameters(const ros::NodeHandle& nh,
                                const std::string& camera_namespace,
                                const int camera_index, const bool invert_T);
  bool valid() const;
  const std::shared_ptr<InputCameraParameters>& getInputPtr(const int camera_index) const;
  int getCameraNum() const;  // 新增：获取摄像头数量

 private:
  std::vector<std::shared_ptr<InputCameraParameters>> input_ptr_;
  int camera_num_;  // 新增：摄像头数量
  double empty_pixels_;
};

class ConcatUndistorter {
 public:
  ConcatUndistorter(const ConcatCameraParameters& input_camera_parameters_pair, ConcatParam_t concat_param);

  const ConcatCameraParameters& getCameraParametersPair();

  void calcMap_AVM(cv::Size resolution_out);
  void calcMap(cv::Size resolution_out);
  void calcPanoramicMap(cv::Size resolution_out);
  void calcPanoramic2Map(cv::Size resolution_out);
  void calcEquirectMap(cv::Size resolution_out);
  
  // 修改：添加对6个摄像头图像的支持
  void undistortImage(int method,
                      const cv::Mat& image_0,
                      const cv::Mat& image_1,
                      const cv::Mat& image_2,
                      const cv::Mat& image_3,
                      const cv::Mat& image_4 = cv::Mat(),  // 第5个摄像头图像（可选）
                      const cv::Mat& image_5 = cv::Mat(),  // 第6个摄像头图像（可选）
                      cv::Mat* undistored_image = nullptr);

 private:
  const ConcatCameraParameters used_concat_camera_parameters_;
  std::vector<std::pair<cv::Mat, cv::Mat>> map_;
  std::vector<cv::Mat> fusion_masks_;
  cv::Mat scratch_dst_;
  cv::Mat scratch_add_;
  std::vector<cv::Mat> cam_dst_;
  std::vector<cv::Mat> cam_weighted_;
  cv::Mat accum16_;
  int camera_num_;  // 新增：摄像头数量

  ConcatParam_t concat_param_;
};

// Default values
constexpr int kQueueSize = 10;
constexpr bool kDefaultInputCameraInfoFromROSParams = true;
const std::string kDefaultFirstCameraNamespace = "first_camera";
const std::string kDefaultSecondCameraNamespace = "second_camera";
const std::string kDefaultThirdCameraNamespace = "third_camera";
const std::string kDefaultFourthCameraNamespace = "fourth_camera";
const std::string kDefaultFifthCameraNamespace = "fifth_camera";  // 新增
const std::string kDefaultSixthCameraNamespace = "sixth_camera";  // 新增
constexpr int kDefaultProcessEveryNthFrame = 1;
const std::string kDefaultOutputImageType = "";
constexpr bool kDefaultInvertT = false;
constexpr int kDefaultConcatWidth = 320 * 4;
constexpr int kDefaultConcatHeight = 640;
constexpr double kDefaultBEVGroundHeight = -0.1;
constexpr double kDefaultBlendWeight = 0.5;
constexpr double kDefaultBevFX = 100;
constexpr int kDefaultConcatMethod = 0;
constexpr double kDefaultPanoRadius = 10.0;
constexpr double kDefaultPanoVFovDeg = 90.0;
constexpr int kDefaultUseImageTransport = 1;

class ImageConcat {
 public:
  ImageConcat(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  // 4个摄像头的回调函数
  void imagesCallback4(const sensor_msgs::ImageConstPtr& first_image_msg_in,
                       const sensor_msgs::ImageConstPtr& second_image_msg_in,
                       const sensor_msgs::ImageConstPtr& third_image_msg_in,
                       const sensor_msgs::ImageConstPtr& fourth_image_msg_in);

  // 5个摄像头的回调函数
  void imagesCallback5(const sensor_msgs::ImageConstPtr& first_image_msg_in,
                       const sensor_msgs::ImageConstPtr& second_image_msg_in,
                       const sensor_msgs::ImageConstPtr& third_image_msg_in,
                       const sensor_msgs::ImageConstPtr& fourth_image_msg_in,
                       const sensor_msgs::ImageConstPtr& fifth_image_msg_in);
  
  // 6个摄像头的回调函数
  void imagesCallback6(const sensor_msgs::ImageConstPtr& first_image_msg_in,
                       const sensor_msgs::ImageConstPtr& second_image_msg_in,
                       const sensor_msgs::ImageConstPtr& third_image_msg_in,
                       const sensor_msgs::ImageConstPtr& fourth_image_msg_in,
                       const sensor_msgs::ImageConstPtr& fifth_image_msg_in,
                       const sensor_msgs::ImageConstPtr& sixth_image_msg_in);

 private:
  int getQueueSize() const;
  void updateUndistorter();

  // 处理4个图像并发送
  void processAndSendImage4(
    const sensor_msgs::ImageConstPtr& first_image_msg_in,
    const sensor_msgs::ImageConstPtr& second_image_msg_in,
    const sensor_msgs::ImageConstPtr& third_image_msg_in,
    const sensor_msgs::ImageConstPtr& fourth_image_msg_in);
  
  // 处理5个图像并发送
  void processAndSendImage5(
    const sensor_msgs::ImageConstPtr& first_image_msg_in,
    const sensor_msgs::ImageConstPtr& second_image_msg_in,
    const sensor_msgs::ImageConstPtr& third_image_msg_in,
    const sensor_msgs::ImageConstPtr& fourth_image_msg_in,
    const sensor_msgs::ImageConstPtr& fifth_image_msg_in);

  // 处理6个图像并发送
  void processAndSendImage6(
    const sensor_msgs::ImageConstPtr& first_image_msg_in,
    const sensor_msgs::ImageConstPtr& second_image_msg_in,
    const sensor_msgs::ImageConstPtr& third_image_msg_in,
    const sensor_msgs::ImageConstPtr& fourth_image_msg_in,
    const sensor_msgs::ImageConstPtr& fifth_image_msg_in,
    const sensor_msgs::ImageConstPtr& sixth_image_msg_in);

  void latestImageCallback(int index, const sensor_msgs::ImageConstPtr& msg);
  void processLatest(const ros::TimerEvent& event);

  // nodes
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  image_transport::ImageTransport it_;

  // subscribers
  image_transport::SubscriberFilter first_image_sub_;
  image_transport::SubscriberFilter second_image_sub_;
  image_transport::SubscriberFilter third_image_sub_;
  image_transport::SubscriberFilter fourth_image_sub_;
  image_transport::SubscriberFilter fifth_image_sub_;  // 新增
  image_transport::SubscriberFilter sixth_image_sub_;   // 新增

  // publishers
  image_transport::Publisher it_concat_image_pub_;
  ros::Publisher concat_image_pub_;

  // 4个图像的同步策略
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                                                    sensor_msgs::Image,
                                                    sensor_msgs::Image,
                                                    sensor_msgs::Image>
      ExactImageSyncPolicy4;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::Image>
      ImageSyncPolicy4;
  std::shared_ptr<message_filters::Synchronizer<ExactImageSyncPolicy4>>
      exact_image_sync_ptr_4_;
  std::shared_ptr<message_filters::Synchronizer<ImageSyncPolicy4>>
      image_sync_ptr_4_;


  // 5个图像的同步策略
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                                                    sensor_msgs::Image,
                                                    sensor_msgs::Image,
                                                    sensor_msgs::Image,
                                                    sensor_msgs::Image>
      ExactImageSyncPolicy5;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::Image>
      ImageSyncPolicy5;
  std::shared_ptr<message_filters::Synchronizer<ExactImageSyncPolicy5>>
      exact_image_sync_ptr_5_;
  std::shared_ptr<message_filters::Synchronizer<ImageSyncPolicy5>>
      image_sync_ptr_5_;

  // 6个图像的同步策略
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                                                    sensor_msgs::Image,
                                                    sensor_msgs::Image,
                                                    sensor_msgs::Image,
                                                    sensor_msgs::Image,
                                                    sensor_msgs::Image>
      ExactImageSyncPolicy6;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::Image,
                                                          sensor_msgs::Image>
      ImageSyncPolicy6;
  std::shared_ptr<message_filters::Synchronizer<ExactImageSyncPolicy6>>
      exact_image_sync_ptr_6_;
  std::shared_ptr<message_filters::Synchronizer<ImageSyncPolicy6>>
      image_sync_ptr_6_;

  // camera parameters
  std::shared_ptr<ConcatCameraParameters> camera_parameters_ptr_;

  // undistorters
  std::shared_ptr<ConcatUndistorter> undistorter_ptr_;
  cv::Mat output_image_;
  std::vector<sensor_msgs::ImagePtr> output_msgs_;
  std::size_t output_msg_index_ = 0;

  std::mutex latest_mutex_;
  sensor_msgs::ImageConstPtr latest_images_[4];
  std::mutex process_mutex_;
  ros::Timer process_timer_;
  int target_fps_ = 10;
  double max_frame_age_s_ = 0.35;
  double max_frame_skew_s_ = 0.20;

  // other variables
  bool input_camera_info_from_ros_params_;
  int queue_size_;
  int process_every_nth_frame_;
  std::string output_image_type_;
  int frame_counter_;
  int use_image_transport_;
  int camera_num_;  // 新增：摄像头数量

  ConcatParam_t concat_param_;
};
}

#endif
