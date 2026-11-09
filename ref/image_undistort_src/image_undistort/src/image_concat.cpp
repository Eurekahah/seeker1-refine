#include "image_undistort/image_concat.h"
#include "image_undistort/camera_parameters.h"
#include <opencv2/ccalib/omnidir.hpp>
#include <opencv2/core/eigen.hpp> 
#include <opencv2/opencv.hpp>
#include <cstring>
#include <malloc.h>
#include <unistd.h>
#include <cstdio>
#include <cmath>

#define TP printf("%s %d\n", __func__,__LINE__);
namespace image_undistort {

static const volatile char v__[] = "深圳市视元智能科技有限公司-蔡则鹏-1103771026@qq.com";
static const volatile char v___[] = "ShenZhenShiShiYuanZhiNengKeJiYouXianGongSi";
static const volatile char v____[] = "CAIZEPENG";

ConcatCameraParameters::ConcatCameraParameters(const int camera_num) {
  camera_num_ = camera_num;
  input_ptr_.resize(camera_num);
}

bool ConcatCameraParameters::setInputCameraParameters(const ros::NodeHandle& nh,
                                const std::string& camera_namespace,
                                const int camera_index, const bool invert_T) {
  try {
    if (camera_index >= input_ptr_.size()) {
      ROS_ERROR("Camera index %d out of bounds (max %d)", camera_index, (int)input_ptr_.size());
      return false;
    }
    input_ptr_[camera_index] = std::make_shared<InputCameraParameters>(nh, camera_namespace,
                                                           invert_T);
    return true;
  } catch (std::runtime_error e) {
    ROS_ERROR("%s", e.what());
    return false;
  }
}

const std::shared_ptr<InputCameraParameters>& 
ConcatCameraParameters::getInputPtr(const int camera_index) const {
  return input_ptr_[camera_index];
}

int ConcatCameraParameters::getCameraNum() const {
  return camera_num_;
}

bool ConcatCameraParameters::valid() const {
  if (input_ptr_.size() != camera_num_) {
    return false;  
  }
  for (int i = 0; i < camera_num_; i++) {
    if (input_ptr_[i] == nullptr)
      return false;
  }
  return true;
};

ConcatUndistorter::ConcatUndistorter(
    const ConcatCameraParameters& input_camera_parameters_pair, ConcatParam_t concat_param) 
    : used_concat_camera_parameters_(input_camera_parameters_pair), concat_param_(concat_param) {
  if (!used_concat_camera_parameters_.valid()) {
    throw std::runtime_error(
        "Attempted to create undistorter from invalid camera parameters");
  }
  
  camera_num_ = used_concat_camera_parameters_.getCameraNum();
  cv::Size resolution_in =
      used_concat_camera_parameters_.getInputPtr(0)->resolution();
  cv::Size resolution_out(concat_param_.concat_width, concat_param_.concat_height);
  
  if (concat_param_.concat_method == 0) {
    calcMap(resolution_out);
  } else if (concat_param_.concat_method == 1) {
    calcMap_AVM(resolution_out);
  } else if (concat_param_.concat_method == 2) {
    calcPanoramicMap(resolution_out);
  } else if (concat_param_.concat_method == 3) {
    calcPanoramic2Map(resolution_out);
  } else if (concat_param_.concat_method == 4) {
    calcEquirectMap(resolution_out);
  }
}

struct CameraParams {
  cv::Matx33d K;      // 内参矩阵 (3x3 CV_64F)
  cv::Vec4d D;      // 畸变参数 (4x1 CV_64F)
  cv::Matx33d R;      // 全局旋转矩阵 (3x3 CV_64F)
  cv::Vec3d t;
  cv::Matx44d T;      // 全局旋转矩阵 (3x3 CV_64F)
  double xi;
  cv::Size image_size;// 图像尺寸
};

std::vector<CameraParams> get_camera_param(const ConcatCameraParameters &used_concat_camera_parameters_) {
  int camera_num = used_concat_camera_parameters_.getCameraNum();
  std::vector<cv::Matx33d> T_cam_imu_vector;
  std::vector<cv::Matx44d> T_cam_imu_vector2;
  Eigen::Matrix<double, 3, 3> T_cam_imu_0 = used_concat_camera_parameters_.getInputPtr(0)->T_cam_imu().topLeftCorner<3, 3>();
  
  std::vector<struct CameraParams> cameras;
  cameras.resize(camera_num);
  T_cam_imu_vector.resize(camera_num);
  T_cam_imu_vector2.resize(camera_num);
  
  // 计算每个相机相对于第一个相机的旋转
  cv::eigen2cv(T_cam_imu_0, T_cam_imu_vector.at(0));
  for (int i = 1; i < camera_num; i++) {
    Eigen::Matrix<double, 3, 3> T_cam_imu = used_concat_camera_parameters_.getInputPtr(i)->R() * T_cam_imu_0;
    T_cam_imu_0 = T_cam_imu;
    cv::eigen2cv(T_cam_imu, T_cam_imu_vector.at(i));
  }
  
  // 计算变换矩阵
  Eigen::Matrix<double, 4, 4> T_cam_imu_0_full = used_concat_camera_parameters_.getInputPtr(0)->T_cam_imu();
  cv::eigen2cv(T_cam_imu_0_full, T_cam_imu_vector2.at(0));
  for (int i = 1; i < camera_num; i++) {
    Eigen::Matrix<double, 4, 4> T_cam_imu = used_concat_camera_parameters_.getInputPtr(i)->T() * T_cam_imu_0_full;
    T_cam_imu_0_full = T_cam_imu;
    cv::eigen2cv(T_cam_imu, T_cam_imu_vector2.at(i));
  }

  for (int camera_index = 0; camera_index < camera_num; camera_index++) {
    cv::Matx33d K;
    cv::eigen2cv(used_concat_camera_parameters_.getInputPtr(camera_index)->K(), K);

    cv::Vec4d D;
    D(0) = used_concat_camera_parameters_.getInputPtr(camera_index)->D().at(1);
    D(1) = used_concat_camera_parameters_.getInputPtr(camera_index)->D().at(2);
    D(2) = used_concat_camera_parameters_.getInputPtr(camera_index)->D().at(3);
    D(3) = used_concat_camera_parameters_.getInputPtr(camera_index)->D().at(4);

    double xi = used_concat_camera_parameters_.getInputPtr(camera_index)->D().at(0);

    cameras[camera_index].K = K;
    cameras[camera_index].D = D;
    cameras[camera_index].R = T_cam_imu_vector.at(camera_index);
    cameras[camera_index].T = T_cam_imu_vector2.at(camera_index);
    cameras[camera_index].xi = xi;
    cameras[camera_index].image_size = used_concat_camera_parameters_.getInputPtr(camera_index)->resolution();
    Eigen::Matrix<double, 3, 1> p_eigen = used_concat_camera_parameters_.getInputPtr(camera_index)->p();
    cv::Vec3d p(p_eigen[0], p_eigen[1], p_eigen[2]);
    cameras[camera_index].t = p;
  }
  return cameras;
}

void ConcatUndistorter::calcMap(cv::Size resolution_out) {
  int camera_num = used_concat_camera_parameters_.getCameraNum();
  map_.resize(camera_num);

  std::vector<struct CameraParams> cameras = get_camera_param(used_concat_camera_parameters_);
  const int pano_width = resolution_out.width;
  const int quarter_width = (int)(resolution_out.width/camera_num);
  const int pano_height = resolution_out.height;

  for (size_t camera_index = 0; camera_index < camera_num; camera_index++) {
    cv::Mat map_x_float(pano_height, quarter_width, CV_32FC1);
    cv::Mat map_y_float(pano_height, quarter_width, CV_32FC1);
    map_x_float.setTo(-1.0f);
    map_y_float.setTo(-1.0f);

    for (int v = 0; v < pano_height; ++v) {
      for (int u = camera_index*quarter_width; u < (camera_index+1)*quarter_width; ++u) {
        // 转换为球面坐标
        double phi = -2 * CV_PI * (u / static_cast<double>(pano_width)) + CV_PI;
        double theta = (v / static_cast<double>(pano_height)) * CV_PI;

        // 转换为3D单位向量 (假设坐标系为x右，y下，z前)
        cv::Point3f dir(
            sin(theta) * cos(phi),
            sin(theta) * sin(phi),
            cos(theta)
        );

        int selected_cam = -1;
        float max_z = -FLT_MAX;
        cv::Point3f dir_cam[camera_num];
        // 转换到每个摄像头坐标系，并选择最合适的摄像头
        for (int cam = 0; cam < camera_num; ++cam) {
            cv::Mat dir_mat = (cv::Mat_<double>(3, 1) << dir.x, dir.y, dir.z);
            cv::Mat rot_dir = cameras[cam].R * dir_mat;
            dir_cam[cam] = cv::Point3f(rot_dir.at<double>(0), rot_dir.at<double>(1), rot_dir.at<double>(2));

            // 检查方向是否朝前
            if (dir_cam[cam].z > 0 && dir_cam[cam].z > max_z) {
                max_z = dir_cam[cam].z;
                selected_cam = cam;
            }
        }
        // 默认选择逻辑：选择相邻的摄像头
        selected_cam = (camera_index + camera_num - 1) % camera_num;

        if (selected_cam == -1) continue;

        // 投影到选定的摄像头
        std::vector<cv::Point3f> object_points = { dir_cam[selected_cam] };
        std::vector<cv::Point2f> image_points;

        cv::omnidir::projectPoints(
            object_points,
            image_points,
            cv::Mat::zeros(3, 1, CV_64F),  // rvec: 无旋转
            cv::Mat::zeros(3, 1, CV_64F),  // tvec: 无平移
            cameras[camera_index].K, cameras[camera_index].xi, cameras[camera_index].D);

        cv::Point2f projected = image_points[0];

        // 检查投影点是否在图像范围内
        if (projected.x >= 0 && projected.x < cameras[camera_index].image_size.width &&
          projected.y >= 0 && projected.y < cameras[camera_index].image_size.height) {
          map_x_float.at<float>(v, u%quarter_width) = projected.x;
          map_y_float.at<float>(v, u%quarter_width) = projected.y;
        }
      }
    }

    cv::convertMaps(map_x_float, map_y_float, map_[camera_index].first, map_[camera_index].second, CV_16SC2);
  }
}
// int saveFusionMasksToPNG(const std::vector<cv::Mat>& fusion_masks, 
//                          const std::string& output_dir = "/tmp",
//                          const std::string& base_name = "fusion_mask") {
//   int saved_count = 0;
  
//   for (size_t i = 0; i < fusion_masks.size(); ++i) {
//     if (fusion_masks[i].empty()) {
//       std::cerr << "Warning: fusion_masks[" << i << "] is empty!" << std::endl;
//       continue;
//     }
    
//     // 构建输出文件名
//     std::string filename = output_dir + "/" + base_name + "_" + std::to_string(i) + ".png";
    
//     // 保存为PNG
//     if (cv::imwrite(filename, fusion_masks[i])) {
//       saved_count++;
//       std::cout << "Saved fusion mask to: " << filename 
//                 << " (size: " << fusion_masks[i].cols << "x" << fusion_masks[i].rows 
//                 << ", type: " << fusion_masks[i].type() << ")" << std::endl;
//     } else {
//       std::cerr << "Error: Failed to save fusion mask to: " << filename << std::endl;
//     }
//   }
  
//   return saved_count;
// }

// 首次调用时计算Mask
void computeFusionMasks(
    const std::vector<cv::Mat>& remap_masks,  // 输入的remap_mask（CV_8UC1）
    std::vector<cv::Mat> &fusion_masks) 
{
  int num_cameras = remap_masks.size();
  std::vector<cv::Mat> inv_mask;
  std::vector<cv::Mat> dist;
  cv::Mat total_weight = cv::Mat::zeros(remap_masks[0].size(), CV_32F);
  
  inv_mask.resize(num_cameras);
  dist.resize(num_cameras);
  fusion_masks.resize(num_cameras);

  for (int cam_idx = 0; cam_idx < num_cameras; ++cam_idx) {
    inv_mask[cam_idx] = remap_masks[cam_idx].clone();
    cv::distanceTransform(inv_mask[cam_idx], dist[cam_idx], cv::DIST_L2, 3);
  }
  for (int cam_idx = 0; cam_idx < num_cameras; ++cam_idx) {
    total_weight += dist[cam_idx];
  }

  cv::Mat total_weight_zero = total_weight.clone();
  total_weight_zero.setTo(1e-6, total_weight == 0); 
  for (int cam_idx = 0; cam_idx < num_cameras; ++cam_idx) {
    cv::Mat w_float;
    cv::Mat a;
    dist[cam_idx].convertTo(w_float, CV_32F);
    cv::divide(w_float, total_weight_zero, a, 1.0, CV_32F);
    a.convertTo(fusion_masks[cam_idx], CV_8UC1, 255);
    cv::Mat mask_color;
    cv::cvtColor(fusion_masks[cam_idx], mask_color, cv::COLOR_GRAY2BGR);
    fusion_masks[cam_idx] = mask_color;
  }

  // saveFusionMasksToPNG(fusion_masks, "/tmp");
}

void ConcatUndistorter::calcMap_AVM(cv::Size resolution_out) {
  int camera_num = used_concat_camera_parameters_.getCameraNum();
  map_.resize(camera_num);
  fusion_masks_.resize(camera_num);

  float meters_per_pixel = cv::abs(concat_param_.bev_ground_height)/concat_param_.bev_fx;
  std::vector<struct CameraParams> cameras = get_camera_param(used_concat_camera_parameters_);
  const int bev_width = resolution_out.width;
  const int bev_height = resolution_out.height;

  // 初始化remap矩阵
  std::vector<cv::Mat> remap_x;
  std::vector<cv::Mat> remap_y;
  std::vector<cv::Mat> remap_mask;
  remap_x.resize(camera_num);
  remap_y.resize(camera_num);
  remap_mask.resize(camera_num);

  for (int cam_idx = 0; cam_idx < camera_num; ++cam_idx) {
    remap_x[cam_idx].create(bev_height, bev_width, CV_32FC1);
    remap_y[cam_idx].create(bev_height, bev_width, CV_32FC1);
    remap_mask[cam_idx].create(bev_height, bev_width, CV_8UC1);
    remap_x[cam_idx].setTo(-1);
    remap_y[cam_idx].setTo(-1);
    remap_mask[cam_idx].setTo(0);
  }
  
  // BEV坐标系参数（假设BEV中心为原点）
  const float start_x = meters_per_pixel * bev_width / 2;
  const float start_y = meters_per_pixel * bev_height / 2;
  
  for (int bev_v = 0; bev_v < bev_height; ++bev_v) {
    for (int bev_u = 0; bev_u < bev_width; ++bev_u) {
      // 转换为世界坐标（地面平面）
      cv::Point3f world_point(
          start_y - bev_v * meters_per_pixel,
          start_x - bev_u * meters_per_pixel,
          concat_param_.bev_ground_height
      );

      // 遍历所有摄像头
      for (int camera_index = 0; camera_index < camera_num; ++camera_index) {
        // 转换到相机坐标系
        cv::Mat world_mat = (cv::Mat_<double>(4,1) << 
          world_point.x, world_point.y, world_point.z, 1);
        
        cv::Mat camera_point = cameras[camera_index].T * world_mat;

        // 检查可见性（Z>0表示在相机前方）
        if (camera_point.at<double>(2) <= 0.75) continue;

        {
          cv::Mat world_mat = (cv::Mat_<double>(4,1) << 
            world_point.x, world_point.y, 0, 0);
            cv::Mat camera_point = cameras[camera_index].T * world_mat;
            camera_point = camera_point(cv::Range(0, 3), cv::Range::all());
          if (camera_point.at<double>(2)/cv::norm(camera_point) <= concat_param_.blend_weight) {
          } else {
            remap_mask[camera_index].at<uchar>(bev_v, bev_u) = 1;
          }
        }

        // 鱼眼投影
        std::vector<cv::Point3d> object_points = {
          cv::Point3d(
            camera_point.at<double>(0),
            camera_point.at<double>(1),
            camera_point.at<double>(2))
        };

        std::vector<cv::Point2d> image_points;

        cv::omnidir::projectPoints(
          object_points,
          image_points,
          cv::Mat::zeros(3, 1, CV_64F),  // rvec: 无旋转
          cv::Mat::zeros(3, 1, CV_64F),  // tvec: 无平移
          cameras[camera_index].K,
          cameras[camera_index].xi,
          cameras[camera_index].D
        );

        cv::Point2f projected = image_points[0];

        // 检查投影有效性
        if (projected.x >= 0 && projected.x < cameras[camera_index].image_size.width &&
          projected.y >= 0 && projected.y < cameras[camera_index].image_size.height) 
        {
          remap_x[camera_index].at<float>(bev_v, bev_u) = projected.x;
          remap_y[camera_index].at<float>(bev_v, bev_u) = projected.y;
        }
      }
    }
  }

  for (size_t camera_index = 0; camera_index < camera_num; camera_index++) {
    cv::convertMaps(remap_x[camera_index], remap_y[camera_index], map_[camera_index].first, map_[camera_index].second, CV_16SC2);
  }

  computeFusionMasks(remap_mask, fusion_masks_);
}

void ConcatUndistorter::calcPanoramicMap(cv::Size resolution_out) {
  int camera_num = used_concat_camera_parameters_.getCameraNum();
  map_.resize(camera_num);
  fusion_masks_.resize(camera_num);
  
  const double pano_radius = concat_param_.pano_radius;
  const double blend_weight = concat_param_.blend_weight;

  std::vector<struct CameraParams> cameras = get_camera_param(used_concat_camera_parameters_);
  const int bev_width = resolution_out.width;
  const int bev_height = resolution_out.height;
  const int pano_width = resolution_out.width;
  const int pano_height = resolution_out.height;

  // 初始化remap矩阵
  std::vector<cv::Mat> remap_x;
  std::vector<cv::Mat> remap_y;
  std::vector<cv::Mat> remap_mask;
  remap_x.resize(camera_num);
  remap_y.resize(camera_num);
  remap_mask.resize(camera_num);

  for (int cam_idx = 0; cam_idx < camera_num; ++cam_idx) {
    remap_x[cam_idx].create(bev_height, bev_width, CV_32FC1);
    remap_y[cam_idx].create(bev_height, bev_width, CV_32FC1);
    remap_mask[cam_idx].create(bev_height, bev_width, CV_8UC1);
    remap_x[cam_idx].setTo(-1);
    remap_y[cam_idx].setTo(-1);
    remap_mask[cam_idx].setTo(0);
  }
  
  for (int bev_v = 0; bev_v < bev_height; ++bev_v) {
    for (int bev_u = 0; bev_u < bev_width; ++bev_u) {
      // 转换为世界坐标（地面平面）
      double phi = -2 * CV_PI * (bev_u / static_cast<double>(pano_width)) + CV_PI;

      double start_angle = CV_PI*9.0/24.0;
      double end_angle = CV_PI*(1.0-9.0/24.0);
      double delta_angle = end_angle - start_angle;
      double theta = start_angle + (bev_v / static_cast<double>(pano_height)) * delta_angle;

      // 转换为3D单位向量 (假设坐标系为x右，y下，z前)
      cv::Point3f world_point(
        pano_radius * sin(theta) * cos(phi),
        pano_radius * sin(theta) * sin(phi),
        pano_radius * cos(theta)
      );

      // 遍历所有摄像头
      for (int camera_index = 0; camera_index < camera_num; ++camera_index) {
        // 转换到相机坐标系
        cv::Mat world_mat = (cv::Mat_<double>(4, 1) << world_point.x, world_point.y, world_point.z, 1);
        cv::Mat camera_point = cameras[camera_index].T * world_mat;
        if (camera_point.at<double>(2) <= 0.75) continue; // 90度以内

        {
          cv::Mat world_mat = (cv::Mat_<double>(4,1) << 
            world_point.x, world_point.y, 0, 0);
            cv::Mat camera_point = cameras[camera_index].T * world_mat;
            camera_point = camera_point(cv::Range(0, 3), cv::Range::all());
          if (camera_point.at<double>(2)/cv::norm(camera_point) <= blend_weight) {
          } else {
            remap_mask[camera_index].at<uchar>(bev_v, bev_u) = 1;
          }
        }

        // 鱼眼投影
        std::vector<cv::Point3d> object_points = {
          cv::Point3d(
            camera_point.at<double>(0),
            camera_point.at<double>(1),
            camera_point.at<double>(2))
        };

        std::vector<cv::Point2d> image_points;

        cv::omnidir::projectPoints(
          object_points,
          image_points,
          cv::Mat::zeros(3, 1, CV_64F),  // rvec: 无旋转
          cv::Mat::zeros(3, 1, CV_64F),  // tvec: 无平移
          cameras[camera_index].K,
          cameras[camera_index].xi,
          cameras[camera_index].D
        );

        cv::Point2f projected = image_points[0];

        // 检查投影有效性
        if (projected.x >= 0 && projected.x < cameras[camera_index].image_size.width &&
          projected.y >= 0 && projected.y < cameras[camera_index].image_size.height) 
        {
          remap_x[camera_index].at<float>(bev_v, bev_u) = projected.x;
          remap_y[camera_index].at<float>(bev_v, bev_u) = projected.y;
        }
      }
    }
  }

  for (size_t camera_index = 0; camera_index < camera_num; camera_index++) {
    cv::convertMaps(remap_x[camera_index], remap_y[camera_index], map_[camera_index].first, map_[camera_index].second, CV_16SC2);
  }
  computeFusionMasks(remap_mask, fusion_masks_);
}

void ConcatUndistorter::calcPanoramic2Map(cv::Size resolution_out) {
  int camera_num = used_concat_camera_parameters_.getCameraNum();
  map_.resize(camera_num);
  fusion_masks_.resize(camera_num);
  
  const double pano_radius = concat_param_.pano_radius;
  const double blend_weight = concat_param_.blend_weight;

  std::vector<struct CameraParams> cameras = get_camera_param(used_concat_camera_parameters_);
  const int pano_width = resolution_out.width;
  const int pano_height = resolution_out.height;

  // 初始化remap矩阵
  std::vector<cv::Mat> remap_x(camera_num);
  std::vector<cv::Mat> remap_y(camera_num);
  std::vector<cv::Mat> remap_mask(camera_num);

  for (int cam_idx = 0; cam_idx < camera_num; ++cam_idx) {
    remap_x[cam_idx].create(pano_height, pano_width, CV_32FC1);
    remap_y[cam_idx].create(pano_height, pano_width, CV_32FC1);
    remap_mask[cam_idx].create(pano_height, pano_width, CV_8UC1);
    remap_x[cam_idx].setTo(-1);
    remap_y[cam_idx].setTo(-1);
    remap_mask[cam_idx].setTo(0);
  }

  // 球极投影参数
  const double projection_radius = pano_width / (2 * CV_PI); // 投影平面半径
  
  for (int v = 0; v < pano_height; ++v) {
    for (int u = 0; u < pano_width; ++u) {
      // 球极投影坐标转换
      // 将平面坐标转换为球面坐标
      const double x = (u - pano_width/2.0) / projection_radius;
      const double y = (pano_height/2.0 - v) / projection_radius;
      
      // 计算球面坐标
      const double r_sq = x*x + y*y;
      
      const double X = 2*x / (1 + r_sq);
      const double Y = 2*y / (1 + r_sq);
      const double Z = (1 - r_sq) / (1 + r_sq);
      
      // 转换为3D点（假设单位球面）
      cv::Point3f world_point(
        pano_radius * Y,
        -pano_radius * X,
        -pano_radius * Z
      );

      // 遍历所有摄像头
      for (int camera_index = 0; camera_index < camera_num; ++camera_index) {
        // 坐标转换到相机坐标系
        cv::Mat world_mat = (cv::Mat_<double>(4, 1) << world_point.x, world_point.y, world_point.z, 1);
        cv::Mat camera_point = cameras[camera_index].T * world_mat;
        
        // 可见性检查
        if (camera_point.at<double>(2) <= 0) continue;

        // 混合权重计算（基于与视轴的夹角）
        {
          cv::Mat world_mat = (cv::Mat_<double>(4, 1) << world_point.x, world_point.y, world_point.z, 0);
          cv::Mat camera_point = cameras[camera_index].T * world_mat;

          camera_point = camera_point(cv::Range(0, 3), cv::Range::all());
          if (camera_point.at<double>(2)/cv::norm(camera_point) <= concat_param_.blend_weight) {
          } else {
            remap_mask[camera_index].at<uchar>(v, u) = 1;
          }
        }

        // 鱼眼投影
        std::vector<cv::Point3d> object_points = {
          cv::Point3d(
            camera_point.at<double>(0),
            camera_point.at<double>(1),
            camera_point.at<double>(2))
        };

        std::vector<cv::Point2d> image_points;
        cv::omnidir::projectPoints(
          object_points,
          image_points,
          cv::Mat::zeros(3, 1, CV_64F),  // rvec
          cv::Mat::zeros(3, 1, CV_64F),  // tvec
          cameras[camera_index].K,
          cameras[camera_index].xi,
          cameras[camera_index].D
        );

        cv::Point2f projected = image_points[0];
        
        // 有效性检查
        if (projected.x >=0 && projected.x < cameras[camera_index].image_size.width &&
            projected.y >=0 && projected.y < cameras[camera_index].image_size.height) 
        {
          remap_x[camera_index].at<float>(v, u) = projected.x;
          remap_y[camera_index].at<float>(v, u) = projected.y;
        }
      }
    }
  }

  // 转换映射表格式
  for (size_t camera_index = 0; camera_index < camera_num; camera_index++) {
    cv::convertMaps(remap_x[camera_index], remap_y[camera_index], 
                   map_[camera_index].first, map_[camera_index].second, CV_16SC2);
  }
  computeFusionMasks(remap_mask, fusion_masks_);
}
// void integerMaskMultiply(cv::Mat &rgb, cv::Mat &mask, cv::Mat &output) {
//     // Step 2: 提升精度
//     cv::Mat rgb_16u, mask_16u;
//     rgb.convertTo(rgb_16u, CV_16U, 1.0);
//     mask.convertTo(mask_16u, CV_16U, 1.0);

//     // Step 3: 乘法+位移
//     cv::Mat result_16u;
//     cv::multiply(rgb_16u, mask_16u, result_16u);

//     // Step 4: 转回8U
//     result_16u.convertTo(output, CV_8U, 1.0f/255);
// }

void integerMaskMultiply(cv::Mat &rgb, cv::Mat &mask, cv::Mat &output) {
    CV_Assert(!rgb.empty() && !mask.empty());
    CV_Assert(rgb.size() == mask.size());
    CV_Assert(rgb.depth() == CV_8U && mask.depth() == CV_8U);
    if (mask.channels() == 3) {
        cv::multiply(rgb, mask, output, 1.0 / 255.0, CV_8U);
        return;
    }
    
    // 确保mask是单通道CV_8U
    if (mask.channels() > 1) {
        cv::Mat mask_single;
        cv::cvtColor(mask, mask_single, cv::COLOR_BGR2GRAY);
        mask = mask_single;
    }
    CV_Assert(mask.depth() == CV_8U);
    
    // 根据rgb的类型处理
    int rgb_depth = rgb.depth();
    int rgb_channels = rgb.channels();
    
    if (rgb_depth == CV_8U) {
        // 8位图像
        if (rgb_channels == 1) {
            // 单通道8位
            cv::Mat rgb_16u, mask_16u;
            rgb.convertTo(rgb_16u, CV_16U, 1.0);
            mask.convertTo(mask_16u, CV_16U, 1.0);
            
            cv::Mat result_16u;
            cv::multiply(rgb_16u, mask_16u, result_16u);
            result_16u.convertTo(output, CV_8U, 1.0f/255);
        } 
        else if (rgb_channels == 3) {
            // 3通道8位 - 使用您的优化方法
            cv::Mat mask_3ch;
            cv::cvtColor(mask, mask_3ch, cv::COLOR_GRAY2BGR);
            
            cv::Mat rgb_16u, mask_16u;
            rgb.convertTo(rgb_16u, CV_16U, 1.0);
            mask_3ch.convertTo(mask_16u, CV_16U, 1.0);
            
            cv::Mat result_16u;
            cv::multiply(rgb_16u, mask_16u, result_16u);
            result_16u.convertTo(output, CV_8U, 1.0f/255);
        }
    } 
    else if (rgb_depth == CV_16U) {
        // 16位图像
        if (rgb_channels == 1) {
            // 单通道16位
            cv::Mat mask_16u;
            mask.convertTo(mask_16u, CV_16U, 257.0); // 255 * 257=65535
            
            cv::Mat result_32u;
            cv::multiply(rgb, mask_16u, result_32u, 1.0, CV_32S);
            result_32u.convertTo(output, CV_16U, 1.0f/65535);
        } 
        else if (rgb_channels == 3) {
            // 3通道16位
            cv::Mat mask_3ch;
            cv::cvtColor(mask, mask_3ch, cv::COLOR_GRAY2BGR);
            
            cv::Mat mask_16u;
            mask_3ch.convertTo(mask_16u, CV_16U, 257.0); // 255 * 257=65535
            
            cv::Mat result_32u;
            cv::multiply(rgb, mask_16u, result_32u, 1.0, CV_32S);
            result_32u.convertTo(output, CV_16U, 1.0f/65535);
        }
    } 
    else {
        CV_Assert(false && "Unsupported depth type");
    }
}

void ConcatUndistorter::calcEquirectMap(cv::Size resolution_out) {
  int camera_num = used_concat_camera_parameters_.getCameraNum();
  map_.resize(camera_num);
  fusion_masks_.resize(camera_num);

  const double pano_radius = concat_param_.pano_radius;
  const double blend_weight = concat_param_.blend_weight;
  const double vfov = concat_param_.pano_vfov_deg * CV_PI / 180.0;

  std::vector<struct CameraParams> cameras =
      get_camera_param(used_concat_camera_parameters_);
  const int pano_width = resolution_out.width;
  const int pano_height = resolution_out.height;

  std::vector<cv::Mat> remap_x(camera_num);
  std::vector<cv::Mat> remap_y(camera_num);
  std::vector<cv::Mat> remap_mask(camera_num);
  for (int cam_idx = 0; cam_idx < camera_num; ++cam_idx) {
    remap_x[cam_idx].create(pano_height, pano_width, CV_32FC1);
    remap_y[cam_idx].create(pano_height, pano_width, CV_32FC1);
    remap_mask[cam_idx].create(pano_height, pano_width, CV_8UC1);
    remap_x[cam_idx].setTo(-1);
    remap_y[cam_idx].setTo(-1);
    remap_mask[cam_idx].setTo(0);
  }

  for (int v = 0; v < pano_height; ++v) {
    const double lat =
        (0.5 - (v + 0.5) / static_cast<double>(pano_height)) * vfov;
    const double cos_lat = std::cos(lat);
    const double sin_lat = std::sin(lat);
    for (int u = 0; u < pano_width; ++u) {
      const double lon =
          ((u + 0.5) / static_cast<double>(pano_width) - 0.5) * 2.0 * CV_PI;
      const cv::Point3f world_point(
          pano_radius * cos_lat * std::sin(lon),
          -pano_radius * sin_lat,
          pano_radius * cos_lat * std::cos(lon));

      for (int camera_index = 0; camera_index < camera_num; ++camera_index) {
        cv::Mat world_mat =
            (cv::Mat_<double>(4, 1) << world_point.x, world_point.y,
             world_point.z, 1);
        cv::Mat camera_point = cameras[camera_index].T * world_mat;
        if (camera_point.at<double>(2) <= 0) continue;

        {
          cv::Mat dir_mat =
              (cv::Mat_<double>(4, 1) << world_point.x, world_point.y,
               world_point.z, 0);
          cv::Mat dir_cam = cameras[camera_index].T * dir_mat;
          dir_cam = dir_cam(cv::Range(0, 3), cv::Range::all());
          if (dir_cam.at<double>(2) / cv::norm(dir_cam) <= blend_weight) {
          } else {
            remap_mask[camera_index].at<uchar>(v, u) = 1;
          }
        }

        std::vector<cv::Point3d> object_points = {
            cv::Point3d(camera_point.at<double>(0),
                        camera_point.at<double>(1),
                        camera_point.at<double>(2))};
        std::vector<cv::Point2d> image_points;
        cv::omnidir::projectPoints(
            object_points, image_points, cv::Mat::zeros(3, 1, CV_64F),
            cv::Mat::zeros(3, 1, CV_64F), cameras[camera_index].K,
            cameras[camera_index].xi, cameras[camera_index].D);

        const cv::Point2f projected = image_points[0];
        if (projected.x >= 0 &&
            projected.x < cameras[camera_index].image_size.width &&
            projected.y >= 0 &&
            projected.y < cameras[camera_index].image_size.height) {
          remap_x[camera_index].at<float>(v, u) = projected.x;
          remap_y[camera_index].at<float>(v, u) = projected.y;
        }
      }
    }
  }

  for (int camera_index = 0; camera_index < camera_num; ++camera_index) {
    cv::convertMaps(remap_x[camera_index], remap_y[camera_index],
                    map_[camera_index].first, map_[camera_index].second,
                    CV_16SC2);
  }
  computeFusionMasks(remap_mask, fusion_masks_);
}

void ConcatUndistorter::undistortImage(int method, 
                      const cv::Mat& image_0,
                      const cv::Mat& image_1,
                      const cv::Mat& image_2,
                      const cv::Mat& image_3,
                      const cv::Mat& image_4,
                      const cv::Mat& image_5,
                      cv::Mat* undistored_image)
{
  int camera_num = used_concat_camera_parameters_.getCameraNum();

  // Keep all processing on CPU cv::Mat. The UMat path leaks memory on the
  // Seeker board's OpenCV 4.2 build and eventually triggers the OOM killer.
  auto getMatImage = [&](int idx) -> cv::Mat {
      switch(idx) {
          case 0: return image_0;
          case 1: return image_1;
          case 2: return image_2;
          case 3: return image_3;
          case 4: return image_4;
          case 5: return image_5;
          default: return cv::Mat();
      }
  };
  
  switch (method) {
    case 0: {
      if (undistored_image->size() !=
              cv::Size(concat_param_.concat_width, concat_param_.concat_height) ||
          undistored_image->type() != image_0.type()) {
        undistored_image->create(concat_param_.concat_height,
                                 concat_param_.concat_width, image_0.type());
      }
      const int width = concat_param_.concat_width / camera_num;
      const int height = concat_param_.concat_height;

      if (camera_num == 4) {
        const int indices[4] = {3, 0, 1, 2};
        for (int slot = 0; slot < 4; ++slot) {
          const int idx = indices[slot];
          cv::Mat roi = (*undistored_image)(cv::Rect(slot * width, 0, width, height));
          cv::remap(getMatImage(idx), roi, map_[idx].first, map_[idx].second,
                    cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        }
      } else if (camera_num == 6) {
        if (scratch_dst_.size() != cv::Size(width, height) ||
            scratch_dst_.type() != image_0.type()) {
          scratch_dst_.create(height, width, image_0.type());
        }
        for (int i = 0; i < 6; i++) {
          cv::remap(getMatImage(i), scratch_dst_, map_[i].first, map_[i].second,
                    cv::INTER_LINEAR, cv::BORDER_CONSTANT);
          cv::Mat roi = (*undistored_image)(cv::Rect(i * width, 0, width, height));
          scratch_dst_.copyTo(roi);
        }
      }
      break;
    }
    case 1:
    case 2:
    case 3:
    case 4: {
      const cv::Size out_size(concat_param_.concat_width, concat_param_.concat_height);
      if (undistored_image->size() != out_size ||
          undistored_image->type() != image_0.type()) {
        undistored_image->create(out_size.height, out_size.width, image_0.type());
      }
      if (image_0.type() != CV_8UC3 ||
          fusion_masks_.size() < static_cast<size_t>(camera_num)) {
        undistored_image->setTo(cv::Scalar::all(0));
        break;
      }
      if (scratch_dst_.size() != out_size || scratch_dst_.type() != CV_8UC3) {
        scratch_dst_.create(out_size.height, out_size.width, CV_8UC3);
      }
      if (cam_dst_.size() != static_cast<size_t>(camera_num)) {
        cam_dst_.resize(camera_num);
        cam_weighted_.resize(camera_num);
      }
      for (int i = 0; i < camera_num; ++i) {
        if (cam_dst_[i].size() != out_size || cam_dst_[i].type() != CV_8UC3) {
          cam_dst_[i].create(out_size.height, out_size.width, CV_8UC3);
        }
        if (cam_weighted_[i].size() != out_size ||
            cam_weighted_[i].type() != CV_16UC3) {
          cam_weighted_[i].create(out_size.height, out_size.width, CV_16UC3);
        }
      }
      if (accum16_.size() != out_size || accum16_.type() != CV_16UC3) {
        accum16_.create(out_size.height, out_size.width, CV_16UC3);
      }
      cv::parallel_for_(cv::Range(0, camera_num), [&](const cv::Range& range) {
        for (int i = range.start; i < range.end; ++i) {
          cv::remap(getMatImage(i), cam_dst_[i], map_[i].first, map_[i].second,
                    cv::INTER_LINEAR, cv::BORDER_CONSTANT);
          cv::multiply(cam_dst_[i], fusion_masks_[i], cam_weighted_[i],
                       1.0 / 255.0, CV_16U);
        }
      });
      accum16_.setTo(cv::Scalar::all(0));
      for (int i = 0; i < camera_num; ++i) {
        cv::add(accum16_, cam_weighted_[i], accum16_);
      }
      accum16_.convertTo(*undistored_image, CV_8UC3);
      break;
    }
    default:
      break;
  }
}

const ConcatCameraParameters& ConcatUndistorter::getCameraParametersPair() {
  return used_concat_camera_parameters_;
}

// needed so that topic filters can be initalized in the constructor
int ImageConcat::getQueueSize() const {
  int queue_size;
  nh_private_.param("queue_size", queue_size, kQueueSize);
  if (queue_size < 1) {
    ROS_ERROR("Queue size must be >= 1, setting to 1");
    queue_size = 1;
  }
  return queue_size;
}

ImageConcat::ImageConcat(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      it_(nh_),
      queue_size_(getQueueSize()),
      first_image_sub_(it_, "raw/first/image", queue_size_),
      second_image_sub_(it_, "raw/second/image", queue_size_),
      third_image_sub_(it_, "raw/third/image", queue_size_),
      fourth_image_sub_(it_, "raw/fourth/image", queue_size_),
      fifth_image_sub_(it_, "raw/fifth/image", queue_size_),
      sixth_image_sub_(it_, "raw/sixth/image", queue_size_),
      undistorter_ptr_(nullptr),
      frame_counter_(0) {

  mallopt(M_MMAP_THRESHOLD, 128 * 1024);
  mallopt(M_TRIM_THRESHOLD, 128 * 1024);

  // set parameters from ros
  nh_private_.param("input_camera_info_from_ros_params",
                    input_camera_info_from_ros_params_,
                    kDefaultInputCameraInfoFromROSParams);

  bool approx;
  nh_private_.param("approximate_sync", approx, false);

  bool invert_T;
  nh_private_.param("invert_T", invert_T, kDefaultInvertT);

  nh_private_.param("queue_size", queue_size_, kQueueSize);
  if (queue_size_ < 1) {
    ROS_ERROR("Queue size must be >= 1, setting to 1");
    queue_size_ = 1;
  }

  int camera_num;
  nh_private_.param("camera_num", camera_num, 4); // 默认4个摄像头，可以设置为6
  if (camera_num != 4 && camera_num != 5 && camera_num != 6) {
    ROS_ERROR("camera_num must be 4 or 6, setting to 4");
    camera_num = 4;
  }
  
  camera_parameters_ptr_ = std::make_shared<ConcatCameraParameters>(camera_num);

  nh_private_.param("concat_width", concat_param_.concat_width, kDefaultConcatWidth);
  if (concat_param_.concat_width < 120) {
    ROS_ERROR("Output width must be >= 120, setting to 120");
    concat_param_.concat_width = 120;
  }

  nh_private_.param("concat_height", concat_param_.concat_height, kDefaultConcatHeight);
  if (concat_param_.concat_height < 120) {
    ROS_ERROR("Output height must be >= 120, setting to 120");
    concat_param_.concat_height = 120;
  }

  nh_private_.param("process_every_nth_frame", process_every_nth_frame_,
                    kDefaultProcessEveryNthFrame);
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

  nh_private_.param("use_image_transport", use_image_transport_,
                    kDefaultUseImageTransport);

  if (use_image_transport_) {
    it_concat_image_pub_ = it_.advertise("rect/image", queue_size_);
  } else {
    concat_image_pub_ = nh_.advertise<sensor_msgs::Image>("rect/image", queue_size_);
  }

  if (input_camera_info_from_ros_params_) {
    std::string first_camera_namespace, second_camera_namespace,
                third_camera_namespace, fourth_camera_namespace,
                fifth_camera_namespace, sixth_camera_namespace;
    nh_private_.param("first_camera_namespace", first_camera_namespace,
                      kDefaultFirstCameraNamespace);
    nh_private_.param("second_camera_namespace", second_camera_namespace,
                      kDefaultSecondCameraNamespace);
    nh_private_.param("third_camera_namespace", third_camera_namespace,
                      kDefaultThirdCameraNamespace);
    nh_private_.param("fourth_camera_namespace", fourth_camera_namespace,
                      kDefaultFourthCameraNamespace);
    nh_private_.param("fifth_camera_namespace", fifth_camera_namespace,
                      kDefaultFifthCameraNamespace);
    nh_private_.param("sixth_camera_namespace", sixth_camera_namespace,
                      kDefaultSixthCameraNamespace);

    // 设置相机参数
    bool success = true;
    success = success && camera_parameters_ptr_->setInputCameraParameters(
            nh_private_, first_camera_namespace, 0, invert_T);
    success = success && camera_parameters_ptr_->setInputCameraParameters(
            nh_private_, second_camera_namespace, 1, invert_T);
    success = success && camera_parameters_ptr_->setInputCameraParameters(
            nh_private_, third_camera_namespace, 2, invert_T);
    success = success && camera_parameters_ptr_->setInputCameraParameters(
            nh_private_, fourth_camera_namespace, 3, invert_T);
            
    // 如果是6个摄像头，设置额外的两个
    if (camera_num == 5) {
      success = success && camera_parameters_ptr_->setInputCameraParameters(
              nh_private_, fifth_camera_namespace, 4, invert_T);
    }
    if (camera_num == 6) {
      success = success && camera_parameters_ptr_->setInputCameraParameters(
              nh_private_, fifth_camera_namespace, 4, invert_T);
      success = success && camera_parameters_ptr_->setInputCameraParameters(
              nh_private_, sixth_camera_namespace, 5, invert_T);
    }
    
    if (!success) {
      ROS_FATAL("Loading of input camera parameters failed, exiting");
      ros::shutdown();
      exit(EXIT_FAILURE);
    }
    
    nh_private_.param("concat_method", concat_param_.concat_method, kDefaultConcatMethod);
    nh_private_.param("blend_weight", concat_param_.blend_weight, kDefaultBlendWeight);
    nh_private_.param("pano_radius", concat_param_.pano_radius, kDefaultPanoRadius);
    nh_private_.param("pano_vfov_deg", concat_param_.pano_vfov_deg, kDefaultPanoVFovDeg);
    nh_private_.param("bev_ground_height", concat_param_.bev_ground_height, kDefaultBEVGroundHeight);
    nh_private_.param("bev_fx", concat_param_.bev_fx, kDefaultBevFX);

    updateUndistorter();

    camera_num_ = camera_num;
    if (camera_num == 4) {
      first_image_sub_.registerCallback(
          boost::bind(&ImageConcat::latestImageCallback, this, 0, _1));
      second_image_sub_.registerCallback(
          boost::bind(&ImageConcat::latestImageCallback, this, 1, _1));
      third_image_sub_.registerCallback(
          boost::bind(&ImageConcat::latestImageCallback, this, 2, _1));
      fourth_image_sub_.registerCallback(
          boost::bind(&ImageConcat::latestImageCallback, this, 3, _1));

      nh_private_.param("target_fps", target_fps_, 10);
      target_fps_ = std::max(1, std::min(target_fps_, 30));
      nh_private_.param("max_frame_age_s", max_frame_age_s_, 0.35);
      nh_private_.param("max_frame_skew_s", max_frame_skew_s_, 0.20);
      process_timer_ = nh_.createTimer(
          ros::Duration(1.0 / static_cast<double>(target_fps_)),
          &ImageConcat::processLatest, this);
      ROS_INFO("Latest-frame pipeline: target_fps=%d max_age=%.2fs max_skew=%.2fs",
               target_fps_, max_frame_age_s_, max_frame_skew_s_);
    } else if (camera_num == 5) {
      if (approx) {
        image_sync_ptr_5_ =
            std::make_shared<message_filters::Synchronizer<ImageSyncPolicy5>>(
                ImageSyncPolicy5(queue_size_), first_image_sub_, second_image_sub_, third_image_sub_,
                fourth_image_sub_, fifth_image_sub_);
        image_sync_ptr_5_->registerCallback(
            boost::bind(&ImageConcat::imagesCallback5, this, _1, _2, _3, _4, _5));
      } else {
        exact_image_sync_ptr_5_ =
            std::make_shared<message_filters::Synchronizer<ExactImageSyncPolicy5>>(
                ExactImageSyncPolicy5(queue_size_), first_image_sub_, second_image_sub_, third_image_sub_,
                fourth_image_sub_, fifth_image_sub_);
        exact_image_sync_ptr_5_->registerCallback(
            boost::bind(&ImageConcat::imagesCallback5, this, _1, _2, _3, _4, _5));
      }
    } else if (camera_num == 6) {
      if (approx) {
        image_sync_ptr_6_ =
            std::make_shared<message_filters::Synchronizer<ImageSyncPolicy6>>(
                ImageSyncPolicy6(queue_size_), first_image_sub_, second_image_sub_, third_image_sub_,
                fourth_image_sub_, fifth_image_sub_, sixth_image_sub_);
        image_sync_ptr_6_->registerCallback(
            boost::bind(&ImageConcat::imagesCallback6, this, _1, _2, _3, _4, _5, _6));
      } else {
        exact_image_sync_ptr_6_ =
            std::make_shared<message_filters::Synchronizer<ExactImageSyncPolicy6>>(
                ExactImageSyncPolicy6(queue_size_), first_image_sub_, second_image_sub_, third_image_sub_,
                fourth_image_sub_, fifth_image_sub_, sixth_image_sub_);
        exact_image_sync_ptr_6_->registerCallback(
            boost::bind(&ImageConcat::imagesCallback6, this, _1, _2, _3, _4, _5, _6));
      }
    }
  } else {
    // 处理不从ROS参数获取相机信息的情况
    ROS_ERROR("Only input_camera_info_from_ros_params mode is supported for multiple cameras");
  }
}

void ImageConcat::updateUndistorter() {
  // if undistorter not built or built using old data update it
  if (!undistorter_ptr_ ) {
    try {
      undistorter_ptr_ =
          std::make_shared<ConcatUndistorter>(*camera_parameters_ptr_, concat_param_);
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
      return;
    }
  }
}

void ImageConcat::latestImageCallback(int index,
                                     const sensor_msgs::ImageConstPtr& msg) {
  if (index < 0 || index >= 4) {
    return;
  }
  std::lock_guard<std::mutex> lock(latest_mutex_);
  latest_images_[index] = msg;
}

void ImageConcat::processLatest(const ros::TimerEvent& event) {
  (void)event;
  sensor_msgs::ImageConstPtr frames[4];
  {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    for (int i = 0; i < 4; ++i) {
      frames[i] = latest_images_[i];
    }
  }
  if (!frames[0] || !frames[1] || !frames[2] || !frames[3]) {
    return;
  }

  ros::Time newest = frames[0]->header.stamp;
  ros::Time oldest = newest;
  for (int i = 1; i < 4; ++i) {
    const ros::Time& stamp = frames[i]->header.stamp;
    if (stamp > newest) newest = stamp;
    if (stamp < oldest) oldest = stamp;
  }
  if (newest.isZero() || oldest.isZero()) {
    return;
  }
  const ros::Time now = ros::Time::now();
  if ((now - newest).toSec() > max_frame_age_s_) {
    return;
  }
  if ((newest - oldest).toSec() > max_frame_skew_s_) {
    return;
  }

  std::unique_lock<std::mutex> processing_lock(process_mutex_, std::try_to_lock);
  if (!processing_lock.owns_lock()) {
    return;
  }
  processAndSendImage4(frames[0], frames[1], frames[2], frames[3]);
}

void ImageConcat::processAndSendImage4( 
    const sensor_msgs::ImageConstPtr& first_image_msg_in,
    const sensor_msgs::ImageConstPtr& second_image_msg_in,
    const sensor_msgs::ImageConstPtr& third_image_msg_in,
    const sensor_msgs::ImageConstPtr& fourth_image_msg_in) {
  cv_bridge::CvImageConstPtr image0_in_ptr =
      cv_bridge::toCvShare(first_image_msg_in, output_image_type_);
  cv_bridge::CvImageConstPtr image1_in_ptr =
      cv_bridge::toCvShare(second_image_msg_in, output_image_type_);
  cv_bridge::CvImageConstPtr image2_in_ptr =
      cv_bridge::toCvShare(third_image_msg_in, output_image_type_);
  cv_bridge::CvImageConstPtr image3_in_ptr =
      cv_bridge::toCvShare(fourth_image_msg_in, output_image_type_);

  std::string encoding = image0_in_ptr->encoding;
  if (encoding == "8UC1") {
    // ros does not recognize U8C1 and it will crash the disparity node
    encoding = "mono8";
  }

  // 对于4个摄像头的情况，后两个参数传空Mat
  cv::Mat empty_mat;
  undistorter_ptr_->undistortImage(concat_param_.concat_method,
                      image0_in_ptr->image,
                      image1_in_ptr->image,
                      image2_in_ptr->image,
                      image3_in_ptr->image,
                      empty_mat,  // 第5个摄像头图像为空
                      empty_mat,  // 第6个摄像头图像为空
                      &output_image_);

  if (output_msgs_.empty()) {
    output_msgs_.resize(4);
    for (std::size_t i = 0; i < output_msgs_.size(); ++i) {
      output_msgs_[i] = boost::make_shared<sensor_msgs::Image>();
    }
  }
  sensor_msgs::ImagePtr image_out = output_msgs_[output_msg_index_];
  output_msg_index_ = (output_msg_index_ + 1) % output_msgs_.size();
  image_out->header = first_image_msg_in->header;
  image_out->height = output_image_.rows;
  image_out->width = output_image_.cols;
  image_out->encoding = encoding;
  image_out->is_bigendian = 0;
  image_out->step = output_image_.step;
  const std::size_t bytes = output_image_.total() * output_image_.elemSize();
  if (image_out->data.size() != bytes) {
    image_out->data.resize(bytes);
  }
  if (bytes > 0) {
    std::memcpy(image_out->data.data(), output_image_.data, bytes);
  }
  if (use_image_transport_) {
    it_concat_image_pub_.publish(image_out);
  } else {
    concat_image_pub_.publish(image_out);
  }
}

void ImageConcat::processAndSendImage5( 
    const sensor_msgs::ImageConstPtr& first_image_msg_in,
    const sensor_msgs::ImageConstPtr& second_image_msg_in,
    const sensor_msgs::ImageConstPtr& third_image_msg_in,
    const sensor_msgs::ImageConstPtr& fourth_image_msg_in,
    const sensor_msgs::ImageConstPtr& fifth_image_msg_in) {
  cv_bridge::CvImageConstPtr image0_in_ptr =
      cv_bridge::toCvShare(first_image_msg_in, output_image_type_);
  cv_bridge::CvImageConstPtr image1_in_ptr =
      cv_bridge::toCvShare(second_image_msg_in, output_image_type_);
  cv_bridge::CvImageConstPtr image2_in_ptr =
      cv_bridge::toCvShare(third_image_msg_in, output_image_type_);
  cv_bridge::CvImageConstPtr image3_in_ptr =
      cv_bridge::toCvShare(fourth_image_msg_in, output_image_type_);
  cv_bridge::CvImageConstPtr image4_in_ptr =
      cv_bridge::toCvShare(fifth_image_msg_in, output_image_type_);

  std::string encoding = image0_in_ptr->encoding;
  if (encoding == "8UC1") {
    // ros does not recognize U8C1 and it will crash the disparity node
    encoding = "mono8";
  }

  cv_bridge::CvImagePtr image_out_ptr(
      new cv_bridge::CvImage(first_image_msg_in->header, encoding));

  cv::Mat empty_mat;
  undistorter_ptr_->undistortImage(concat_param_.concat_method,
                      image0_in_ptr->image,
                      image1_in_ptr->image,
                      image2_in_ptr->image,
                      image3_in_ptr->image,
                      image4_in_ptr->image,
                      empty_mat,
                      &(image_out_ptr->image));

  if (use_image_transport_) {
    it_concat_image_pub_.publish(*(image_out_ptr->toImageMsg()));
  } else {
    concat_image_pub_.publish(*(image_out_ptr->toImageMsg()));
  }
}

void ImageConcat::processAndSendImage6( 
    const sensor_msgs::ImageConstPtr& first_image_msg_in,
    const sensor_msgs::ImageConstPtr& second_image_msg_in,
    const sensor_msgs::ImageConstPtr& third_image_msg_in,
    const sensor_msgs::ImageConstPtr& fourth_image_msg_in,
    const sensor_msgs::ImageConstPtr& fifth_image_msg_in,
    const sensor_msgs::ImageConstPtr& sixth_image_msg_in) {
  cv_bridge::CvImageConstPtr image0_in_ptr =
      cv_bridge::toCvShare(first_image_msg_in, output_image_type_);
  cv_bridge::CvImageConstPtr image1_in_ptr =
      cv_bridge::toCvShare(second_image_msg_in, output_image_type_);
  cv_bridge::CvImageConstPtr image2_in_ptr =
      cv_bridge::toCvShare(third_image_msg_in, output_image_type_);
  cv_bridge::CvImageConstPtr image3_in_ptr =
      cv_bridge::toCvShare(fourth_image_msg_in, output_image_type_);
  cv_bridge::CvImageConstPtr image4_in_ptr =
      cv_bridge::toCvShare(fifth_image_msg_in, output_image_type_);
  cv_bridge::CvImageConstPtr image5_in_ptr =
      cv_bridge::toCvShare(sixth_image_msg_in, output_image_type_);

  std::string encoding = image0_in_ptr->encoding;
  if (encoding == "8UC1") {
    // ros does not recognize U8C1 and it will crash the disparity node
    encoding = "mono8";
  }

  cv_bridge::CvImagePtr image_out_ptr(
      new cv_bridge::CvImage(first_image_msg_in->header, encoding));

  undistorter_ptr_->undistortImage(concat_param_.concat_method,
                      image0_in_ptr->image,
                      image1_in_ptr->image,
                      image2_in_ptr->image,
                      image3_in_ptr->image,
                      image4_in_ptr->image,
                      image5_in_ptr->image,
                      &(image_out_ptr->image));

  if (use_image_transport_) {
    it_concat_image_pub_.publish(*(image_out_ptr->toImageMsg()));
  } else {
    concat_image_pub_.publish(*(image_out_ptr->toImageMsg()));
  }
}

void ImageConcat::imagesCallback4(const sensor_msgs::ImageConstPtr& first_image_msg_in,
                                 const sensor_msgs::ImageConstPtr& second_image_msg_in,
                                 const sensor_msgs::ImageConstPtr& third_image_msg_in,
                                 const sensor_msgs::ImageConstPtr& fourth_image_msg_in) {
  if (++frame_counter_ < process_every_nth_frame_) {
    return;
  }
  frame_counter_ = 0;
  processAndSendImage4(first_image_msg_in, second_image_msg_in,
                      third_image_msg_in, fourth_image_msg_in);
}

void ImageConcat::imagesCallback5(const sensor_msgs::ImageConstPtr& first_image_msg_in,
                                 const sensor_msgs::ImageConstPtr& second_image_msg_in,
                                 const sensor_msgs::ImageConstPtr& third_image_msg_in,
                                 const sensor_msgs::ImageConstPtr& fourth_image_msg_in,
                                 const sensor_msgs::ImageConstPtr& fifth_image_msg_in) {
  if (++frame_counter_ < process_every_nth_frame_) {
    return;
  }
  frame_counter_ = 0;
  processAndSendImage5(first_image_msg_in, second_image_msg_in,
                      third_image_msg_in, fourth_image_msg_in,
                      fifth_image_msg_in);
}

void ImageConcat::imagesCallback6(const sensor_msgs::ImageConstPtr& first_image_msg_in,
                                 const sensor_msgs::ImageConstPtr& second_image_msg_in,
                                 const sensor_msgs::ImageConstPtr& third_image_msg_in,
                                 const sensor_msgs::ImageConstPtr& fourth_image_msg_in,
                                 const sensor_msgs::ImageConstPtr& fifth_image_msg_in,
                                 const sensor_msgs::ImageConstPtr& sixth_image_msg_in) {
  if (++frame_counter_ < process_every_nth_frame_) {
    return;
  }
  frame_counter_ = 0;
  processAndSendImage6(first_image_msg_in, second_image_msg_in,
                      third_image_msg_in, fourth_image_msg_in,
                      fifth_image_msg_in, sixth_image_msg_in);
}

}  // namespace image_undistort
