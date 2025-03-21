#include <ros/package.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <sensor_msgs/PointCloud2.h>

#include <pangolin/pangolin.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <unordered_map>

#include "extrinsic_param.hpp"
#include "multi_sensor_calibration/CGI610GNSSMsg.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "autocalib/lidar2imu/BALM.hpp"
#include "autocalib/lidar2imu/gen_BALM_feature.hpp"
#include "autocalib/lidar2imu/transform_util.hpp"
#include "autocalib/lidar2imu/earth.h"
#include "autocalib/lidar2imu/common/Lidar_parser_base.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

using namespace std;

#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049
#define MAX_RADAR_TIME_GAP 15 * 1e6

string pkg_loc;

std::vector<pcl::PointCloud<LidarPointXYZIRT>> pcds;
std::vector<Eigen::Matrix4d> lidar_poses;

pangolin::GlBuffer *source_vertexBuffer_;
pangolin::GlBuffer *source_colorBuffer_;
pangolin::GlBuffer *target_vertexBuffer_;
pangolin::GlBuffer *target_colorBuffer_;

double cali_scale_degree_ = 0.3;
double cali_scale_trans_ = 0.06;
static Eigen::Matrix4d calibration_matrix_ = Eigen::Matrix4d::Identity();
static Eigen::Matrix4d orign_calibration_matrix_ = Eigen::Matrix4d::Identity();
std::vector<Eigen::Matrix4d> modification_list_;
bool display_mode_ = false;
int point_size_ = 2;
bool start_origin;
Eigen::Vector3d origin_blh;
Eigen::Matrix3d origin_qua;

struct RGB {
  unsigned char r;
  unsigned char g;
  unsigned char b;
};

struct PointCloudBbox {
  int min_x = 0;
  int min_y = 0;
  int min_z = 0;

  int max_x = 0;
  int max_y = 0;
  int max_z = 0;
};

pcl::PointCloud<pcl::PointXYZI>::Ptr
    cloudLidar(new pcl::PointCloud<pcl::PointXYZI>);
;
pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>::Ptr
    all_octree(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>(0.05));

double degree_2_radian = 0.017453293;

bool kbhit() {
  termios term;
  tcgetattr(0, &term);
  termios term2 = term;
  term2.c_lflag &= ~ICANON;
  tcsetattr(0, TCSANOW, &term2);
  int byteswaiting;
  ioctl(0, FIONREAD, &byteswaiting);
  tcsetattr(0, TCSANOW, &term);
  return byteswaiting > 0;
}

void CalibrationInit(Eigen::Matrix4d json_param) {
  Eigen::Matrix4d init_cali;
  init_cali << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  calibration_matrix_ = json_param;
  orign_calibration_matrix_ = json_param;
  modification_list_.reserve(12);
  for (int32_t i = 0; i < 12; i++) {
    std::vector<int> transform_flag(6, 0);
    transform_flag[i / 2] = (i % 2) ? (-1) : 1;
    Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rot_tmp;
    rot_tmp =
        Eigen::AngleAxisd(transform_flag[0] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(transform_flag[1] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(transform_flag[2] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitZ());
    tmp.block(0, 0, 3, 3) = rot_tmp;
    tmp(0, 3) = transform_flag[3] * cali_scale_trans_;
    tmp(1, 3) = transform_flag[4] * cali_scale_trans_;
    tmp(2, 3) = transform_flag[5] * cali_scale_trans_;
    modification_list_[i] = tmp;
  }
  std::cout << "=>Calibration scale Init!\n";
}

void CalibrationScaleChange() {
  Eigen::Matrix4d init_cali;
  init_cali << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  modification_list_.reserve(12);
  for (int32_t i = 0; i < 12; i++) {
    std::vector<int> transform_flag(6, 0);
    transform_flag[i / 2] = (i % 2) ? (-1) : 1;
    Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rot_tmp;
    rot_tmp =
        Eigen::AngleAxisd(transform_flag[0] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(transform_flag[1] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(transform_flag[2] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitZ());
    tmp.block(0, 0, 3, 3) = rot_tmp;
    tmp(0, 3) = transform_flag[3] * cali_scale_trans_;
    tmp(1, 3) = transform_flag[4] * cali_scale_trans_;
    tmp(2, 3) = transform_flag[5] * cali_scale_trans_;
    modification_list_[i] = tmp;
  }
  std::cout << "=>Calibration scale update done!\n";
}

void saveResult(const int &frame_id) {
  std::string file_name = pkg_loc + "/data/lidar2imu_extrinsic_" + std::to_string(frame_id) + ".txt";
  std::ofstream fCalib(file_name);
  if (!fCalib.is_open()) {
    std::cerr << "open file " << file_name << " failed." << std::endl;
    return;
  }
  fCalib << "Extrinsic:" << std::endl;
  fCalib << "R:\n"
         << calibration_matrix_(0, 0) << " " << calibration_matrix_(0, 1) << " "
         << calibration_matrix_(0, 2) << "\n"
         << calibration_matrix_(1, 0) << " " << calibration_matrix_(1, 1) << " "
         << calibration_matrix_(1, 2) << "\n"
         << calibration_matrix_(2, 0) << " " << calibration_matrix_(2, 1) << " "
         << calibration_matrix_(2, 2) << std::endl;
  fCalib << "t: " << calibration_matrix_(0, 3) << " "
         << calibration_matrix_(1, 3) << " " << calibration_matrix_(2, 3)
         << std::endl;

  fCalib << "************* json format *************" << std::endl;
  fCalib << "Extrinsic:" << std::endl;
  fCalib << "[" << calibration_matrix_(0, 0) << "," << calibration_matrix_(0, 1)
         << "," << calibration_matrix_(0, 2) << "," << calibration_matrix_(0, 3)
         << "],"
         << "[" << calibration_matrix_(1, 0) << "," << calibration_matrix_(1, 1)
         << "," << calibration_matrix_(1, 2) << "," << calibration_matrix_(1, 3)
         << "],"
         << "[" << calibration_matrix_(2, 0) << "," << calibration_matrix_(2, 1)
         << "," << calibration_matrix_(2, 2) << "," << calibration_matrix_(2, 3)
         << "],"
         << "[" << calibration_matrix_(3, 0) << "," << calibration_matrix_(3, 1)
         << "," << calibration_matrix_(3, 2) << "," << calibration_matrix_(3, 3)
         << "]" << std::endl;
  fCalib.close();
}

bool ManualCalibration(int key_input) {
  char table[] = {'q', 'a', 'w', 's', 'e', 'd', 'r', 'f', 't', 'g', 'y', 'h'};
  bool real_hit = false;
  for (int32_t i = 0; i < 12; i++) {
    if (key_input == table[i]) {
      calibration_matrix_ = calibration_matrix_ * modification_list_[i];
      real_hit = true;
    }
  }
  return real_hit;
}

RGB GreyToColorMix(int val) {
  int r, g, b;
  if (val < 128) {
    r = 0;
  } else if (val < 192) {
    r = 255 / 64 * (val - 128);
  } else {
    r = 255;
  }
  if (val < 64) {
    g = 255 / 64 * val;
  } else if (val < 192) {
    g = 255;
  } else {
    g = -255 / 63 * (val - 192) + 255;
  }
  if (val < 64) {
    b = 255;
  } else if (val < 128) {
    b = -255 / 63 * (val - 192) + 255;
  } else {
    b = 0;
  }
  RGB rgb;
  rgb.b = b;
  rgb.g = g;
  rgb.r = r;
  return rgb;
}

bool is_exists(const std::string &name) {
  std::ifstream f(name.c_str());
  return f.good();
}

void PointCloudFilterByROI(const pcl::PointCloud<LidarPointXYZIRT>::Ptr &in_cloud,
                           const PointCloudBbox &roi,
                           pcl::PointCloud<LidarPointXYZIRT>::Ptr &out_cloud) {
  out_cloud->clear();
  for (const auto &src_pt : in_cloud->points) {
    if (src_pt.x > roi.min_x && src_pt.x < roi.max_x) {
      if (src_pt.y > roi.min_y && src_pt.y < roi.max_y) {
        if (src_pt.z > roi.min_z && src_pt.z < roi.max_z) {
          out_cloud->points.push_back(src_pt);
        }
      }
    }
  }
}


int ProcessAllLidarFrame(const std::vector<pcl::PointCloud<LidarPointXYZIRT>> &pcds,
                      const std::vector<Eigen::Matrix4d> &lidar_poses,
                      const Eigen::Matrix4d &calibration_matrix_,
                      const bool &diaplay_mode) {


  pcl::PointCloud<LidarPointXYZIRT>::Ptr cloud(
      new pcl::PointCloud<LidarPointXYZIRT>);
  for (size_t i = 0; i < pcds.size(); i++) {
    *cloud = pcds[i];
    //       (T world<-imu0 * T imu <-lidar).inverse)                                (T world<-imu * T imu <-lidar )
    // T lidar0 <- lidar  =    T lidar0<-world                                  *               T world<-lidar                                                                      
    Eigen::Matrix4d T = (lidar_poses[0] * calibration_matrix_).inverse().eval() * (lidar_poses[i] * calibration_matrix_);
    for (const auto &src_pt : cloud->points) {
      if (!std::isfinite(src_pt.x) || !std::isfinite(src_pt.y) ||
          !std::isfinite(src_pt.z))
        continue;
      Eigen::Vector3d p(src_pt.x, src_pt.y, src_pt.z);
      Eigen::Vector3d p_res;
      p_res = T.block<3, 3>(0, 0) * p + T.block<3, 1>(0, 3);
      pcl::PointXYZI dst_pt;
      dst_pt.x = p_res(0);
      dst_pt.y = p_res(1);
      dst_pt.z = p_res(2);
      dst_pt.intensity = src_pt.intensity;
      if (!all_octree->isVoxelOccupiedAtPoint(dst_pt)) {
        all_octree->addPointToCloud(dst_pt, cloudLidar);
      }
    }
  }

  if (target_vertexBuffer_ != nullptr)
    delete (target_vertexBuffer_);
  if (target_colorBuffer_ != nullptr)
    delete (target_colorBuffer_);

  int pointsNum = cloudLidar->points.size();

  pangolin::GlBuffer *vertexbuffer = new pangolin::GlBuffer(
      pangolin::GlArrayBuffer, pointsNum, GL_FLOAT, 3, GL_DYNAMIC_DRAW);
  pangolin::GlBuffer *colorbuffer = new pangolin::GlBuffer(
      pangolin::GlArrayBuffer, pointsNum, GL_UNSIGNED_BYTE, 3, GL_DYNAMIC_DRAW);

  float *dataUpdate = new float[pointsNum * 3];
  unsigned char *colorUpdate = new unsigned char[pointsNum * 3];
  for (int ipt = 0; ipt < pointsNum; ipt++) {
    Eigen::Vector4d pointPos(cloudLidar->points[ipt].x,
                             cloudLidar->points[ipt].y,
                             cloudLidar->points[ipt].z, 1.0);
    dataUpdate[ipt * 3 + 0] = pointPos.x();
    dataUpdate[ipt * 3 + 1] = pointPos.y();
    dataUpdate[ipt * 3 + 2] = pointPos.z();

    if (diaplay_mode) {

      colorUpdate[ipt * 3 + 0] = static_cast<unsigned char>(0);
      colorUpdate[ipt * 3 + 1] = static_cast<unsigned char>(0);
      colorUpdate[ipt * 3 + 2] = static_cast<unsigned char>(255);
    } else {
      for (int k = 0; k < 3; k++) {
        colorUpdate[ipt * 3 + k] =
            static_cast<unsigned char>(cloudLidar->points[ipt].intensity);
      }
    }
  }

  (vertexbuffer)->Upload(dataUpdate, sizeof(float) * 3 * pointsNum, 0);
  (colorbuffer)->Upload(colorUpdate, sizeof(unsigned char) * 3 * pointsNum, 0);

  target_vertexBuffer_ = vertexbuffer;
  target_colorBuffer_ = colorbuffer;
  int points_size = cloudLidar->points.size();
  cloudLidar->clear();
  all_octree->deleteTree();

  return points_size;
}                

int ProcessLidarFrame(const std::vector<pcl::PointCloud<LidarPointXYZIRT>> &pcds,
                      const std::vector<Eigen::Matrix4d> &lidar_poses,
                      const Eigen::Matrix4d &calibration_matrix_,
                      const bool &diaplay_mode) {
  if(pcds.size() <= 10){
    for (size_t i = 0; i < pcds.size(); i++) {
      Eigen::Matrix4d T = (lidar_poses[0] * calibration_matrix_).inverse().eval() * (lidar_poses[i] * calibration_matrix_);

      for (const auto &src_pt : pcds[i].points) {
        if (!std::isfinite(src_pt.x) || !std::isfinite(src_pt.y) ||
            !std::isfinite(src_pt.z))
          continue;
        pcl::PointXYZI dst_pt;
        Eigen::Vector3d p(src_pt.x, src_pt.y, src_pt.z);
        Eigen::Vector3d p_res;
        p_res = T.block<3, 3>(0, 0) * p + T.block<3, 1>(0, 3);

        dst_pt.x = p_res(0);
        dst_pt.y = p_res(1);
        dst_pt.z = p_res(2);
        dst_pt.intensity = src_pt.intensity;

        double min_x, min_y, min_z, max_x, max_y, max_z;
        all_octree->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
        bool isInBox = (dst_pt.x >= min_x && dst_pt.x <= max_x) && (dst_pt.y >= min_y && dst_pt.y <= max_y) && (dst_pt.z >= min_z && dst_pt.z <= max_z);
        if (!isInBox || !all_octree->isVoxelOccupiedAtPoint(dst_pt))
        {
          all_octree->addPointToCloud(dst_pt, cloudLidar);
        }
      }
    }
  }
  else{
    for (size_t i = pcds.size()-10; i < pcds.size(); i++) {
      Eigen::Matrix4d T = (lidar_poses[0] * calibration_matrix_).inverse().eval() * (lidar_poses[i] * calibration_matrix_);

      for (const auto &src_pt : pcds[i].points) {
        if (!std::isfinite(src_pt.x) || !std::isfinite(src_pt.y) ||
            !std::isfinite(src_pt.z))
          continue;
        pcl::PointXYZI dst_pt;
        Eigen::Vector3d p(src_pt.x, src_pt.y, src_pt.z);
        Eigen::Vector3d p_res;
        p_res = T.block<3, 3>(0, 0) * p + T.block<3, 1>(0, 3);

        dst_pt.x = p_res(0);
        dst_pt.y = p_res(1);
        dst_pt.z = p_res(2);
        dst_pt.intensity = src_pt.intensity;

        double min_x, min_y, min_z, max_x, max_y, max_z;
        all_octree->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
        bool isInBox = (dst_pt.x >= min_x && dst_pt.x <= max_x) && (dst_pt.y >= min_y && dst_pt.y <= max_y) && (dst_pt.z >= min_z && dst_pt.z <= max_z);
        if (!isInBox || !all_octree->isVoxelOccupiedAtPoint(dst_pt))
        {
          all_octree->addPointToCloud(dst_pt, cloudLidar);
        }
      }
    }
  }

  if (target_vertexBuffer_ != nullptr)
    delete (target_vertexBuffer_);
  if (target_colorBuffer_ != nullptr)
    delete (target_colorBuffer_);

  int pointsNum = cloudLidar->points.size();

  pangolin::GlBuffer *vertexbuffer = new pangolin::GlBuffer(
      pangolin::GlArrayBuffer, pointsNum, GL_FLOAT, 3, GL_DYNAMIC_DRAW);
  pangolin::GlBuffer *colorbuffer = new pangolin::GlBuffer(
      pangolin::GlArrayBuffer, pointsNum, GL_UNSIGNED_BYTE, 3, GL_DYNAMIC_DRAW);

  float *dataUpdate = new float[pointsNum * 3];
  unsigned char *colorUpdate = new unsigned char[pointsNum * 3];
  for (int ipt = 0; ipt < pointsNum; ipt++) {
    Eigen::Vector4d pointPos(cloudLidar->points[ipt].x,
                             cloudLidar->points[ipt].y,
                             cloudLidar->points[ipt].z, 1.0);
    dataUpdate[ipt * 3 + 0] = pointPos.x();
    dataUpdate[ipt * 3 + 1] = pointPos.y();
    dataUpdate[ipt * 3 + 2] = pointPos.z();

    if (diaplay_mode) {

      colorUpdate[ipt * 3 + 0] = static_cast<unsigned char>(0);
      colorUpdate[ipt * 3 + 1] = static_cast<unsigned char>(0);
      colorUpdate[ipt * 3 + 2] = static_cast<unsigned char>(255);
    } else {
      for (int k = 0; k < 3; k++) {
        colorUpdate[ipt * 3 + k] =
            static_cast<unsigned char>(cloudLidar->points[ipt].intensity);
      }
    }
  }

  (vertexbuffer)->Upload(dataUpdate, sizeof(float) * 3 * pointsNum, 0);
  (colorbuffer)->Upload(colorUpdate, sizeof(unsigned char) * 3 * pointsNum, 0);

  target_vertexBuffer_ = vertexbuffer;
  target_colorBuffer_ = colorbuffer;
  int points_size = cloudLidar->points.size();
  cloudLidar->clear();
  all_octree->deleteTree();

  return points_size;
}

Eigen::Matrix4d GetDeltaTrans(double R[3], double t[3]) {
    Eigen::Matrix3d deltaR;
    double mat[9];
    // ceres::EulerAnglesToRotationMatrix(R, mat);
    ceres::AngleAxisToRotationMatrix(R, mat);
    deltaR << mat[0], mat[3], mat[6], mat[1], mat[4], mat[7], mat[2], mat[5],
            mat[8];
    // auto deltaR = Eigen::Matrix3d(
    //     Eigen::AngleAxisd(R[2], Eigen::Vector3d::UnitZ()) *
    //     Eigen::AngleAxisd(R[1], Eigen::Vector3d::UnitY()) *
    //     Eigen::AngleAxisd(R[0], Eigen::Vector3d::UnitX()));
    Eigen::Matrix4d deltaT = Eigen::Matrix4d::Identity();
    deltaT.block<3, 3>(0, 0) = deltaR;
    deltaT(0, 3) = t[0];
    deltaT(1, 3) = t[1];
    deltaT(2, 3) = t[2];
    return deltaT;
}

Eigen::Matrix4d AutoCalibration(std::vector<pcl::PointCloud<LidarPointXYZIRT>> &pcds,
                      std::vector<Eigen::Matrix4d> &lidar_poses, Eigen::Matrix4d &calibration_matrix) {
    if(pcds.size() < 500){
      std::cout << "The number of frames is less than 500, please check the data!" << std::endl;
      return calibration_matrix;
    }
    auto time_begin = std::chrono::steady_clock::now();
//    std::vector<pcl::PointCloud<LidarPointXYZIRT>> pcds_new = pcds;
//    std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d>> lidar_poses_new = lidar_poses;

    int turn = 20;
    int window = 10;
    //   Eigen::Matrix4d calibration_matrix = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double, 6, 1> last_deltaT;
    //LoadTimeAndPoes(odom_path, calibration_matrix, lidar_files_, lidar_poses);

    std::vector<int> frm_start_box;
    std::vector<int> frm_step_box;
    std::vector<int> frm_num_box;
    int upper_bound = std::min(int(pcds.size()), 1000);
    int start_step = (upper_bound / 2) / turn - 1;
    for (int i = 0; i < turn; i++) {
        int a = upper_bound / 2 - i * start_step - 1;
        frm_start_box.push_back(a);
        frm_step_box.push_back((upper_bound - a) / window - 1);
        frm_num_box.push_back(window);
    }
    double deltaRPY[3] = {0, 0, 0};
    double deltaT[3] = {0, 0, 0};
    for (int i = 0; i < frm_start_box.size(); i++) {
        std::cout << "\n==>ROUND " << i << std::endl;
        int step = frm_step_box[i];
        int start = frm_start_box[i];
        int frmnum = frm_num_box[i];
        // The hash table of voxel map
        std::unordered_map<VOXEL_LOC, OCTO_TREE *> surf_map, corn_map;
        // build voxel_map
        OCTO_TREE::imu_transmat.clear();
        Eigen::Matrix4d deltaTrans = GetDeltaTrans(deltaRPY, deltaT);
        OCTO_TREE::voxel_windowsize = frmnum;
        int window_size = frmnum;
        for (size_t frmIdx = 0; frmIdx < frmnum; frmIdx++) {
            int real_frmIdx = start + frmIdx * step;

            pcl::PointCloud<LidarPointXYZIRT>::Ptr cloud(
                    new pcl::PointCloud<LidarPointXYZIRT>);
            *cloud = pcds[real_frmIdx];

            pcl::PointCloud<pcl::PointXYZI>::Ptr pl_corn(
                    new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr pl_surf(
                    new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr pl_surf_sharp(
                    new pcl::PointCloud<pcl::PointXYZI>);
            // generate feature points
            // GenFeature feature;


            genPcdFeature(cloud, pl_surf, pl_surf_sharp, pl_corn, Lidar_type, LIDAR_LASER_NUM, MAX_POINT_CLOUD_NUM, SCAN_LINE_CUT, INTENSITY_THRESHOLD);
            Eigen::Matrix4d imu_T = lidar_poses[real_frmIdx] * calibration_matrix;

            Eigen::Matrix4d refined_T = imu_T * deltaTrans;
            OCTO_TREE::imu_transmat.push_back(imu_T);
            if (i < turn / 2)
            {
                cut_voxel(surf_map, pl_surf_sharp, refined_T, 0, frmIdx,
                          window_size + 5);
            } else {
                cut_voxel(surf_map, pl_surf, refined_T, 0, frmIdx, window_size + 5);
            }
            // if (i > turn / 2)
            //     cut_voxel(corn_map, pl_corn, refined_T, 1, frmIdx, window_size +
            //     5);

            // Points in new frame have been distributed in corresponding root node
            // voxel
            // Then continue to cut the root voxel until right size
            for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter) {
                if (iter->second->is2opt) // Sliding window of root voxel should
                    // have points
                {
                    iter->second->root_centors.clear();
                    iter->second->recut(0, frmIdx, iter->second->root_centors);
                }
            }

            for (auto iter = corn_map.begin(); iter != corn_map.end(); ++iter) {
                if (iter->second->is2opt) {
                    iter->second->root_centors.clear();
                    iter->second->recut(0, frmIdx, iter->second->root_centors);
                }
            }
        }
        // display
        // displayVoxelMap(surf_map);
        // optimize delta R, t1, t2
        std::vector<double> cost1;
        if (i < turn / 2) {
            cost1 = optimizeDeltaTrans(surf_map, corn_map, 4, deltaRPY, deltaT);
        } else {
            cost1 = optimizeDeltaTrans(surf_map, corn_map, 2, deltaRPY, deltaT);
        }
        std::cout << "delta rpy: " << deltaRPY[0] / degree_2_radian << " "
                  << deltaRPY[1] / degree_2_radian << " "
                  << deltaRPY[2] / degree_2_radian << std::endl;
        std::cout << "delta T: " << deltaT[0] << " " << deltaT[1] << " "
                  << deltaT[2] << std::endl;

        //  clear tree
        for (auto iter = corn_map.begin(); iter != corn_map.end(); ++iter) {
            clear_tree(iter->second);
        }
        for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter) {
            clear_tree(iter->second);
        }
        std::cout << "Round Finish!\n";
    }
    double bestVal[6];
    bestVal[0] = deltaRPY[0];
    bestVal[1] = deltaRPY[1];
    bestVal[2] = deltaRPY[2];
    bestVal[3] = deltaT[0];
    bestVal[4] = deltaT[1];
    bestVal[5] = deltaT[2];
    auto time_end = std::chrono::steady_clock::now();
    std::cout << "calib cost "
              << std::chrono::duration<double>(time_end - time_begin).count()
              << "s" << std::endl;
    // save refined calib
    // std::string refine_calib_file = "~/refined_calib_imu_to_lidar.txt";
    Eigen::Matrix4d deltaTrans = Eigen::Matrix4d::Identity();
    // SaveStitching(deltaTrans,"before.pcd");
    deltaTrans = GetDeltaTrans(deltaRPY, deltaT);
    // SaveStitching(deltaTrans,"after.pcd");
    std::cout << "delta T is:" << std::endl;
    std::cout << deltaTrans << std::endl;
    Eigen::Matrix4d refined_Tl2i = calibration_matrix * deltaTrans;

    return refined_Tl2i;
}

void ClearPcdAndPose(std::vector<pcl::PointCloud<LidarPointXYZIRT>> &pcds,
                      std::vector<Eigen::Matrix4d> &lidar_poses) {
    pcds.clear();
    lidar_poses.clear();
}

void CalibCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg,
                   const multi_sensor_calibration::CGI610GNSSMsg::ConstPtr& imu_msg) {
    pcl::PointCloud<LidarPointXYZIRT>::Ptr cloud(
      new pcl::PointCloud<LidarPointXYZIRT>);
    pcl::PointCloud<LidarPointXYZIRT>::Ptr filter_cloud_roi(
      new pcl::PointCloud<LidarPointXYZIRT>);
    pcl::fromROSMsg(*lidar_msg, *cloud);

    PointCloudBbox roi;
    roi.max_x = 20;
    roi.min_x = -20;
    roi.max_y = 20;
    roi.min_y = -20;
    roi.max_z = 5;
    roi.min_z = -5;
    PointCloudFilterByROI(cloud, roi, filter_cloud_roi);
    if(pcds.size() < 1000){
      pcds.emplace_back(*filter_cloud_roi);
    }


    Eigen::Affine3d transform_ = Eigen::Affine3d::Identity();
    transform_.rotate(Eigen::AngleAxisd(imu_msg->AngleHeading, Eigen::Vector3d::UnitZ()));
    transform_.rotate(Eigen::AngleAxisd(imu_msg->AnglePitch, Eigen::Vector3d::UnitY()));
    transform_.rotate(Eigen::AngleAxisd(imu_msg->AngleRoll, Eigen::Vector3d::UnitX()));
    transform_.translation() << imu_msg->PosLat, imu_msg->PosLon, imu_msg->PosAlt;
    Eigen::Matrix4d transform_matrix = transform_.matrix();
    

    if(lidar_poses.size() < 1000){
      lidar_poses.emplace_back(transform_matrix);
    }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "lidar_imu_calibration");
  pkg_loc = ros::package::getPath("multi_sensor_calibration");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  string lidar_topic, imu_topic;

  private_nh.param<string>("lidar_topic", lidar_topic, "/lidar_PointCloud2");
  private_nh.param<string>("imu_topic", imu_topic, "/CGI610_GNSS");

  start_origin = true;

  message_filters::Subscriber<multi_sensor_calibration::CGI610GNSSMsg> sub_imu(nh, imu_topic, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_lidar(nh, lidar_topic, 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, multi_sensor_calibration::CGI610GNSSMsg> MySyncPolicy_Calib;
  message_filters::Synchronizer<MySyncPolicy_Calib> sync_object(MySyncPolicy_Calib(10), sub_lidar, sub_imu);
  sync_object.registerCallback(boost::bind(&CalibCallback, _1, _2)); 

  string extrinsic_json = pkg_loc + "/data/extrinsic_lidar2imu.json";
  Eigen::Matrix4d json_param;
  all_octree->setInputCloud(cloudLidar);
  LoadExtrinsic(extrinsic_json, json_param);
  cout << "Loading data completed!" << endl;
  CalibrationInit(json_param);
  const int width = 1920, height = 1280;
  pangolin::CreateWindowAndBind("lidar2imu player", width, height);

  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
  glDepthFunc(GL_LESS);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      // pangolin::ModelViewLookAt(0, 0, 100, 0, 0, 0, 0.0, 1.0, 0.0));
      pangolin::ModelViewLookAt(0, 100, 0, 0, 0, 0, 0.0, 0.0, 1.0));
  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(150),
                                         1.0, -1.0 * width / height)
                              .SetHandler(new pangolin::Handler3D(s_cam));
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  pangolin::OpenGlMatrix Twc; // camera to world
  Twc.SetIdentity();

  // control panel
  pangolin::CreatePanel("cp").SetBounds(pangolin::Attach::Pix(30), 1.0, 0.0,
                                        pangolin::Attach::Pix(150));
  pangolin::Var<bool> displayMode("cp.Intensity Color", display_mode_,
                                  true); // logscale
  pangolin::Var<int> pointSize("cp.Point Size", 2, 0, 8);
  pangolin::Var<double> degreeStep("cp.deg step", cali_scale_degree_, 0,
                                   1); // logscale
  pangolin::Var<double> tStep("cp.t step(cm)", 6, 0, 15);

  pangolin::Var<bool> addXdegree("cp.+ x degree", false, false);
  pangolin::Var<bool> minusXdegree("cp.- x degree", false, false);
  pangolin::Var<bool> addYdegree("cp.+ y degree", false, false);
  pangolin::Var<bool> minusYdegree("cp.- y degree", false, false);
  pangolin::Var<bool> addZdegree("cp.+ z degree", false, false);
  pangolin::Var<bool> minusZdegree("cp.- z degree", false, false);
  pangolin::Var<bool> addXtrans("cp.+ x trans", false, false);
  pangolin::Var<bool> minusXtrans("cp.- x trans", false, false);
  pangolin::Var<bool> addYtrans("cp.+ y trans", false, false);
  pangolin::Var<bool> minusYtrans("cp.- y trans", false, false);
  pangolin::Var<bool> addZtrans("cp.+ z trans", false, false);
  pangolin::Var<bool> minusZtrans("cp.- z trans", false, false);

  pangolin::Var<bool> autocalib("cp.Auto Calibration", false, false);
  pangolin::Var<bool> showallpointcloud("cp.Show All Point", false, false);

  pangolin::Var<bool> cleardata("cp.Clear Data", false, false);

  pangolin::Var<bool> resetButton("cp.Reset", false, false);
  pangolin::Var<bool> saveImg("cp.Save Result", false, false);

  std::vector<pangolin::Var<bool>> mat_calib_box;
  mat_calib_box.push_back(addXdegree);
  mat_calib_box.push_back(minusXdegree);
  mat_calib_box.push_back(addYdegree);
  mat_calib_box.push_back(minusYdegree);
  mat_calib_box.push_back(addZdegree);
  mat_calib_box.push_back(minusZdegree);
  mat_calib_box.push_back(addXtrans);
  mat_calib_box.push_back(minusXtrans);
  mat_calib_box.push_back(addYtrans);
  mat_calib_box.push_back(minusYtrans);
  mat_calib_box.push_back(addZtrans);
  mat_calib_box.push_back(minusZtrans);

  int frame_num = 0;
  int points_size = 0;

  std::cout << "\n=>START\n";
  while (!pangolin::ShouldQuit()) {
    ros::spinOnce();
    points_size = ProcessLidarFrame(pcds, lidar_poses, calibration_matrix_, display_mode_);
    s_cam.Follow(Twc);
    d_cam.Activate(s_cam);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (displayMode) {
      if (display_mode_ == false) {
        display_mode_ = true;
        // ProcessTargetFrame(target_cloud, display_mode_);
        // ProcessSourceFrame(source_cloud, calibration_matrix_, display_mode_);
        points_size = ProcessLidarFrame(pcds, lidar_poses, calibration_matrix_,
                                        display_mode_);
      }
    } else {
      if (display_mode_ == true) {
        display_mode_ = false;
        // ProcessTargetFrame(target_cloud, display_mode_);
        // ProcessSourceFrame(source_cloud, calibration_matrix_, display_mode_);
        points_size = ProcessLidarFrame(pcds, lidar_poses, calibration_matrix_,
                                        display_mode_);
      }
    }
    if (pointSize.GuiChanged()) {
      point_size_ = pointSize.Get();
      std::cout << "Point size changed to " << point_size_ << " degree\n";
    }

    if (degreeStep.GuiChanged()) {
      cali_scale_degree_ = degreeStep.Get();
      CalibrationScaleChange();
      std::cout << "Degree calib scale changed to " << cali_scale_degree_
                << " degree\n";
    }
    if (tStep.GuiChanged()) {
      cali_scale_trans_ = tStep.Get() / 100.0;
      CalibrationScaleChange();
      std::cout << "Trans calib scale changed to " << cali_scale_trans_ * 100
                << " cm\n";
    }
    for (int i = 0; i < 12; i++) {
      if (pangolin::Pushed(mat_calib_box[i])) {
        calibration_matrix_ = calibration_matrix_ * modification_list_[i];
        // ProcessSourceFrame(source_cloud, calibration_matrix_, display_mode_);
        points_size = ProcessLidarFrame(pcds, lidar_poses, calibration_matrix_,
                                        display_mode_);
        std::cout << "Changed!\n";
      }
    }

    if (pangolin::Pushed(autocalib)) {
      calibration_matrix_ = AutoCalibration(pcds, lidar_poses, calibration_matrix_);
      std::cout << "Auto Calib Done!\n";
    }

    if (pangolin::Pushed(showallpointcloud)) {
      points_size = ProcessAllLidarFrame(pcds, lidar_poses, calibration_matrix_,
                                        display_mode_);
          // draw lidar points
      glDisable(GL_LIGHTING);
      glPointSize(point_size_);
      // draw target lidar points
      target_colorBuffer_->Bind();
      glColorPointer(target_colorBuffer_->count_per_element,
                    target_colorBuffer_->datatype, 0, 0);
      glEnableClientState(GL_COLOR_ARRAY);
      target_vertexBuffer_->Bind();
      glVertexPointer(target_vertexBuffer_->count_per_element,
                      target_vertexBuffer_->datatype, 0, 0);
      glEnableClientState(GL_VERTEX_ARRAY);
      glDrawArrays(GL_POINTS, 0, points_size);
      glDisableClientState(GL_VERTEX_ARRAY);
      target_vertexBuffer_->Unbind();
      glDisableClientState(GL_COLOR_ARRAY);
      target_colorBuffer_->Unbind();
      pangolin::FinishFrame();
      usleep(100);
      glFinish();
      std::cout << "Show all lidar points!\n";
      while (1)
      {
        s_cam.Follow(Twc);
        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if(pangolin::Pushed(showallpointcloud)){
          break;
        }
                  // draw lidar points
        glDisable(GL_LIGHTING);
        glPointSize(point_size_);
        // draw target lidar points
        target_colorBuffer_->Bind();
        glColorPointer(target_colorBuffer_->count_per_element,
                      target_colorBuffer_->datatype, 0, 0);
        glEnableClientState(GL_COLOR_ARRAY);
        target_vertexBuffer_->Bind();
        glVertexPointer(target_vertexBuffer_->count_per_element,
                        target_vertexBuffer_->datatype, 0, 0);
        glEnableClientState(GL_VERTEX_ARRAY);
        glDrawArrays(GL_POINTS, 0, points_size);
        glDisableClientState(GL_VERTEX_ARRAY);
        target_vertexBuffer_->Unbind();
        glDisableClientState(GL_COLOR_ARRAY);
        target_colorBuffer_->Unbind();
        pangolin::FinishFrame();
        usleep(100);
        glFinish();
      }
      
    }

    if (pangolin::Pushed(cleardata)) {
      ClearPcdAndPose(pcds, lidar_poses);
      std::cout << "Clear!\n";
    }

    if (pangolin::Pushed(resetButton)) {
      calibration_matrix_ = orign_calibration_matrix_;
      // ProcessSourceFrame(source_cloud, calibration_matrix_, display_mode_);
      points_size = ProcessLidarFrame(pcds, lidar_poses, calibration_matrix_,
                                      display_mode_);
      std::cout << "Reset!\n";
    }
    if (pangolin::Pushed(saveImg)) {
      saveResult(frame_num);
      std::cout << "\n==>Save Result " << frame_num << std::endl;
      Eigen::Matrix4d transform = calibration_matrix_;
      std::cout << "Transfromation Matrix:\n" << transform << std::endl;
      frame_num++;
    }

    if (kbhit()) {
      int c = getchar();
      if (ManualCalibration(c)) {
        Eigen::Matrix4d transform = calibration_matrix_;
        // ProcessSourceFrame(source_cloud, calibration_matrix_, display_mode_);
        points_size = ProcessLidarFrame(pcds, lidar_poses, calibration_matrix_,
                                        display_mode_);
        std::cout << "\nTransfromation Matrix:\n" << transform << std::endl;
      }
    }

    // draw lidar points
    glDisable(GL_LIGHTING);
    glPointSize(point_size_);
    // draw target lidar points
    target_colorBuffer_->Bind();
    glColorPointer(target_colorBuffer_->count_per_element,
                   target_colorBuffer_->datatype, 0, 0);
    glEnableClientState(GL_COLOR_ARRAY);
    target_vertexBuffer_->Bind();
    glVertexPointer(target_vertexBuffer_->count_per_element,
                    target_vertexBuffer_->datatype, 0, 0);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_POINTS, 0, points_size);
    glDisableClientState(GL_VERTEX_ARRAY);
    target_vertexBuffer_->Unbind();
    glDisableClientState(GL_COLOR_ARRAY);
    target_colorBuffer_->Unbind();
    pangolin::FinishFrame();
    usleep(100);
    glFinish();
  }

  // delete[] imageArray;
  ros::shutdown();
  Eigen::Matrix4d transform = calibration_matrix_;
  cout << "\nFinal Transfromation Matrix:\n" << transform << std::endl;

  return 0;
}