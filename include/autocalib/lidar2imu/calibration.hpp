/*
* Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
* Limited. All rights reserved.
* Yan Guohang <yanguohang@pjlab.org.cn>
*/

/*
* Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
* Limited. All rights reserved.
* Yan Guohang <yanguohang@pjlab.org.cn>
*/
#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree2buf_base.h>
#include <pcl/octree/impl/octree2buf_base.hpp>
#include <queue>
#include <vector>
#include "common/Lidar_parser_base.h"

class Calibrator
{
public:
    Calibrator();
    ~Calibrator();

    // load data
    void LoadTimeAndPoes(const std::string &filename, const Eigen::Matrix4d &Tl2i,
                         std::vector<std::string> &lidarTimes,
                         std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d>>  &lidarPoses);

    Eigen::Matrix4d GetDeltaTrans(double R[3], double t[3]);
    Eigen::Matrix4d Calibration(const std::vector<pcl::PointCloud<LidarPointXYZIRT>> lidar_cloud, const std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d>> lidar_pos, const Eigen::Matrix4d init_Tl2i, std::vector<std::vector<double>>& cost);
    void SavePcd(const Eigen::Matrix4d transform, const std::vector<pcl::PointCloud<LidarPointXYZIRT>> lidar_cloud, const std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d>> lidar_pos, const std::string calib_pcd);
    int Lidar_type= 1 ;
    int LIDAR_LASER_NUM=5;
    int MAX_POINT_CLOUD_NUM=2000000;
    int SCAN_LINE_CUT=6;
    int INTENSITY_THRESHOLD=35;

private:
    int turn_ = 20;
    int window_ = 10;
    std::vector<std::string> lidar_files_;
    std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d>> lidar_poses_;
    // std::vector<pcl::PointCloud<LidarPointXYZIRT>> pcd_seq_;
    double degree_2_radian = 0.017453293;
    std::string lidar_path_;
};
