#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/registration/ndt.h>

#include <pangolin/pangolin.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <Eigen/Core>
#include <iostream>
#include <string>

#include "extrinsic_param.hpp"

using namespace std;

#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049
#define MAX_RADAR_TIME_GAP 15 * 1e6
// #define APPLY_COLOR_TO_LIDAR_INTENSITY  // to set intensity colored or not

string pkg_loc;

ros::Subscriber target_lidar_sub;
ros::Subscriber source_lidar_sub;

pcl::PointCloud<pcl::PointXYZI>::Ptr target_lidar(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr source_lidar(new pcl::PointCloud<pcl::PointXYZI>());

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

struct RGB {
  unsigned char r;
  unsigned char g;
  unsigned char b;
};

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
  std::string file_name = pkg_loc + "/data/lidar2lidar_extrinsic_" + std::to_string(frame_id) + ".txt";
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

Eigen::Matrix4d AutoCalibration(const pcl::PointCloud<pcl::PointXYZI>::Ptr TargetLidar, const pcl::PointCloud<pcl::PointXYZI>::Ptr SourceLidar) {
  // 创建一个新的点云对象，类型为 PointXYZ
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
    
  // 调整输出点云的大小以匹配输入点云的大小
  cloud_target->width = TargetLidar->width;
  cloud_target->height = TargetLidar->height;
  cloud_target->is_dense = TargetLidar->is_dense;
  cloud_target->points.resize(TargetLidar->width * TargetLidar->height);
    
  // 遍历输入点云，复制坐标到输出点云
  for (size_t i = 0; i < TargetLidar->points.size(); ++i) {
    cloud_target->points[i].x = TargetLidar->points[i].x;
    cloud_target->points[i].y = TargetLidar->points[i].y;
    cloud_target->points[i].z = TargetLidar->points[i].z;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);

    // 调整输出点云的大小以匹配输入点云的大小
  cloud_source->width = SourceLidar->width;
  cloud_source->height = SourceLidar->height;
  cloud_source->is_dense = SourceLidar->is_dense;
  cloud_source->points.resize(SourceLidar->width * SourceLidar->height);
    
  // 遍历输入点云，复制坐标到输出点云
  for (size_t i = 0; i < SourceLidar->points.size(); ++i) {
    cloud_source->points[i].x = SourceLidar->points[i].x;
    cloud_source->points[i].y = SourceLidar->points[i].y;
    cloud_source->points[i].z = SourceLidar->points[i].z;
  }

 // 移除无效的点
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_target, *cloud_target, indices);
  pcl::removeNaNFromPointCloud(*cloud_source, *cloud_source, indices);

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setTransformationEpsilon(0.1);
  ndt.setStepSize(0.1);
  ndt.setResolution(0.5);

  ndt.setMaximumIterations(100);
  ndt.setInputSource(cloud_source);
  ndt.setInputTarget(cloud_target);

  pcl::PointCloud<pcl::PointXYZ> Final;
  ndt.align(Final);

  if (ndt.hasConverged()) {
    std::cout << "NDT has converged, score is " << ndt.getFitnessScore() << std::endl;
  } else {
    std::cout << "NDT did not converge." << std::endl;
  }
  return ndt.getFinalTransformation().cast<double>();
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

void ProcessTargetFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloudLidar,
                        const bool &diaplay_mode) {
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
      RGB colorFake = GreyToColorMix(cloudLidar->points[ipt].intensity);
      colorUpdate[ipt * 3 + 0] = static_cast<unsigned char>(colorFake.r);
      colorUpdate[ipt * 3 + 1] = static_cast<unsigned char>(colorFake.g);
      colorUpdate[ipt * 3 + 2] = static_cast<unsigned char>(colorFake.b);
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
}

void ProcessSourceFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloudLidar,
                        const Eigen::Matrix4d &extrinsic,
                        const bool &diaplay_mode) {
  if (source_vertexBuffer_ != nullptr)
    delete (source_vertexBuffer_);
  if (source_colorBuffer_ != nullptr)
    delete (source_colorBuffer_);
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
    Eigen::Vector4d trans_p = extrinsic * pointPos;
    dataUpdate[ipt * 3 + 0] = trans_p.x();
    dataUpdate[ipt * 3 + 1] = trans_p.y();
    dataUpdate[ipt * 3 + 2] = trans_p.z();

    if (diaplay_mode) {
      RGB colorFake = GreyToColorMix(cloudLidar->points[ipt].intensity);
      colorUpdate[ipt * 3 + 0] = static_cast<unsigned char>(colorFake.r);
      colorUpdate[ipt * 3 + 1] = static_cast<unsigned char>(colorFake.b);
      colorUpdate[ipt * 3 + 2] = static_cast<unsigned char>(colorFake.g);
    } else {
      // red
      colorUpdate[ipt * 3 + 0] =
          static_cast<unsigned char>(cloudLidar->points[ipt].intensity);
      colorUpdate[ipt * 3 + 1] = 0;
      colorUpdate[ipt * 3 + 2] = 0;
    }
  }

  (vertexbuffer)->Upload(dataUpdate, sizeof(float) * 3 * pointsNum, 0);
  (colorbuffer)->Upload(colorUpdate, sizeof(unsigned char) * 3 * pointsNum, 0);

  source_vertexBuffer_ = vertexbuffer;
  source_colorBuffer_ = colorbuffer;
}

void target_lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& target_lidar_msg)
{
    target_lidar->clear();
    pcl::fromROSMsg(*target_lidar_msg, *target_lidar);
}

void source_lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& source_lidar_msg)
{
    source_lidar->clear();
    pcl::fromROSMsg(*source_lidar_msg, *source_lidar);  
} 

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_lidar_calibration");
  pkg_loc = ros::package::getPath("multi_sensor_calibration");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  string target_lidar_topic, source_lidar_topic;

  private_nh.param<string>("target_lidar_topic", target_lidar_topic, "/target_lidar_PointCloud2");
  private_nh.param<string>("source_lidar_topic", source_lidar_topic, "/source_lidar_PointCloud2");

  target_lidar_sub = nh.subscribe(target_lidar_topic, 1, target_lidar_callback);
  source_lidar_sub = nh.subscribe(source_lidar_topic, 1, source_lidar_callback);

  string extrinsic_json = pkg_loc + "/data/extrinsic_lidar2lidar.json";

  // load extrinsic
  Eigen::Matrix4d json_param;
  LoadExtrinsic(extrinsic_json, json_param);
  std::cout << "lidar to lidar extrinsic:\n" << json_param << std::endl;

  cout << "Loading data completed!" << endl;
  CalibrationInit(json_param);
  const int width = 1920, height = 1280;
  pangolin::CreateWindowAndBind("lidar2lidar player", width, height);

  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
  glDepthFunc(GL_LESS);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      // pangolin::ModelViewLookAt(0, 0, 100, 0, 0, 0, 0.0, 1.0, 0.0));
      pangolin::ModelViewLookAt(-100, 0, 0, 0, 0, 0, 0.707, 0.0, 0.707));
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

  pangolin::Var<bool> auto_calibrate("cp. ndt_auto_calibrate", false, false);

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

  std::cout << "\n=>START\n";
  while (!pangolin::ShouldQuit()) {
    ros::spinOnce();
    int source_lidar_ptsize = source_lidar->points.size();
    int target_lidar_ptsize = target_lidar->points.size();
    ProcessTargetFrame(target_lidar, display_mode_);
    ProcessSourceFrame(source_lidar, calibration_matrix_, display_mode_);
    s_cam.Follow(Twc);
    d_cam.Activate(s_cam);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (displayMode) {
      if (display_mode_ == false) {
        display_mode_ = true;
        ProcessTargetFrame(target_lidar, display_mode_);
        ProcessSourceFrame(source_lidar, calibration_matrix_, display_mode_);
      }
    } else {
      if (display_mode_ == true) {
        display_mode_ = false;
        ProcessTargetFrame(target_lidar, display_mode_);
        ProcessSourceFrame(source_lidar, calibration_matrix_, display_mode_);
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
        ProcessSourceFrame(source_lidar, calibration_matrix_, display_mode_);
        cout << "\nTransfromation Matrix:\n" << calibration_matrix_ << std::endl;
      }
    }

    if (pangolin::Pushed(auto_calibrate)) {    
      Eigen::Matrix4d matrix_trans = AutoCalibration(target_lidar, source_lidar);
      calibration_matrix_ = matrix_trans * calibration_matrix_;
      ProcessSourceFrame(source_lidar, calibration_matrix_, display_mode_);
      cout << "\nTransfromation Matrix:\n" << calibration_matrix_ << std::endl;
    }

    if (pangolin::Pushed(resetButton)) {
      calibration_matrix_ = orign_calibration_matrix_;
      ProcessSourceFrame(source_lidar, calibration_matrix_, display_mode_);
      std::cout << "Reset!\n";
    }
    if (pangolin::Pushed(saveImg)) {
      saveResult(frame_num);
      std::cout << "\n==>Save Result " << frame_num << std::endl;
      Eigen::Matrix4d transform = calibration_matrix_;
      cout << "Transfromation Matrix:\n" << transform << std::endl;
      frame_num++;
    }

    if (kbhit()) {
      int c = getchar();
      if (ManualCalibration(c)) {
        Eigen::Matrix4d transform = calibration_matrix_;
        ProcessSourceFrame(source_lidar, calibration_matrix_, display_mode_);
        cout << "\nTransfromation Matrix:\n" << transform << std::endl;
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
    glDrawArrays(GL_POINTS, 0, target_lidar_ptsize);
    glDisableClientState(GL_VERTEX_ARRAY);
    target_vertexBuffer_->Unbind();
    glDisableClientState(GL_COLOR_ARRAY);
    target_colorBuffer_->Unbind();

    // draw source lidar points
    source_colorBuffer_->Bind();
    glColorPointer(source_colorBuffer_->count_per_element,
                   source_colorBuffer_->datatype, 0, 0);
    glEnableClientState(GL_COLOR_ARRAY);
    source_vertexBuffer_->Bind();
    glVertexPointer(source_vertexBuffer_->count_per_element,
                    source_vertexBuffer_->datatype, 0, 0);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_POINTS, 0, source_lidar_ptsize);
    glDisableClientState(GL_VERTEX_ARRAY);
    source_vertexBuffer_->Unbind();
    glDisableClientState(GL_COLOR_ARRAY);
    source_colorBuffer_->Unbind();

    pangolin::FinishFrame();
    usleep(100);
    glFinish();
  }

  // delete[] imageArray;

  Eigen::Matrix4d transform = calibration_matrix_;
  cout << "\nFinal Transfromation Matrix:\n" << transform << std::endl;

  return 0;
}