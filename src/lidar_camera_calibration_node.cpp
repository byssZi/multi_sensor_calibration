#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pangolin/pangolin.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <Eigen/Core>
#include <iostream>
#include <string>

#include "extrinsic_param.hpp"
#include "intrinsic_param.hpp"
#include "projector_lidar.hpp"

using namespace std;

#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049
#define APPLY_COLOR_TO_LIDAR_INTENSITY // to set intensity colored or not

string pkg_loc;

ros::Subscriber lidar_sub;
ros::Subscriber camera_sub;

pcl::PointCloud<pcl::PointXYZI>::Ptr lidar(new pcl::PointCloud<pcl::PointXYZI>());
cv::Mat img;

double cali_scale_degree_ = 0.3;
double cali_scale_trans_ = 0.06;
double cali_scale_fxfy_ = 1.005;
static Eigen::Matrix4d calibration_matrix_ = Eigen::Matrix4d::Identity();
static Eigen::Matrix4d orign_calibration_matrix_ = Eigen::Matrix4d::Identity();
static Eigen::Matrix3d intrinsic_matrix_ = Eigen::Matrix3d::Identity();
static Eigen::Matrix3d orign_intrinsic_matrix_ = Eigen::Matrix3d::Identity();
std::vector<float> distortions_;
std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Vector4d>> modification_list_(12);
bool display_mode_ = false;
bool filter_mode_ = false;

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
  modification_list_.resize(12);
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

void saveResult(const cv::Mat &calib_img, const int &frame_id) {
  std::string file_name = pkg_loc + "/data/lidar2camera_extrinsic_" + std::to_string(frame_id) + ".txt";
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
  fCalib << "\nIntrinsic:" << std::endl;
  fCalib << intrinsic_matrix_(0, 0) << " " << intrinsic_matrix_(0, 1) << " "
         << intrinsic_matrix_(0, 2) << "\n"
         << intrinsic_matrix_(1, 0) << " " << intrinsic_matrix_(1, 1) << " "
         << intrinsic_matrix_(1, 2) << "\n"
         << intrinsic_matrix_(2, 0) << " " << intrinsic_matrix_(2, 1) << " "
         << intrinsic_matrix_(2, 2) << std::endl;

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
  fCalib << "\nIntrinsic:" << std::endl;
  fCalib << "[" << intrinsic_matrix_(0, 0) << "," << intrinsic_matrix_(0, 1)
         << "," << intrinsic_matrix_(0, 2) << "],"
         << "[" << intrinsic_matrix_(1, 0) << "," << intrinsic_matrix_(1, 1)
         << "," << intrinsic_matrix_(1, 2) << "],"
         << "[" << intrinsic_matrix_(2, 0) << "," << intrinsic_matrix_(2, 1)
         << "," << intrinsic_matrix_(2, 2) << "]" << std::endl;

  fCalib << "\nDistortion:" << std::endl;
  fCalib << "[";
  for (size_t i = 0; i < distortions_.size(); i++) {
    fCalib << distortions_[i];
    if (i == distortions_.size() - 1)
      continue;
    fCalib << ",";
  }
  fCalib << "]";
  fCalib.close();

  std::string img_name = pkg_loc + "/data/calibimg_" + std::to_string(frame_id) + ".jpg";
  cv::imwrite(img_name, calib_img);
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
  // adjust fx, fy
  if (key_input == 'u') {
    intrinsic_matrix_(0, 0) *= cali_scale_fxfy_;
    std::cout << "fx changed to " << intrinsic_matrix_(0, 0) << std::endl;
  }
  if (key_input == 'j') {
    intrinsic_matrix_(0, 0) /= cali_scale_fxfy_;
    std::cout << "fx changed to " << intrinsic_matrix_(0, 0) << std::endl;
  }
  if (key_input == 'i') {
    intrinsic_matrix_(1, 1) *= cali_scale_fxfy_;
    std::cout << "fy changed to " << intrinsic_matrix_(1, 1) << std::endl;
  }
  if (key_input == 'k') {
    intrinsic_matrix_(1, 1) /= cali_scale_fxfy_;
    std::cout << "fy changed to " << intrinsic_matrix_(1, 1) << std::endl;
  }
  return real_hit;
}

void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg)
{
    lidar->clear();
    pcl::fromROSMsg(*lidar_msg, *lidar);
}

void camera_callback(const sensor_msgs::Image::ConstPtr& camera_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
      cv_ptr = cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  img  = cv_ptr->image;
} 

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "lidar_camera_calibration");
  pkg_loc = ros::package::getPath("multi_sensor_calibration");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  string lidar_topic, camera_topic;

  private_nh.param<string>("lidar_topic", lidar_topic, "/lidar_PointCloud2");
  private_nh.param<string>("camera_topic", camera_topic, "/camera/image");

  lidar_sub = nh.subscribe(lidar_topic, 1, lidar_callback);
  camera_sub = nh.subscribe(camera_topic, 1, camera_callback);

  string extrinsic_json = pkg_loc + "/data/extrinsic_lidar2camera.json";
  string intrinsic_json = pkg_loc + "/data/intrinsic_camera.json";
  // load intrinsic
  Eigen::Matrix3d K;
  std::vector<double> dist;
  int width;
  int height;
  LoadIntrinsic(intrinsic_json, K, dist, width, height);
  for (size_t i = 0; i < dist.size(); i++) {
    distortions_.push_back(dist[i]);
  }

  intrinsic_matrix_ = K;
  orign_intrinsic_matrix_ = intrinsic_matrix_;
  std::cout << "intrinsic:\n"
            << K(0, 0) << " " << K(0, 1) << " " << K(0, 2) << "\n"
            << K(1, 0) << " " << K(1, 1) << " " << K(1, 2) << "\n"
            << K(2, 0) << " " << K(2, 1) << " " << K(2, 2) << "\n";
  std::cout << "dist:\n" << dist[0] << " " << dist[1] << "\n";

  // load extrinsic
  Eigen::Matrix4d json_param;
  LoadExtrinsic(extrinsic_json, json_param);

  cout << "Loading data completed!" << endl;
  CalibrationInit(json_param);

  std::cout << __LINE__ << "\n";

  // view
  std::cout << "width:" << width << " , height:" << height << std::endl;
  // const int width = 1920, height = 1200;
  pangolin::CreateWindowAndBind("lidar2camera player", width, height);

  glEnable(GL_DEPTH_TEST);
  // glDepthMask(GL_TRUE);
  // glDepthFunc(GL_LESS);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, 0, 100, 0, 0, 0, 0.0, 1.0, 0.0));

  pangolin::View &project_image =
      pangolin::Display("project")
          .SetBounds(0.0, 1.0, pangolin::Attach::Pix(150), 1.0,
                     -1.0 * (width - 200) / height)
          .SetLock(pangolin::LockLeft, pangolin::LockTop);

  unsigned char *imageArray = new unsigned char[3 * width * height];
  pangolin::GlTexture imageTexture(width, height, GL_RGB, false, 0, GL_RGB,
                                   GL_UNSIGNED_BYTE);

  // control panel
  pangolin::CreatePanel("cp").SetBounds(pangolin::Attach::Pix(30), 1.0, 0.0,
                                        pangolin::Attach::Pix(150));
  pangolin::Var<bool> displayMode("cp.Intensity Color", false,
                                  true);                            // logscale
  pangolin::Var<bool> filterMode("cp.Overlap Filter", false, true); // logscale
  pangolin::Var<double> degreeStep("cp.deg step", 0.3, 0, 1);       // logscale
  pangolin::Var<double> tStep("cp.t step(cm)", 6, 0, 15);
  pangolin::Var<double> fxfyScale("cp.fxfy scale", 1.005, 1, 1.1);
  pangolin::Var<int> pointSize("cp.point size", 3, 1, 5);

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

  pangolin::Var<bool> addFx("cp.+ fx", false, false);
  pangolin::Var<bool> minusFx("cp.- fx", false, false);
  pangolin::Var<bool> addFy("cp.+ fy", false, false);
  pangolin::Var<bool> minusFy("cp.- fy", false, false);

  pangolin::Var<bool> resetButton("cp.Reset", false, false);
  pangolin::Var<bool> saveImg("cp.Save Image", false, false);

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
  Projector projector;

  std::cout << "\n=>START\n";
  while (!pangolin::ShouldQuit()) {
    ros::spinOnce();
    if(!img.empty()){
      pcl::PointCloud<pcl::PointXYZI> pcd = *lidar;
      projector.loadPointCloud(pcd);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      cv::Mat current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist, calibration_matrix_);
      if (displayMode) {
        if (display_mode_ == false) {
          projector.setDisplayMode(true);
          current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_,
                                                      dist, calibration_matrix_);
          display_mode_ = true;
        }
      } else {
        if (display_mode_ == true) {
          projector.setDisplayMode(false);
          current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_,
                                                      dist, calibration_matrix_);
          display_mode_ = false;
        }
      }

      if (filterMode) {
        if (filter_mode_ == false) {
          projector.setFilterMode(true);
          current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_,
                                                      dist, calibration_matrix_);
          filter_mode_ = true;
        }
      } else {
        if (filter_mode_ == true) {
          projector.setFilterMode(false);
          current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_,
                                                      dist, calibration_matrix_);
          filter_mode_ = false;
        }
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
      if (fxfyScale.GuiChanged()) {
        cali_scale_fxfy_ = fxfyScale.Get();
        std::cout << "fxfy calib scale changed to " << cali_scale_fxfy_
                  << std::endl;
      }
      if (pointSize.GuiChanged()) {
        int ptsize = pointSize.Get();
        projector.setPointSize(ptsize);
        current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
                                                    calibration_matrix_);
        std::cout << "point size changed to " << ptsize << std::endl;
      }
      for (int i = 0; i < 12; i++) {
        if (pangolin::Pushed(mat_calib_box[i])) {
          calibration_matrix_ = calibration_matrix_ * modification_list_[i];
          std::cout << "Changed!\n";
          current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_,
                                                      dist, calibration_matrix_);
        }
      }

      if (pangolin::Pushed(addFx)) {
        intrinsic_matrix_(0, 0) *= cali_scale_fxfy_;
        current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
                                                    calibration_matrix_);
        std::cout << "fx changed to " << intrinsic_matrix_(0, 0) << std::endl;
      }
      if (pangolin::Pushed(minusFx)) {
        intrinsic_matrix_(0, 0) /= cali_scale_fxfy_;
        current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
                                                    calibration_matrix_);
        std::cout << "fx changed to " << intrinsic_matrix_(0, 0) << std::endl;
      }
      if (pangolin::Pushed(addFy)) {
        intrinsic_matrix_(1, 1) *= cali_scale_fxfy_;
        current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
                                                    calibration_matrix_);
        std::cout << "fy changed to " << intrinsic_matrix_(1, 1) << std::endl;
      }
      if (pangolin::Pushed(minusFy)) {
        intrinsic_matrix_(1, 1) /= cali_scale_fxfy_;
        current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
                                                    calibration_matrix_);
        std::cout << "fy changed to " << intrinsic_matrix_(1, 1) << std::endl;
      }

      if (pangolin::Pushed(resetButton)) {
        calibration_matrix_ = orign_calibration_matrix_;
        intrinsic_matrix_ = orign_intrinsic_matrix_;
        current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
                                                    calibration_matrix_);
        std::cout << "Reset!\n";
      }
      if (pangolin::Pushed(saveImg)) {
        saveResult(current_frame, frame_num);
        std::cout << "\n==>Save Result " << frame_num << std::endl;
        Eigen::Matrix4d transform = calibration_matrix_;
        cout << "Transfromation Matrix:\n" << transform << std::endl;
        frame_num++;
      }

      if (kbhit()) {
        int c = getchar();
        if (ManualCalibration(c)) {
          Eigen::Matrix4d transform = calibration_matrix_;
          cout << "\nTransfromation Matrix:\n" << transform << std::endl;
        }
        current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
                                                    calibration_matrix_);
      }

      imageArray = current_frame.data;
      imageTexture.Upload(imageArray, GL_BGR, GL_UNSIGNED_BYTE);

      project_image.Activate();
      glColor3f(1.0, 1.0, 1.0);
      imageTexture.RenderToViewportFlipY();

      pangolin::FinishFrame();
      usleep(100);
      glFinish();
      }
  }

  // delete[] imageArray;

  Eigen::Matrix4d transform = calibration_matrix_;
  cout << "\nFinal Transfromation Matrix:\n" << transform << std::endl;

  return 0;
}
