#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pangolin/pangolin.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <fstream>
#include <jsoncpp/json/json.h>

#include <Eigen/Core>
#include <iostream>
#include <string>

#include "autocalib/camera_intrinsic/cam_intrinsic_calib.hpp"
#include "intrinsic_camera_utility.hpp"

using namespace std;
#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049

string pkg_loc;

ros::Subscriber camera_sub;
cv::Mat img, global_K, global_D;

cv::Mat ProjectToImage(cv::Mat img){
  
  cv::Mat outImg = img.clone();
  cv::cvtColor(outImg, outImg, cv::COLOR_BGR2RGB);
  return outImg;
}

void saveCalibrationResults(const cv::Mat& K, const cv::Mat& distortionParams, const int &frame_id) {
  // 打开文件
  std::string file_name = pkg_loc + "/data/camera_intrinsic_calibration/camera_intrinsic_" + std::to_string(frame_id) + ".txt";
  std::ofstream outFile(file_name);
  if (!outFile.is_open()) {
      std::cerr << "Failed to open file: " << file_name << std::endl;
      return;
  }

  // 打印内参矩阵到终端
  std::cout << "Camera Intrinsic Matrix (K):" << std::endl;
  std::cout << K << std::endl;

  // 打印畸变系数到终端
  std::cout << "Distortion Coefficients:" << std::endl;
  std::cout << distortionParams << std::endl;

  // 写入内参矩阵到文件
  outFile << "Camera Intrinsic Matrix (K):" << std::endl;

  outFile << K.at<float>(0, 0) << " " << K.at<float>(0, 1) << " " << K.at<float>(0, 2) << std::endl;
  outFile << K.at<float>(1, 0) << " " << K.at<float>(1, 1) << " " << K.at<float>(1, 2) << std::endl;
  outFile << K.at<float>(2, 0) << " " << K.at<float>(2, 1) << " " << K.at<float>(2, 2) << std::endl;

  // 写入畸变系数到文件
  outFile << "Distortion Coefficients:" << std::endl;
  for (int i = 0; i < distortionParams.rows; ++i) {
      outFile << distortionParams.at<float>(i, 0) << " ";
  }
  outFile << std::endl;

  // 关闭文件
  outFile.close();
  std::cout << "Calibration results saved to: " << file_name << std::endl;
}

void LoadConfigJson(const std::string &filename, int &num_row, int &num_col, float &square_size) {
  Json::Reader reader;
  Json::Value root;

  std::ifstream in(filename, std::ios::binary);
  if (!in.is_open()) {
      std::cerr << "Error Opening " << filename << std::endl;
      return;
  }

  if (!reader.parse(in, root, false)) {
      std::cerr << "Failed to parse JSON file: " << filename << std::endl;
      return;
  }

  // 检查 JSON 文件的结构
  if (!root.isMember("camera_calibration")) {
      std::cerr << "Missing 'camera_calibration' section in JSON file" << std::endl;
      return;
  }

  Json::Value calibration = root["camera_calibration"];
  if (!calibration.isMember("num_row") || !calibration.isMember("num_col") || !calibration.isMember("square_size")) {
      std::cerr << "Missing required fields in 'camera_calibration' section" << std::endl;
      return;
  }

  // 读取值
  num_row = calibration["num_row"].asInt();
  num_col = calibration["num_col"].asInt();
  square_size = calibration["square_size"].asFloat();

  std::cout << "Loaded calibration parameters: "
            << "num_row=" << num_row << ", "
            << "num_col=" << num_col << ", "
            << "square_size=" << square_size << std::endl;

  in.close();
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
  //cv::imshow("camera", img);
  //cv::waitKey(1);
} 

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "camera_intrinsic_calibration");
  pkg_loc = ros::package::getPath("multi_sensor_calibration");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  string camera_topic;
  private_nh.param<string>("camera_topic", camera_topic, "/camera/image");

  camera_sub = nh.subscribe(camera_topic, 1, camera_callback);

  string config_json = pkg_loc + "/data/camera_intrinsic_calibration_config.json";
  string imagePath = pkg_loc + "/data/camera_intrinsic_calibration";
  int num_row, num_col;
  float square_size;
  LoadConfigJson(config_json, num_row, num_col, square_size);

  cout << "Loading data completed!" << endl;
  std::cout << __LINE__ << "\n";

  // view
  const int width = 1920, height = 1280;
  //std::cout << "width:" << width << " , height:" << height << std::endl;
  // const int width = 1920, height = 1200;
  pangolin::CreateWindowAndBind("cameraintrinsic player", width, height);

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

  pangolin::Var<bool> save_current_data("cp. save current data", false, false);
  pangolin::Var<bool> auto_calibrate("cp. auto_calibrate", false, false);

  pangolin::Var<bool> show_undistort("cp.show undistort", false, false);
  pangolin::Var<bool> saveImg("cp.Save Image", false, false);

  int frame_num = 0;
  std::vector<cv::Mat> img_list;

  std::cout << "\n=>START\n";
  while (!pangolin::ShouldQuit()) {
    ros::spinOnce();
    if(!img.empty()){
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      cv::Mat current_frame = ProjectToImage(img);
      if (pangolin::Pushed(save_current_data)) {
        img_list.push_back(img);
        std::cout << "Save current data!\n";
      }
      if (pangolin::Pushed(auto_calibrate)) {
        _cv::CalibrationHandler::Param param = _cv::getCalibrationHandlerParam(imagePath, img_list, num_row, num_col, square_size);
        _cv::CalibrationHandler calibHandler(param);
        if (calibHandler.allImagePoints().empty()) {
          std::cerr << "failed to get enough image points" << std::endl;
          return EXIT_FAILURE;
        }
        // uncomment the following line to draw detected chessboard corners in images written under /tmp
        // calibHandler.drawChessboardCorners();
    
        cv::Mat K, distortionParams;
        std::vector<cv::Mat> extrinsics;
        std::cout<<"start run"<<std::endl;
        calibHandler.run(K, distortionParams, extrinsics);
        std::cout<<"end run"<<std::endl;
        global_K = K;
        global_D = distortionParams;
  
        calibHandler.drawReporjections(K, distortionParams, extrinsics);
      }
      if (pangolin::Pushed(show_undistort)) {
        for(int i = 0; i < img_list.size(); ++i) {
          const cv::Size& imgSize = img_list[i].size();
          float alpha = 0;
          cv::Mat newCameraMatrix = _cv::getOptimalNewCameraMatrix(global_K, global_D, imgSize, alpha);
          cv::Mat map1, map2;
          _cv::initUndistortRectifyMap(global_K, global_D, imgSize, newCameraMatrix, map1, map2);
          cv::Mat undistorted;
      
          // _cv::remap(sampleImage, undistorted, map1, map2);
          cv::remap(img_list[i], undistorted, map1, map2, cv::INTER_AREA);  // use opencv's remap for now
          cv::hconcat(img_list[i], undistorted, undistorted);
          std::string img_name = pkg_loc + "/data/camera_intrinsic_calibration/camera_distort_" + std::to_string(i) + ".jpg";
          cv::imwrite(img_name, undistorted);
        }
      }
      if (pangolin::Pushed(saveImg)) {
        saveCalibrationResults(global_K, global_D, frame_num);
        frame_num++;
      }

        imageArray = current_frame.data;
        imageTexture.Upload(imageArray, GL_RGB, GL_UNSIGNED_BYTE);

        project_image.Activate();
        glColor3f(1.0, 1.0, 1.0);
        imageTexture.RenderToViewportFlipY();

        pangolin::FinishFrame();
        usleep(100);
        glFinish();
      }
  }

  // delete[] imageArray;
  ros::shutdown();
  return 0;
}
