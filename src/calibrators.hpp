#pragma once

// The five calibration workers. Each one is a straight port of the
// corresponding original standalone node: subscribe topics, load parameter
// files, run the per-frame Pangolin interaction loop (manual adjust /
// auto calibrate / save) and clean up when the user goes back to the
// setup page.

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pangolin/pangolin.h>

#include <sys/stat.h>

#include <type_traits>

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "calib_common.hpp"
#include "calib_types.hpp"
#include "projector_lidar.hpp"
#include "projector_radar.hpp"
#include "autocalib/camera_intrinsic/cam_intrinsic_calib.hpp"
#include "autocalib/lidar2camera/sensors_calib.hpp"
#include "autocalib/lidar2cameraV2/Calibrator.hpp"
#include "autocalib/lidar2cameraV2/Core.hpp"
#include "autocalib/lidar2cameraV2/Rand.hpp"

namespace msc {

// Create (once) and cache a Pangolin panel so repeated calibration sessions
// do not try to register the same panel name twice.
inline pangolin::View &ensurePanel(const std::string &ns, int x0, int x1) {
  static std::map<std::string, pangolin::View *> cache;
  auto it = cache.find(ns);
  if (it != cache.end())
    return *it->second;
  pangolin::View &p = pangolin::CreatePanel(ns);
  p.SetBounds(pangolin::Attach::Pix(30), 1.0, pangolin::Attach::Pix(x0),
              pangolin::Attach::Pix(x1));
  p.Show(false);
  // panels created at runtime (after the initial window layout) need an
  // explicit relayout, otherwise they render at a garbage viewport
  pangolin::DisplayBase().Resize(pangolin::DisplayBase().GetBounds());
  cache[ns] = &p;
  return p;
}

inline void ensureDir(const std::string &path) {
  struct stat st;
  if (stat(path.c_str(), &st) != 0)
    mkdir(path.c_str(), 0755);
}

// Views / GL state shared with the main application.
struct SharedViews {
  pangolin::View *view3d = nullptr;             // point cloud 3D view
  pangolin::OpenGlRenderState *cam3d = nullptr; // its render state
  pangolin::View *viewImg = nullptr;            // main image view
  pangolin::View *viewBv = nullptr;             // bird view (radar-camera)
};

class CalibratorBase {
public:
  CalibratorBase(ros::NodeHandle &nh, const std::string &pkg,
                 const SharedViews &views)
      : nh_(nh), pkg_loc_(pkg), views_(views) {}
  virtual ~CalibratorBase() { stop(); }

  virtual void start(const TypeConfig &cfg, const CalibParams &params) = 0;
  virtual void renderFrame() = 0;
  virtual void stop() {
    for (auto &s : subs_)
      s.shutdown();
    subs_.clear();
    if (panel_)
      panel_->Show(false);
  }

  bool backRequested() const { return back_; }
  void clearBack() { back_ = false; }
  const Eigen::Matrix4d &result() const { return adjuster_.matrix_; }

protected:
  // Subscribe a point-like slot: PointCloud2 (regular conversion) or
  // MarkerArray (one point per marker pose, like the original radar nodes).
  template <typename CloudT>
  ros::Subscriber subscribePoints(
      const std::string &topic, const std::string &datatype,
      const typename pcl::PointCloud<CloudT>::Ptr &cloud) {
    auto first_msg = std::make_shared<bool>(false);
    auto log_first = [topic, first_msg](size_t n) {
      if (!*first_msg) {
        *first_msg = true;
        std::cout << "[data] " << topic << ": first message, " << n
                  << " points" << std::endl;
      }
    };
    if (datatype == "visualization_msgs/MarkerArray") {
      return nh_.subscribe<visualization_msgs::MarkerArray>(
          topic, 1,
          [cloud, log_first](const visualization_msgs::MarkerArray::ConstPtr &m) {
            cloud->clear();
            for (const auto &o : m->markers) {
              CloudT p;
              p.x = o.pose.position.x;
              p.y = o.pose.position.y;
              p.z = o.pose.position.z;
              if constexpr (std::is_same<CloudT, pcl::PointXYZI>::value)
                p.intensity = 0.f;
              cloud->push_back(p);
            }
            log_first(cloud->size());
          });
    }
    return nh_.subscribe<sensor_msgs::PointCloud2>(
        topic, 1,
        [cloud, log_first](const sensor_msgs::PointCloud2::ConstPtr &m) {
          cloud->clear();
          pcl::fromROSMsg(*m, *cloud);
          log_first(cloud->size());
        });
  }

  // Recompute all view layouts after runtime SetBounds changes.
  void relayout() {
    pangolin::DisplayBase().Resize(pangolin::DisplayBase().GetBounds());
  }
  void handleCommonKeys() {
    if (kbhit()) {
      int c = getchar();
      if (adjuster_.applyKey(c)) {
        std::cout << "\nTransfromation Matrix:\n"
                  << adjuster_.matrix_ << std::endl;
      }
    }
  }
  // Handles the shared deg/t step sliders of an extrinsic calibrator.
  void handleStepSliders(pangolin::Var<double> *deg, pangolin::Var<double> *t) {
    if (deg->GuiChanged()) {
      adjuster_.degree_step_ = deg->Get();
      adjuster_.rebuild();
      std::cout << "Degree calib scale changed to " << adjuster_.degree_step_
                << " degree\n";
    }
    if (t->GuiChanged()) {
      adjuster_.trans_step_ = t->Get() / 100.0;
      adjuster_.rebuild();
      std::cout << "Trans calib scale changed to " << adjuster_.trans_step_ * 100
                << " cm\n";
    }
  }
  bool pushedBack() {
    if (back_var_ && pangolin::Pushed(*back_var_)) {
      back_ = true;
      return true;
    }
    return false;
  }

  ros::NodeHandle &nh_;
  std::string pkg_loc_;
  SharedViews views_;
  std::vector<ros::Subscriber> subs_;
  pangolin::View *panel_ = nullptr;
  ExtrinsicAdjuster adjuster_;
  bool back_ = false;
  pangolin::Var<bool> *back_var_ = nullptr;
  int frame_num_ = 0;
};

// =====================================================================
// 1. lidar <-> lidar  (NDT auto calibration)
// =====================================================================
class LidarLidarCalibrator : public CalibratorBase {
public:
  using CalibratorBase::CalibratorBase;

  void start(const TypeConfig &cfg, const CalibParams &params) override {
    target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    source_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);

    std::cout << "lidar to lidar extrinsic:\n" << params.extrinsic
              << std::endl;
    adjuster_.init(params.extrinsic);

    subs_.push_back(subscribePoints<pcl::PointXYZI>(
        cfg.slots[0].topic, cfg.slots[0].datatype, target_cloud_));
    subs_.push_back(subscribePoints<pcl::PointXYZI>(
        cfg.slots[1].topic, cfg.slots[1].datatype, source_cloud_));

    ensureDir(pkg_loc_ + "/data/lidar2lidar_extrinsic_calibration");

    // panel + vars
    panel_ = &ensurePanel("c.ll", 0, 260);
    panel_->Show();
    if (!vars_) {
      vars_.reset(new Vars());
      const char *names[12] = {"+ x degree", "- x degree", "+ y degree",
                               "- y degree", "+ z degree", "- z degree",
                               "+ x trans",  "- x trans",  "+ y trans",
                               "- y trans",  "+ z trans",  "- z trans"};
      for (int i = 0; i < 12; i++)
        vars_->buttons.emplace_back("c.ll." + std::string(names[i]), false,
                                    false);
    }
    back_var_ = &vars_->back;
    views_.view3d->Show();
  }

  void stop() override {
    CalibratorBase::stop();
    views_.view3d->Show(false);
  }

  void renderFrame() override {
    int source_n = source_cloud_->points.size();
    int target_n = target_cloud_->points.size();

    processTarget();
    processSource();

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();
    views_.cam3d->Follow(Twc);
    views_.view3d->Activate(*views_.cam3d);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (vars_->intensity.GuiChanged()) {
      display_mode_ = vars_->intensity.Get();
      std::cout << "intensity colour mode: " << display_mode_ << std::endl;
    }
    if (vars_->pointSize.GuiChanged()) {
      point_size_ = vars_->pointSize.Get();
      std::cout << "Point size changed to " << point_size_ << std::endl;
    }
    handleStepSliders(&vars_->degStep, &vars_->tStep);

    for (int i = 0; i < 12; i++) {
      if (pangolin::Pushed(vars_->buttons[i])) {
        adjuster_.apply(i);
        processSource();
        std::cout << "\nTransfromation Matrix:\n" << adjuster_.matrix_
                  << std::endl;
      }
    }

    if (pangolin::Pushed(vars_->autoCalib)) {
      Eigen::Matrix4d m = autoCalibration(target_cloud_, source_cloud_,
                                          adjuster_.matrix_);
      adjuster_.matrix_ = m * adjuster_.matrix_;
      processSource();
      std::cout << "\nTransfromation Matrix:\n" << adjuster_.matrix_
                << std::endl;
    }
    if (pangolin::Pushed(vars_->reset)) {
      adjuster_.reset();
      processSource();
      std::cout << "Reset!\n";
    }
    if (pangolin::Pushed(vars_->save)) {
      std::string name = pkg_loc_ + "/data/lidar2lidar_extrinsic_calibration/"
                        "lidar2lidar_extrinsic_" + std::to_string(frame_num_)
                        + ".txt";
      saveExtrinsicTxt(name, adjuster_.matrix_);
      std::cout << "\n==>Save Result " << frame_num_ << " -> " << name
                << std::endl;
      frame_num_++;
    }
    pushedBack();
    handleCommonKeys();

    glDisable(GL_LIGHTING);
    target_buffers_.draw(point_size_);
    source_buffers_.draw(point_size_);
    (void)source_n;
    (void)target_n;
  }

private:
  void processTarget() { target_buffers_.upload(target_cloud_, display_mode_); }
  void processSource() {
    // transform source cloud by current extrinsic, colour it red in grey mode
    pcl::PointCloud<pcl::PointXYZI>::Ptr out(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*source_cloud_, *out, adjuster_.matrix_);
    source_buffers_.upload(out, display_mode_, true);
  }

  Eigen::Matrix4d
  autoCalibration(const pcl::PointCloud<pcl::PointXYZI>::Ptr &target,
                  const pcl::PointCloud<pcl::PointXYZI>::Ptr &source,
                  const Eigen::Matrix4d &extrinsic) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);
    tgt->points.resize(target->points.size());
    for (size_t i = 0; i < target->points.size(); ++i) {
      tgt->points[i].x = target->points[i].x;
      tgt->points[i].y = target->points[i].y;
      tgt->points[i].z = target->points[i].z;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
    src->points.resize(source->points.size());
    for (size_t i = 0; i < source->points.size(); ++i) {
      Eigen::Vector4d p(source->points[i].x, source->points[i].y,
                        source->points[i].z, 1.0);
      Eigen::Vector4d t = extrinsic * p;
      src->points[i].x = t.x();
      src->points[i].y = t.y();
      src->points[i].z = t.z();
    }
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(*tgt, *tgt, idx);
    pcl::removeNaNFromPointCloud(*src, *src, idx);

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon(0.1);
    ndt.setStepSize(0.1);
    ndt.setResolution(0.5);
    ndt.setMaximumIterations(100);
    ndt.setInputSource(src);
    ndt.setInputTarget(tgt);
    pcl::PointCloud<pcl::PointXYZ> final_cloud;
    ndt.align(final_cloud);
    if (ndt.hasConverged())
      std::cout << "NDT has converged, score is " << ndt.getFitnessScore()
                << std::endl;
    else
      std::cout << "NDT did not converge." << std::endl;
    return ndt.getFinalTransformation().cast<double>();
  }

  struct Vars {
    pangolin::Var<bool> intensity{"c.ll.Intensity Color", false, true};
    pangolin::Var<int> pointSize{"c.ll.Point Size", 2, 0, 8};
    pangolin::Var<double> degStep{"c.ll.deg step", 0.3, 0, 1};
    pangolin::Var<double> tStep{"c.ll.t step(cm)", 6, 0, 15};
    std::vector<pangolin::Var<bool>> buttons;
    pangolin::Var<bool> autoCalib{"c.ll.ndt_auto_calibrate", false, false};
    pangolin::Var<bool> reset{"c.ll.Reset", false, false};
    pangolin::Var<bool> save{"c.ll.Save Result", false, false};
    pangolin::Var<bool> back{"c.ll.Back to Setup", false, false};
  };
  std::unique_ptr<Vars> vars_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud_;
  CloudGlBuffers target_buffers_;
  CloudGlBuffers source_buffers_;
  bool display_mode_ = false;
  int point_size_ = 2;
};

// =====================================================================
// 2. radar <-> lidar  (ICP auto calibration)
// =====================================================================
class RadarLidarCalibrator : public CalibratorBase {
public:
  using CalibratorBase::CalibratorBase;

  void start(const TypeConfig &cfg, const CalibParams &params) override {
    lidar_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    radar_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    transformed_radar_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    std::cout << "radar to lidar extrinsic:\n" << params.extrinsic
              << std::endl;
    adjuster_.init(params.extrinsic);

    subs_.push_back(subscribePoints<pcl::PointXYZI>(
        cfg.slots[0].topic, cfg.slots[0].datatype, lidar_cloud_));
    subs_.push_back(subscribePoints<pcl::PointXYZ>(
        cfg.slots[1].topic, cfg.slots[1].datatype, radar_cloud_));

    ensureDir(pkg_loc_ + "/data/radar2lidar_extrinsic_calibration");

    panel_ = &ensurePanel("c.rl", 0, 260);
    panel_->Show();
    if (!vars_) {
      vars_.reset(new Vars());
      const char *names[12] = {"+ x degree", "- x degree", "+ y degree",
                               "- y degree", "+ z degree", "- z degree",
                               "+ x trans",  "- x trans",  "+ y trans",
                               "- y trans",  "+ z trans",  "- z trans"};
      for (int i = 0; i < 12; i++)
        vars_->buttons.emplace_back("c.rl." + std::string(names[i]), false,
                                    false);
    }
    back_var_ = &vars_->back;
    views_.view3d->Show();
  }

  void stop() override {
    CalibratorBase::stop();
    views_.view3d->Show(false);
  }

  void renderFrame() override {
    int lidar_n = lidar_cloud_->points.size();
    lidar_buffers_.upload(lidar_cloud_, display_mode_);

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();
    views_.cam3d->Follow(Twc);
    views_.view3d->Activate(*views_.cam3d);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (vars_->intensity.GuiChanged()) {
      display_mode_ = vars_->intensity.Get();
      lidar_buffers_.upload(lidar_cloud_, display_mode_);
    }
    handleStepSliders(&vars_->degStep, &vars_->tStep);
    if (vars_->lineSize.GuiChanged()) {
      radar_line_size_ = vars_->lineSize.Get();
      std::cout << "radar line size changed to " << radar_line_size_
                << std::endl;
    }
    for (int i = 0; i < 12; i++) {
      if (pangolin::Pushed(vars_->buttons[i])) {
        adjuster_.apply(i);
        std::cout << "\nTransfromation Matrix:\n" << adjuster_.matrix_
                  << std::endl;
      }
    }
    if (pangolin::Pushed(vars_->autoCalib)) {
      Eigen::Matrix4d m = autoCalibration(lidar_cloud_, transformed_radar_);
      adjuster_.matrix_ = m * adjuster_.matrix_;
      std::cout << "\nTransfromation Matrix:\n" << adjuster_.matrix_
                << std::endl;
    }
    if (pangolin::Pushed(vars_->reset)) {
      adjuster_.reset();
      std::cout << "Reset!\n";
    }
    if (pangolin::Pushed(vars_->save)) {
      std::string name = pkg_loc_ + "/data/radar2lidar_extrinsic_calibration/"
                        "radar2lidar_extrinsic_" + std::to_string(frame_num_)
                        + ".txt";
      saveExtrinsicTxt(name, adjuster_.matrix_);
      std::cout << "\n==>Save Result " << frame_num_ << " -> " << name
                << std::endl;
      frame_num_++;
    }
    pushedBack();
    handleCommonKeys();

    // draw lidar points
    glDisable(GL_LIGHTING);
    lidar_buffers_.draw(2);
    (void)lidar_n;

    // draw radar lines (transformed by current extrinsic)
    transformed_radar_->clear();
    pcl::transformPointCloud(*radar_cloud_, *transformed_radar_,
                             adjuster_.matrix_);
    glLineWidth(radar_line_size_);
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_LINES);
    for (size_t i = 0; i < transformed_radar_->points.size(); i++) {
      double x = transformed_radar_->points[i].x;
      double y = transformed_radar_->points[i].y;
      double z = transformed_radar_->points[i].z;
      glVertex3d(x, y, z);
      glVertex3d(x, y, z + 2.5);
    }
    glEnd();
  }

private:
  Eigen::Matrix4d autoCalibration(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &radar) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);
    tgt->points.resize(lidar->points.size());
    for (size_t i = 0; i < lidar->points.size(); ++i) {
      tgt->points[i].x = lidar->points[i].x;
      tgt->points[i].y = lidar->points[i].y;
      tgt->points[i].z = lidar->points[i].z;
    }
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(radar);
    icp.setInputTarget(tgt);
    icp.setMaxCorrespondenceDistance(20);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(0.001);
    icp.setEuclideanFitnessEpsilon(0.01);
    pcl::PointCloud<pcl::PointXYZ> final_cloud;
    icp.align(final_cloud);
    if (icp.hasConverged())
      std::cout << "ICP has converged, score is " << icp.getFitnessScore()
                << std::endl;
    else
      std::cout << "ICP did not converge." << std::endl;
    return icp.getFinalTransformation().cast<double>();
  }

  struct Vars {
    pangolin::Var<bool> intensity{"c.rl.Intensity Color", false, true};
    pangolin::Var<double> degStep{"c.rl.deg step", 0.3, 0, 1};
    pangolin::Var<double> tStep{"c.rl.t step(cm)", 6, 0, 15};
    pangolin::Var<int> lineSize{"c.rl.radar line size", 4, 1, 10};
    std::vector<pangolin::Var<bool>> buttons;
    pangolin::Var<bool> autoCalib{"c.rl.icp_auto_calibrate", false, false};
    pangolin::Var<bool> reset{"c.rl.Reset", false, false};
    pangolin::Var<bool> save{"c.rl.Save Result", false, false};
    pangolin::Var<bool> back{"c.rl.Back to Setup", false, false};
  };
  std::unique_ptr<Vars> vars_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr radar_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_radar_;
  CloudGlBuffers lidar_buffers_;
  bool display_mode_ = false;
  int radar_line_size_ = 4;
};

// =====================================================================
// 3. lidar <-> camera
// =====================================================================
class LidarCameraCalibrator : public CalibratorBase {
public:
  using CalibratorBase::CalibratorBase;

  void start(const TypeConfig &cfg, const CalibParams &params) override {
    lidar_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);

    Eigen::Matrix3d K;
    K << params.fx, 0, params.cx, 0, params.fy, params.cy, 0, 0, 1;
    dist_ = {params.k1, params.k2, params.p1, params.p2, params.k3};
    intrinsic_ = K;
    origin_intrinsic_ = K;
    assignIntrinsic(K, dist_);
    const int width = params.img_w;
    const int height = params.img_h;

    std::cout << "lidar to camera extrinsic:\n" << params.extrinsic
              << std::endl;
    adjuster_.init(params.extrinsic);

    subs_.push_back(subscribePoints<pcl::PointXYZI>(
        cfg.slots[0].topic, cfg.slots[0].datatype, lidar_cloud_));
    subs_.push_back(nh_.subscribe<sensor_msgs::Image>(
        cfg.slots[1].topic, 1,
        [this](const sensor_msgs::Image::ConstPtr &m) {
          cv_bridge::CvImagePtr cv_ptr;
          try {
            cv_ptr = cv_bridge::toCvCopy(m, sensor_msgs::image_encodings::BGR8);
          } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
          }
          img_ = cv_ptr->image;
        }));

    ensureDir(pkg_loc_ + "/data/lidar2camera_extrinsic_calibration");

    texture_.reset(
        new pangolin::GlTexture(width, height, GL_RGB, false, 0, GL_RGB,
                                GL_UNSIGNED_BYTE));
    views_.viewImg->SetBounds(0.0, 1.0, pangolin::Attach::Pix(270), 1.0,
                              -1.0 * static_cast<double>(width) / height);
    views_.viewImg->Show();

    panel_ = &ensurePanel("c.lc", 0, 260);
    panel_->Show();
    if (!vars_) {
      vars_.reset(new Vars());
      const char *names[12] = {"+ x degree", "- x degree", "+ y degree",
                               "- y degree", "+ z degree", "- z degree",
                               "+ x trans",  "- x trans",  "+ y trans",
                               "- y trans",  "+ z trans",  "- z trans"};
      for (int i = 0; i < 12; i++)
        vars_->buttons.emplace_back("c.lc." + std::string(names[i]), false,
                                    false);
    }
    back_var_ = &vars_->back;
    img_list_.clear();
    pcd_list_.clear();
  }

  void stop() override {
    CalibratorBase::stop();
    views_.viewImg->Show(false);
    img_.release();
    lidar_cloud_.reset();
    img_list_.clear();
    pcd_list_.clear();
  }

  void renderFrame() override {
    if (img_.empty())
      return;

    projector_.loadPointCloud(*lidar_cloud_);
    cv::Mat current_frame =
        projector_.ProjectToRawImage(img_, intrinsic_, dist_, adjuster_.matrix_);

    if (vars_->intensity.GuiChanged()) {
      projector_.setDisplayMode(vars_->intensity.Get());
      current_frame =
          projector_.ProjectToRawImage(img_, intrinsic_, dist_, adjuster_.matrix_);
    }
    if (vars_->filter.GuiChanged()) {
      projector_.setFilterMode(vars_->filter.Get());
      current_frame =
          projector_.ProjectToRawImage(img_, intrinsic_, dist_, adjuster_.matrix_);
    }
    handleStepSliders(&vars_->degStep, &vars_->tStep);
    if (vars_->fxfyScale.GuiChanged()) {
      fxfy_scale_ = vars_->fxfyScale.Get();
      std::cout << "fxfy calib scale changed to " << fxfy_scale_ << std::endl;
    }
    if (vars_->pointSize.GuiChanged()) {
      projector_.setPointSize(vars_->pointSize.Get());
      current_frame =
          projector_.ProjectToRawImage(img_, intrinsic_, dist_, adjuster_.matrix_);
    }
    for (int i = 0; i < 12; i++) {
      if (pangolin::Pushed(vars_->buttons[i])) {
        adjuster_.apply(i);
        current_frame = projector_.ProjectToRawImage(img_, intrinsic_, dist_,
                                                     adjuster_.matrix_);
      }
    }
    if (pangolin::Pushed(vars_->addFx)) {
      intrinsic_(0, 0) *= fxfy_scale_;
      current_frame = projector_.ProjectToRawImage(img_, intrinsic_, dist_,
                                                   adjuster_.matrix_);
      std::cout << "fx changed to " << intrinsic_(0, 0) << std::endl;
    }
    if (pangolin::Pushed(vars_->minusFx)) {
      intrinsic_(0, 0) /= fxfy_scale_;
      current_frame = projector_.ProjectToRawImage(img_, intrinsic_, dist_,
                                                   adjuster_.matrix_);
      std::cout << "fx changed to " << intrinsic_(0, 0) << std::endl;
    }
    if (pangolin::Pushed(vars_->addFy)) {
      intrinsic_(1, 1) *= fxfy_scale_;
      current_frame = projector_.ProjectToRawImage(img_, intrinsic_, dist_,
                                                   adjuster_.matrix_);
      std::cout << "fy changed to " << intrinsic_(1, 1) << std::endl;
    }
    if (pangolin::Pushed(vars_->minusFy)) {
      intrinsic_(1, 1) /= fxfy_scale_;
      current_frame = projector_.ProjectToRawImage(img_, intrinsic_, dist_,
                                                   adjuster_.matrix_);
      std::cout << "fy changed to " << intrinsic_(1, 1) << std::endl;
    }
    if (pangolin::Pushed(vars_->saveData)) {
      cv::Mat undistorted;
      assignIntrinsic(intrinsic_, dist_);
      cv::undistort(img_, undistorted, global_K_, global_D_);
      img_list_.push_back(undistorted);
      pcd_list_.push_back(*lidar_cloud_);
      std::cout << "Save current data! (total " << img_list_.size() << ")\n";
    }
    if (pangolin::Pushed(vars_->autoCalib)) {
      runAutoCalibV1(current_frame);
    }
    if (pangolin::Pushed(vars_->autoCalibV2)) {
      runAutoCalibV2(current_frame);
    }
    if (pangolin::Pushed(vars_->reset)) {
      adjuster_.reset();
      intrinsic_ = origin_intrinsic_;
      current_frame = projector_.ProjectToRawImage(img_, intrinsic_, dist_,
                                                   adjuster_.matrix_);
      std::cout << "Reset!\n";
    }
    if (pangolin::Pushed(vars_->save)) {
      std::string name = pkg_loc_ +
                         "/data/lidar2camera_extrinsic_calibration/"
                         "lidar2camera_extrinsic_" +
                         std::to_string(frame_num_) + ".txt";
      saveExtrinsicTxt(name, adjuster_.matrix_, intrinsic_, &dist_, true);
      cv::imwrite(pkg_loc_ + "/data/lidar2camera_extrinsic_calibration/"
                   "calibimg_" + std::to_string(frame_num_) + ".jpg",
                  current_frame);
      std::cout << "\n==>Save Result " << frame_num_ << " -> " << name
                << std::endl;
      frame_num_++;
    }
    pushedBack();
    handleCommonKeys();

    texture_->Upload(current_frame.data, GL_BGR, GL_UNSIGNED_BYTE);
    views_.viewImg->Activate();
    glColor3f(1.0, 1.0, 1.0);
    texture_->RenderToViewportFlipY();
  }

private:
  void assignIntrinsic(const Eigen::Matrix3d &K, const std::vector<double> &d) {
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        global_K_.at<double>(i, j) = K(i, j);
    for (size_t i = 0; i < d.size() && i < 5; ++i)
      global_D_.at<double>(i, 0) = d[i];
  }

  void runAutoCalibV1(cv::Mat &current_frame) {
    perception::CalibrationHandlerParam param =
        perception::getCalibrationHandlerParam(adjuster_.matrix_, img_list_,
                                               pcd_list_, intrinsic_);
    perception::CalibrationHandler<pcl::PointXYZI>::Ptr handler(
        new perception::CalibrationHandler<pcl::PointXYZI>(param));
    auto transform = handler->optimize();
    Eigen::Affine3d affine = perception::toAffine(transform);
    if (affine.matrix().array().isNaN().any()) {
      std::cout << "The affine matrix contains NaN values." << std::endl;
    } else {
      adjuster_.matrix_ = affine.matrix();
    }
    current_frame = projector_.ProjectToRawImage(img_, intrinsic_, dist_,
                                                 adjuster_.matrix_);
    std::cout << "\nTransfromation Matrix:\n" << adjuster_.matrix_ << std::endl;
    img_list_.clear();
    pcd_list_.clear();
  }

  void runAutoCalibV2(cv::Mat &current_frame) {
    alcc::Calibrator calibrator;
    float fx = intrinsic_(0, 0), fy = intrinsic_(1, 1);
    float cx = intrinsic_(0, 2), cy = intrinsic_(1, 2);
    calibrator.SetCameraIntrinsic(fx * 0.3f, fy * 0.3f, cx * 0.3f, cy * 0.3f);
    calibrator.SetMaxFrameNum(static_cast<int>(img_list_.size()));
    for (size_t i = 0; i < img_list_.size(); i++) {
      cv::resize(img_list_[i], img_list_[i], cv::Size(), 0.3, 0.3);
      pcl::VoxelGrid<pcl::PointXYZI> vg;
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
          new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcd(
          new pcl::PointCloud<pcl::PointXYZI>(pcd_list_[i]));
      vg.setInputCloud(pcd);
      vg.setLeafSize(0.2, 0.2, 0.2);
      vg.filter(*filtered);
      calibrator.AddDataFrame(filtered, img_list_[i]);
    }
    Eigen::Isometry3f origin_extri = Eigen::Isometry3f::Identity();
    origin_extri.matrix() = adjuster_.matrix_.cast<float>();
    float score_origin = calibrator.MiscalibrationDetection(origin_extri);
    std::cout << "Matrix before autocalib is : " << std::endl
              << origin_extri.matrix() << std::endl;
    std::cout << "Score before calib is : " << score_origin << std::endl;

    int iteration_number = 10;
    Eigen::Isometry3f result;
    calibrator.CalibrationTrack(origin_extri, result, iteration_number);
    float score_after = calibrator.MiscalibrationDetection(result);
    std::cout << "Matrix after autocalib is : " << std::endl
              << origin_extri.matrix() << std::endl;
    std::cout << "Score after calib is : " << score_after << std::endl;
    if (result.matrix().array().isNaN().any()) {
      std::cout << "The affine matrix contains NaN values." << std::endl;
    } else {
      adjuster_.matrix_ = result.matrix().cast<double>();
    }
    current_frame = projector_.ProjectToRawImage(img_, intrinsic_, dist_,
                                                 adjuster_.matrix_);
    std::cout << "\nTransfromation Matrix:\n" << adjuster_.matrix_ << std::endl;
    img_list_.clear();
    pcd_list_.clear();
  }

  struct Vars {
    pangolin::Var<bool> intensity{"c.lc.Intensity Color", false, true};
    pangolin::Var<bool> filter{"c.lc.Overlap Filter", false, true};
    pangolin::Var<double> degStep{"c.lc.deg step", 0.3, 0, 1};
    pangolin::Var<double> tStep{"c.lc.t step(cm)", 6, 0, 15};
    pangolin::Var<double> fxfyScale{"c.lc.fxfy scale", 1.005, 1, 1.1};
    pangolin::Var<int> pointSize{"c.lc.point size", 3, 1, 5};
    std::vector<pangolin::Var<bool>> buttons;
    pangolin::Var<bool> addFx{"c.lc.+ fx", false, false};
    pangolin::Var<bool> minusFx{"c.lc.- fx", false, false};
    pangolin::Var<bool> addFy{"c.lc.+ fy", false, false};
    pangolin::Var<bool> minusFy{"c.lc.- fy", false, false};
    pangolin::Var<bool> saveData{"c.lc.save current data", false, false};
    pangolin::Var<bool> autoCalib{"c.lc.auto_calibrate", false, false};
    pangolin::Var<bool> autoCalibV2{"c.lc.auto_calibrate_v2", false, false};
    pangolin::Var<bool> reset{"c.lc.Reset", false, false};
    pangolin::Var<bool> save{"c.lc.Save Result", false, false};
    pangolin::Var<bool> back{"c.lc.Back to Setup", false, false};
  };
  std::unique_ptr<Vars> vars_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_;
  cv::Mat img_;
  cv::Mat global_K_ = cv::Mat(3, 3, CV_64FC1);
  cv::Mat global_D_ = cv::Mat(5, 1, CV_64FC1);
  Eigen::Matrix3d intrinsic_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d origin_intrinsic_ = Eigen::Matrix3d::Identity();
  std::vector<double> dist_;
  double fxfy_scale_ = 1.005;
  LidarProjector projector_;
  std::unique_ptr<pangolin::GlTexture> texture_;
  std::vector<cv::Mat> img_list_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> pcd_list_;
};

// =====================================================================
// 4. radar <-> camera  (projection + bird view)
// =====================================================================

// Forwards image-view clicks (GL window coords, y up) to a callback; the
// calibrator converts them to image pixels.
class ImgClickHandler : public pangolin::Handler {
public:
  std::function<void(int, int)> on_click;
  ImgClickHandler(std::function<void(int, int)> cb) : on_click(std::move(cb)) {}
  void Mouse(pangolin::View &v, pangolin::MouseButton button, int x, int y,
             bool pressed, int button_state) override {
    if (pressed && (button & pangolin::MouseButtonLeft) && on_click)
      on_click(x, y);
  }
};

class RadarCameraCalibrator : public CalibratorBase {
public:
  using CalibratorBase::CalibratorBase;

  void start(const TypeConfig &cfg, const CalibParams &params) override {
    radar_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Matrix3d K;
    K << params.fx, 0, params.cx, 0, params.fy, params.cy, 0, 0, 1;
    dist_ = {params.k1, params.k2, params.p1, params.p2, params.k3};
    intrinsic_ = K;
    const int width = params.img_w;
    const int height = params.img_h;

    homography_ = params.homography;
    std::cout << "Homography:\n" << homography_ << std::endl;

    std::cout << "radar to camera extrinsic:\n" << params.extrinsic
              << std::endl;
    adjuster_.init(params.extrinsic);

    bv_width_ = width / 2;
    bv_height_ = height;
    projector_ready_ = false;

    subs_.push_back(subscribePoints<pcl::PointXYZ>(
        cfg.slots[0].topic, cfg.slots[0].datatype, radar_cloud_));
    subs_.push_back(nh_.subscribe<sensor_msgs::Image>(
        cfg.slots[1].topic, 1,
        [this](const sensor_msgs::Image::ConstPtr &m) {
          cv_bridge::CvImagePtr cv_ptr;
          try {
            cv_ptr = cv_bridge::toCvCopy(m, sensor_msgs::image_encodings::BGR8);
          } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
          }
          img_ = cv_ptr->image;
        }));

    ensureDir(pkg_loc_ + "/data/radar2camera_extrinsic_calibration");

    texture_.reset(
        new pangolin::GlTexture(width, height, GL_RGB, false, 0, GL_RGB,
                                GL_UNSIGNED_BYTE));
    texture_bv_.reset(
        new pangolin::GlTexture(bv_width_, bv_height_, GL_RGB, false, 0, GL_RGB,
                                GL_UNSIGNED_BYTE));
    // positive aspect = fit inside the bounds (letterboxed), so the main
    // view and the bird view never grow into each other's area
    views_.viewImg->SetBounds(0.0, 1.0, pangolin::Attach::Pix(270), 0.65,
                              static_cast<double>(width) / height);
    views_.viewBv->SetBounds(0.0, 1.0, 0.65, 1.0,
                             static_cast<double>(bv_width_) / bv_height_);
    views_.viewImg->Show();
    views_.viewBv->Show();

    // lane-point picking directly on the image view (replaces the original
    // OpenCV picker window)
    picked_.clear();
    picking_done_ = false;
    pick_handler_.reset(new ImgClickHandler(
        [this](int x, int y) { onImageViewClick(x, y); }));
    views_.viewImg->SetHandler(pick_handler_.get());

    panel_ = &ensurePanel("c.rc", 0, 260);
    panel_->Show();
    if (!vars_) {
      vars_.reset(new Vars());
        // original node only exposed 10 buttons (no z translation)
      const char *names[10] = {"+ x degree", "- x degree", "+ y degree",
                               "- y degree", "+ z degree", "- z degree",
                               "+ x trans",  "- x trans",  "+ y trans",
                               "- y trans"};
      for (int i = 0; i < 10; i++)
        vars_->buttons.emplace_back("c.rc." + std::string(names[i]), false,
                                    false);
    }
    back_var_ = &vars_->back;
    std::cout << "\n[radar-camera] click 4 lane points on the image: "
                 "left-down, left-up, right-down, right-up\n"
                 "   (panel: 'repick points' to restart, 'default region' to "
                 "skip picking)\n"
              << std::endl;
  }

  void stop() override {
    // detach the click handler BEFORE this object dies (it captures `this`)
    views_.viewImg->SetHandler(new pangolin::Handler());
    CalibratorBase::stop();
    views_.viewImg->Show(false);
    views_.viewBv->Show(false);
    img_.release();
    radar_cloud_.reset();
    projector_ready_ = false;
    picking_done_ = false;
  }

  void renderFrame() override {
    if (img_.empty())
      return;
    if (!projector_ready_) {
      projector_.init(img_, homography_, bv_width_, bv_height_);
      projector_ready_ = true;
      // freeze the first frame for lane-point picking (like the original
      // tool, which picked on a static image)
      pick_frame_ = img_.clone();
      cv::Mat frozen_main;
      projector_.loadPointCloud(*radar_cloud_);
      projector_.ProjectToRawImage(img_, intrinsic_, dist_, adjuster_.matrix_,
                                   frozen_main, frozen_bv_);
      std::cout << "projector ready (birdview " << bv_width_ << "x"
                << bv_height_ << "), image frozen for picking" << std::endl;
    }

    // lane-point picking controls
    if (pangolin::Pushed(vars_->repick)) {
      picked_.clear();
      picking_done_ = false;
      pick_frame_ = img_.clone(); // freeze the CURRENT frame for repicking
      std::cout << "[radar-camera] repick: click 4 lane points (image "
                   "frozen)"
                << std::endl;
    }
    if (pangolin::Pushed(vars_->defaultRegion)) {
      picked_.clear();
      projector_.init(img_, homography_, bv_width_, bv_height_);
      picking_done_ = true;
      std::cout << "[radar-camera] bird view reset to default region"
                << std::endl;
    }

    if (!picking_done_) {
      // static image + picking overlay; no radar projection while picking
      cv::Mat disp = pick_frame_.clone();
      drawPickingOverlay(disp);
      vars_->pickLabel = "frozen, click " + std::to_string(picked_.size()) +
                         "/4";
      texture_->Upload(disp.data, GL_BGR, GL_UNSIGNED_BYTE);
      texture_bv_->Upload(frozen_bv_.data, GL_BGR, GL_UNSIGNED_BYTE);
      views_.viewImg->Activate();
      glGetIntegerv(GL_VIEWPORT, img_vp_);
      glColor3f(1.0, 1.0, 1.0);
      texture_->RenderToViewportFlipY();
      views_.viewBv->Activate();
      glColor3f(1.0, 1.0, 1.0);
      texture_bv_->RenderToViewportFlipY();
      return;
    }
    vars_->pickLabel = "done (" + std::to_string(picked_.size()) + " pts)";

    projector_.loadPointCloud(*radar_cloud_);
    cv::Mat current_frame, bv_frame;
    projector_.ProjectToRawImage(img_, intrinsic_, dist_, adjuster_.matrix_,
                                 current_frame, bv_frame);

    handleStepSliders(&vars_->degStep, &vars_->tStep);
    if (vars_->pointSize.GuiChanged()) {
      projector_.setPointSize(vars_->pointSize.Get());
      projector_.ProjectToRawImage(img_, intrinsic_, dist_, adjuster_.matrix_,
                                   current_frame, bv_frame);
    }
    for (int i = 0; i < 10; i++) {
      if (pangolin::Pushed(vars_->buttons[i])) {
        adjuster_.apply(i);
        projector_.ProjectToRawImage(img_, intrinsic_, dist_, adjuster_.matrix_,
                                     current_frame, bv_frame);
      }
    }
    if (pangolin::Pushed(vars_->reset)) {
      adjuster_.reset();
      projector_.ProjectToRawImage(img_, intrinsic_, dist_, adjuster_.matrix_,
                                   current_frame, bv_frame);
      std::cout << "Reset!\n";
    }
    if (pangolin::Pushed(vars_->save)) {
      std::string name = pkg_loc_ +
                         "/data/radar2camera_extrinsic_calibration/"
                         "radar2camera_extrinsic_" +
                         std::to_string(frame_num_) + ".txt";
      saveExtrinsicTxt(name, adjuster_.matrix_, intrinsic_, &dist_, true);
      cv::imwrite(pkg_loc_ + "/data/radar2camera_extrinsic_calibration/"
                   "calibimg_" + std::to_string(frame_num_) + ".jpg",
                  current_frame);
      cv::imwrite(pkg_loc_ + "/data/radar2camera_extrinsic_calibration/"
                   "calibimg_bv_" + std::to_string(frame_num_) + ".jpg",
                  bv_frame);
      std::cout << "\n==>Save Result " << frame_num_ << " -> " << name
                << std::endl;
      frame_num_++;
    }
    pushedBack();
    handleCommonKeys();

    texture_->Upload(current_frame.data, GL_BGR, GL_UNSIGNED_BYTE);
    texture_bv_->Upload(bv_frame.data, GL_BGR, GL_UNSIGNED_BYTE);
    views_.viewImg->Activate();
    glGetIntegerv(GL_VIEWPORT, img_vp_); // remember for click->pixel mapping
    glColor3f(1.0, 1.0, 1.0);
    texture_->RenderToViewportFlipY();
    views_.viewBv->Activate();
    glColor3f(1.0, 1.0, 1.0);
    texture_bv_->RenderToViewportFlipY();
  }

private:
  // window GL coords (y up) -> image pixel
  cv::Point2f windowToImage(int wx, int wy) const {
    float vx = (float)(wx - img_vp_[0]);
    float vy = (float)(wy - img_vp_[1]);
    float col = vx / img_vp_[2] * img_.cols;
    float row = (1.0f - vy / img_vp_[3]) * img_.rows; // FlipY rendering
    return cv::Point2f(col, row);
  }

  void onImageViewClick(int wx, int wy) {
    if (picking_done_ || img_.empty() || img_vp_[2] <= 0)
      return;
    cv::Point2f p = windowToImage(wx, wy);
    if (p.x < 0 || p.y < 0 || p.x >= img_.cols || p.y >= img_.rows)
      return;
    picked_.push_back(p);
    std::cout << "[radar-camera] picked " << (int)p.x << ", " << (int)p.y
              << " (" << picked_.size() << "/4)" << std::endl;
    if (picked_.size() == 4) {
      if (projector_.initFromImagePoints(img_, homography_, bv_width_,
                                         bv_height_, picked_)) {
        picking_done_ = true;
        std::cout << "[radar-camera] bird view updated from lane points"
                  << std::endl;
      } else {
        picked_.clear();
        std::cout << "[radar-camera] invalid points, cleared - click 4 "
                     "points again"
                  << std::endl;
      }
    }
  }

  void drawPickingOverlay(cv::Mat &frame) {
    const char *order[4] = {"1 left-down", "2 left-up", "3 right-down",
                            "4 right-up"};
    for (size_t i = 0; i < picked_.size(); i++) {
      cv::circle(frame, picked_[i], 8, cv::Scalar(0, 0, 255), -1);
      cv::putText(frame, order[i], picked_[i] + cv::Point2f(12, -12),
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
      if (i > 0)
        cv::line(frame, picked_[i - 1], picked_[i], cv::Scalar(0, 0, 255), 2);
    }
    if (picked_.size() == 4)
      cv::line(frame, picked_[3], picked_[0], cv::Scalar(0, 0, 255), 2);
    cv::putText(frame,
                "image FROZEN - click lane points: left-down, left-up, "
                "right-down, right-up",
                cv::Point(30, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(0, 255, 255), 2);
  }

  struct Vars {
    pangolin::Var<std::string> pickLabel{"c.rc.lane points", "click 0/4",
                                         pangolin::META_FLAG_READONLY};
    pangolin::Var<bool> repick{"c.rc.repick points", false, false};
    pangolin::Var<bool> defaultRegion{"c.rc.default region", false, false};
    pangolin::Var<double> degStep{"c.rc.deg step", 0.3, 0, 1};
    pangolin::Var<double> tStep{"c.rc.t step(cm)", 6, 0, 15};
    pangolin::Var<int> pointSize{"c.rc.point size", 6, 1, 10};
    std::vector<pangolin::Var<bool>> buttons;
    pangolin::Var<bool> reset{"c.rc.Reset", false, false};
    pangolin::Var<bool> save{"c.rc.Save Result", false, false};
    pangolin::Var<bool> back{"c.rc.Back to Setup", false, false};
  };
  std::unique_ptr<Vars> vars_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr radar_cloud_;
  cv::Mat img_;
  Eigen::Matrix3d intrinsic_ = Eigen::Matrix3d::Identity();
  std::vector<double> dist_;
  Eigen::Matrix3d homography_ = Eigen::Matrix3d::Identity();
  RadarProjector projector_;
  bool projector_ready_ = false;
  std::vector<cv::Point2f> picked_;
  bool picking_done_ = false;
  cv::Mat pick_frame_;  // frozen image used while picking
  cv::Mat frozen_bv_;   // bird view rendered from the frozen frame
  GLint img_vp_[4] = {0, 0, 0, 0};
  std::unique_ptr<ImgClickHandler> pick_handler_;
  int bv_width_ = 0, bv_height_ = 0;
  std::unique_ptr<pangolin::GlTexture> texture_;
  std::unique_ptr<pangolin::GlTexture> texture_bv_;
};

// =====================================================================
// 5. camera intrinsic (chessboard)
// =====================================================================
class CameraIntrinsicCalibrator : public CalibratorBase {
public:
  using CalibratorBase::CalibratorBase;

  void start(const TypeConfig &cfg, const CalibParams &params) override {
    board_rows_ = params.board_rows;
    board_cols_ = params.board_cols;
    board_square_ = params.board_square_size;
    std::cout << "chessboard: rows=" << board_rows_ << " cols=" << board_cols_
              << " square=" << board_square_ << std::endl;

    subs_.push_back(nh_.subscribe<sensor_msgs::Image>(
        cfg.slots[0].topic, 1,
        [this](const sensor_msgs::Image::ConstPtr &m) {
          cv_bridge::CvImagePtr cv_ptr;
          try {
            cv_ptr = cv_bridge::toCvCopy(m, sensor_msgs::image_encodings::BGR8);
          } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
          }
          img_ = cv_ptr->image;
        }));

    ensureDir(pkg_loc_ + "/data/camera_intrinsic_calibration");

    views_.viewImg->SetBounds(0.0, 1.0, pangolin::Attach::Pix(270), 1.0, 1.0);
    views_.viewImg->Show();

    panel_ = &ensurePanel("c.ci", 0, 260);
    panel_->Show();
    if (!vars_) {
      vars_.reset(new Vars());
    }
    back_var_ = &vars_->back;
    img_list_.clear();
    global_K_.release();
    global_D_.release();
    texture_ready_ = false;
  }

  void stop() override {
    CalibratorBase::stop();
    views_.viewImg->Show(false);
    img_.release();
    img_list_.clear();
    texture_ready_ = false;
  }

  void renderFrame() override {
    if (img_.empty())
      return;
    if (!texture_ready_) {
      std::cout << "camera intrinsic: first image " << img_.cols << "x"
                << img_.rows << std::endl;
      texture_.reset(new pangolin::GlTexture(img_.cols, img_.rows, GL_RGB,
                                             false, 0, GL_RGB,
                                             GL_UNSIGNED_BYTE));
      texture_ready_ = true;
      views_.viewImg->SetBounds(0.0, 1.0, pangolin::Attach::Pix(270), 1.0,
                                static_cast<double>(img_.cols) / img_.rows);
      relayout();
    }
    cv::Mat current_frame;
    cv::cvtColor(img_, current_frame, cv::COLOR_BGR2RGB);

    if (pangolin::Pushed(vars_->saveData)) {
      img_list_.push_back(img_.clone());
      std::cout << "Save current data! (total " << img_list_.size() << ")\n";
    }
    if (pangolin::Pushed(vars_->autoCalib)) {
      runCalibration();
    }
    if (pangolin::Pushed(vars_->showUndistort)) {
      showUndistorted();
    }
    if (pangolin::Pushed(vars_->save)) {
      saveResults(frame_num_);
      frame_num_++;
    }
    pushedBack();

    texture_->Upload(current_frame.data, GL_RGB, GL_UNSIGNED_BYTE);
    views_.viewImg->Activate();
    glColor3f(1.0, 1.0, 1.0);
    texture_->RenderToViewportFlipY();
  }

private:
  void runCalibration() {
    if (img_list_.size() < 3) {
      std::cerr << "not enough frames saved (need >= 3, have "
                << img_list_.size() << ")" << std::endl;
      return;
    }
    std::string image_path = pkg_loc_ + "/data/camera_intrinsic_calibration";
    _cv::CalibrationHandler::Param param = _cv::getCalibrationHandlerParam(
        image_path, img_list_, board_rows_, board_cols_, board_square_);
    _cv::CalibrationHandler calib_handler(param);
    if (calib_handler.allImagePoints().empty()) {
      std::cerr << "failed to get enough image points" << std::endl;
      return;
    }
    cv::Mat K, D;
    std::vector<cv::Mat> extrinsics;
    std::cout << "start calibration run" << std::endl;
    calib_handler.run(K, D, extrinsics);
    std::cout << "end calibration run" << std::endl;
    global_K_ = K;
    global_D_ = D;
    calib_handler.drawReporjections(K, D, extrinsics);
    std::cout << "Camera Intrinsic Matrix (K):\n" << K << std::endl;
    std::cout << "Distortion Coefficients:\n" << D << std::endl;
  }

  void showUndistorted() {
    if (global_K_.empty() || global_D_.empty()) {
      std::cerr << "run auto_calibrate first" << std::endl;
      return;
    }
    for (size_t i = 0; i < img_list_.size(); ++i) {
      const cv::Size &size = img_list_[i].size();
      float alpha = 0;
      cv::Mat new_K =
          _cv::getOptimalNewCameraMatrix(global_K_, global_D_, size, alpha);
      cv::Mat map1, map2;
      _cv::initUndistortRectifyMap(global_K_, global_D_, size, new_K, map1,
                                   map2);
      cv::Mat undistorted;
      cv::remap(img_list_[i], undistorted, map1, map2, cv::INTER_AREA);
      cv::hconcat(img_list_[i], undistorted, undistorted);
      cv::imwrite(pkg_loc_ + "/data/camera_intrinsic_calibration/"
                   "camera_distort_" + std::to_string(i) + ".jpg",
                  undistorted);
    }
    std::cout << "undistortion comparison images written to "
              << pkg_loc_ + "/data/camera_intrinsic_calibration" << std::endl;
  }

  void saveResults(int frame_id) {
    if (global_K_.empty() || global_D_.empty()) {
      std::cerr << "no calibration result yet (run auto_calibrate first)"
                << std::endl;
      return;
    }
    std::string name = pkg_loc_ + "/data/camera_intrinsic_calibration/"
                        "camera_intrinsic_" + std::to_string(frame_id) + ".txt";
    std::ofstream out(name);
    if (!out.is_open()) {
      std::cerr << "Failed to open file: " << name << std::endl;
      return;
    }
    out << "Camera Intrinsic Matrix (K):" << std::endl;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        out << global_K_.at<float>(i, j) << (j == 2 ? "\n" : " ");
      }
    }
    out << "Distortion Coefficients:" << std::endl;
    for (int i = 0; i < global_D_.rows; ++i)
      out << global_D_.at<float>(i, 0) << " ";
    out << std::endl;
    out.close();
    std::cout << "Calibration results saved to: " << name << std::endl;
  }

  struct Vars {
    pangolin::Var<bool> saveData{"c.ci.save current data", false, false};
    pangolin::Var<bool> autoCalib{"c.ci.auto_calibrate", false, false};
    pangolin::Var<bool> showUndistort{"c.ci.show undistort", false, false};
    pangolin::Var<bool> save{"c.ci.Save Result", false, false};
    pangolin::Var<bool> back{"c.ci.Back to Setup", false, false};
  };
  std::unique_ptr<Vars> vars_;

  cv::Mat img_;
  std::vector<cv::Mat> img_list_;
  cv::Mat global_K_, global_D_;
  int board_rows_ = 8, board_cols_ = 11;
  double board_square_ = 0.015;
  std::unique_ptr<pangolin::GlTexture> texture_;
  bool texture_ready_ = false;
};

} // namespace msc
