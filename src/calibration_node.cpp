// multi_sensor_calibration - unified calibration node.
//
// One node, one window, two pages:
//   Setup page: pick calibration type -> pick live topics (auto discovered)
//               -> review type specific parameters -> START.
//   Calibration page: the original per-type interactive calibration GUI
//               (manual adjust / auto calibrate / save), plus a
//               "Back to Setup" button.
// The red "Exit" button in the top-right corner quits the node cleanly,
// no terminal Ctrl-C required.

#include <ros/master.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pangolin/pangolin.h>
#include <jsoncpp/json/json.h>

#include <unistd.h>

#include <csignal>
#include <cstdio>
#include <execinfo.h>

#include <algorithm>
#include <array>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "calib_types.hpp"
#include "calib_ui.hpp"
#include "calibrators.hpp"
#include "extrinsic_param.hpp"   // json loaders for widget prefill
#include "homography_param.hpp"
#include "intrinsic_param.hpp"

using namespace msc;

namespace {

// print a backtrace on fatal signals (no gdb needed on target machines)
void fatalSignalHandler(int sig) {
  void *bt[32];
  int n = backtrace(bt, 32);
  fprintf(stderr, "\n=== FATAL signal %d, backtrace (%d frames) ===\n", sig, n);
  backtrace_symbols_fd(bt, n, 2);
  signal(sig, SIG_DFL);
  raise(sig);
}

enum class Page { Setup, Calibration };

// ----------------------------------------------------------- setup page UI
struct SetupVars {
  pangolin::Var<std::string> type{"setup.current type", "- none -",
                                  pangolin::META_FLAG_READONLY};
  pangolin::Var<bool> btnLL{"setup.1 lidar<->lidar", false, false};
  pangolin::Var<bool> btnRL{"setup.2 radar<->lidar", false, false};
  pangolin::Var<bool> btnLC{"setup.3 lidar<->camera", false, false};
  pangolin::Var<bool> btnRC{"setup.4 radar<->camera", false, false};
  pangolin::Var<bool> btnCI{"setup.5 camera intrinsic", false, false};
  pangolin::Var<std::string> slotA{"setup.slot A", "(empty)",
                                   pangolin::META_FLAG_READONLY};
  pangolin::Var<std::string> slotB{"setup.slot B", "(empty)",
                                   pangolin::META_FLAG_READONLY};
  pangolin::Var<int> topicIdx{"setup.topic index", 0, 0, 0};
  pangolin::Var<std::string> topicName{"setup.topic", "- none -",
                                       pangolin::META_FLAG_READONLY};
  pangolin::Var<std::string> topicCount{"setup.topics found", "0",
                                        pangolin::META_FLAG_READONLY};
  pangolin::Var<bool> assignA{"setup.assign topic -> A", false, false};
  pangolin::Var<bool> assignB{"setup.assign topic -> B", false, false};
  pangolin::Var<bool> refresh{"setup.refresh topics", false, false};
  pangolin::Var<bool> clearSlots{"setup.clear slots", false, false};
  pangolin::Var<bool> start{"setup.START CALIBRATION", false, false};
  pangolin::Var<std::string> status{"setup.status",
                                    "1) choose calibration type",
                                    pangolin::META_FLAG_READONLY};
};

// one numeric slider widget
struct NumVar {
  std::unique_ptr<pangolin::Var<double>> v;
  void create(const std::string &name, double init, double lo, double hi) {
    v.reset(new pangolin::Var<double>(name, init, lo, hi));
  }
  double get() const { return v->Get(); }
  void set(double x) { *v = x; }
};

// type-specific parameter widgets (created once per type on first use).
// Widgets are prefilled from the default json files; the user can tweak
// values directly or press a load button to re-read the file.
struct ParamVars {
  bool has_extrinsic = false;
  NumVar roll, pitch, yaw, tx, ty, tz;
  std::unique_ptr<pangolin::Var<bool>> loadExt;
  // matrix as loaded from json; used unchanged while the widgets keep the
  // loaded values (RPY <-> matrix conversion is lossy near gimbal lock)
  Eigen::Matrix4d ext_loaded = Eigen::Matrix4d::Identity();
  bool ext_dirty = false;
  bool has_intrinsic = false;
  NumVar fx, fy, cx, cy, k1, k2, p1, p2, k3, imgw, imgh;
  std::unique_ptr<pangolin::Var<bool>> loadInt;
  bool has_homography = false;
  std::array<NumVar, 9> h;
  std::unique_ptr<pangolin::Var<bool>> loadHom;
  bool has_board = false;
  NumVar boardRows, boardCols, boardSquare;
};

// R = Rz(yaw) * Ry(pitch) * Rx(roll)
inline Eigen::Matrix4d rpyToMatrix(double roll_deg, double pitch_deg,
                                   double yaw_deg, double x, double y,
                                   double z) {
  const double D2R = M_PI / 180.0;
  Eigen::Matrix3d R =
      (Eigen::AngleAxisd(yaw_deg * D2R, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(pitch_deg * D2R, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(roll_deg * D2R, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R;
  T(0, 3) = x;
  T(1, 3) = y;
  T(2, 3) = z;
  return T;
}

inline void matrixToRpy(const Eigen::Matrix4d &T, double &roll_deg,
                        double &pitch_deg, double &yaw_deg, double &x,
                        double &y, double &z) {
  const double R2D = 180.0 / M_PI;
  Eigen::Vector3d ypr = T.block<3, 3>(0, 0).eulerAngles(2, 1, 0);
  yaw_deg = ypr(0) * R2D;
  pitch_deg = ypr(1) * R2D;
  roll_deg = ypr(2) * R2D;
  x = T(0, 3);
  y = T(1, 3);
  z = T(2, 3);
}

class CalibrationApp {
public:
  CalibrationApp(ros::NodeHandle &nh, const std::string &pkg,
                 CalibType preset = CalibType::None)
      : nh_(nh), pkg_(pkg), preset_type_(preset) {}

  int run() {
    const int width = 1600, height = 1000;
    pangolin::CreateWindowAndBind("Multi-Sensor Calibration", width, height);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
    glClearColor(0.12f, 0.12f, 0.14f, 1.0f);

    buildViews();

    // optional UI screenshot for debugging:
    //   MSC_SCREENSHOT=/tmp/shot rosrun ... calibration_node
    // saves /tmp/shot.png after ~2 s
    const char *shot_env = getenv("MSC_SCREENSHOT");
    const std::string shot_path = shot_env ? shot_env : "";
    int frame_count = 0;

    ros::Time last_refresh = ros::Time::now();
    refreshTopics();

    // optional pre-selected calibration type (--type lidar-camera etc.)
    if (preset_type_ != CalibType::None)
      selectType(preset_type_);

    std::cout << "=> multi-sensor calibration tool started\n";
    while (!pangolin::ShouldQuit() && ros::ok()) {
      ros::spinOnce();
      glClearColor(0.12f, 0.12f, 0.14f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      if (page_ == Page::Setup) {
        if ((ros::Time::now() - last_refresh).toSec() > 2.0) {
          refreshTopics();
          last_refresh = ros::Time::now();
        }
        setupTick();
      } else if (calibrator_) {
        calibrator_->renderFrame();
        if (calibrator_->backRequested()) {
          backToSetup();
        }
      }

      pangolin::FinishFrame();
      if (!shot_path.empty() && ++frame_count == 120) {
        pangolin::SaveWindowOnRender(shot_path);
        std::cout << "[screenshot] " << shot_path << ".png" << std::endl;
      }
      usleep(1000);
    }

    ros::shutdown();
    if (calibrator_ && type_ != CalibType::CameraIntrinsic) {
      std::cout << "\nFinal Transfromation Matrix:\n"
                << calibrator_->result() << std::endl;
    }
    // Explicit teardown in a defined order: calibrator first (it still
    // touches views), then the Pangolin window/context (which deletes the
    // views it owns) - mirrors the original nodes' window-close exit path.
    calibrator_.reset();
    pangolin::DestroyWindow("Multi-Sensor Calibration");
    std::cout << "[exit] done." << std::endl;
    return 0;
  }

private:
  // ------------------------------------------------------------ view setup
  void buildViews() {
    // setup panel (left column)
    setupPanel_ = &pangolin::CreatePanel("setup");
    setupPanel_->SetBounds(pangolin::Attach::Pix(30), 1.0, 0.0,
                           pangolin::Attach::Pix(320));

    // (setup page: only the left panel; middle and right stay black until
    // a calibration is started)

    // calibration page views (hidden until started)
    cam3d_ = new pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(-100, 0, 0, 0, 0, 0, 0.707, 0.0, 0.707));
    views_.cam3d = cam3d_;
    views_.view3d = &pangolin::Display("calib3d");
    views_.view3d->SetBounds(0.0, 1.0, pangolin::Attach::Pix(270), 1.0,
                             -1.0 * (double)1920 / 1080)
        .SetHandler(new pangolin::Handler3D(*cam3d_));
    views_.view3d->Show(false);

    views_.viewImg = &pangolin::Display("calibimg");
    views_.viewImg->SetBounds(0.0, 1.0, pangolin::Attach::Pix(270), 1.0, 1.0);
    views_.viewImg->Show(false);

    views_.viewBv = &pangolin::Display("calibbv");
    views_.viewBv->SetBounds(0.0, 1.0, 0.65, 1.0, 1.0);
    views_.viewBv->Show(false);

    setupVars_.reset(new SetupVars());

    // exit button LAST so it is drawn (and hit-tested) on top
    exitBtn_ = new ExitButtonView();
    exitBtn_->SetBounds(pangolin::Attach::ReversePix(38),
                        pangolin::Attach::ReversePix(6),
                        pangolin::Attach::ReversePix(118),
                        pangolin::Attach::ReversePix(8));
    pangolin::DisplayBase().AddDisplay(*exitBtn_);

    // Views added after CreateWindowAndBind never received the initial
    // window-resize layout pass: force a full relayout so every view gets a
    // valid viewport (otherwise garbage viewports garble the whole UI).
    relayout();
  }

  void relayout() {
    pangolin::DisplayBase().Resize(pangolin::DisplayBase().GetBounds());
  }

  // ------------------------------------------------------- topic discovery
  void refreshTopics() {
    ros::master::V_TopicInfo infos;
    if (!ros::master::getTopics(infos)) {
      std::cout << "[topics] could not query ROS master (roscore running?)"
                << std::endl;
      return;
    }
    std::vector<TopicRow> rows;
    for (const auto &ti : infos) {
      if (ti.datatype == "sensor_msgs/PointCloud2")
        rows.push_back({ti.name, ti.datatype, false, false});
      else if (ti.datatype == "visualization_msgs/MarkerArray")
        rows.push_back({ti.name, ti.datatype, false, true});
      else if (ti.datatype == "sensor_msgs/Image")
        rows.push_back({ti.name, ti.datatype, true, false});
    }
    std::sort(rows.begin(), rows.end(),
              [](const TopicRow &a, const TopicRow &b) {
                return a.name < b.name;
              });
    topic_rows_ = std::move(rows);
    if (setupVars_) {
      setupVars_->topicCount =
          std::to_string(topic_rows_.size()) + "  (PC2+MRK+IMG)";
      const int max_idx = std::max<int>(0, topic_rows_.size() - 1);
      setupVars_->topicIdx.Meta().range[1] = max_idx;
      if (setupVars_->topicIdx.Get() > max_idx)
        setupVars_->topicIdx = max_idx;
    }
    if (topic_rows_.size() != last_topic_count_) {
      std::cout << "[topics] " << topic_rows_.size()
                << " PointCloud2/MarkerArray/Image topics discovered"
                << std::endl;
      last_topic_count_ = topic_rows_.size();
    }
  }

  // text shown next to the topic index slider
  void updateTopicLabel() {
    int idx = setupVars_->topicIdx.Get();
    if (idx < 0 || idx >= (int)topic_rows_.size()) {
      setupVars_->topicName = "- none -";
      return;
    }
    const TopicRow &t = topic_rows_[idx];
    const char *tag = t.is_image ? "IMG" : (t.is_marker ? "MRK" : "PC2");
    setupVars_->topicName = std::string("[") + tag + "] " + t.name;
  }

  // assign the topic currently selected by the slider to a slot
  void assignTopicToSlot(int slot) {
    if (type_ == CalibType::None) {
      setupVars_->status = "select a calibration type first!";
      return;
    }
    if (slot >= (int)cfg_.slots.size()) {
      setupVars_->status = "this calibration type has no slot B";
      return;
    }
    int idx = setupVars_->topicIdx.Get();
    if (idx < 0 || idx >= (int)topic_rows_.size()) {
      setupVars_->status = "no topic selected (check topic index)";
      return;
    }
    const TopicRow &t = topic_rows_[idx];
    SensorSlot &dst = cfg_.slots[slot];
    if (dst.wants_image != t.is_image) {
      setupVars_->status = "topic type mismatch: slot " +
                           std::string(slot == 0 ? "A" : "B") + " needs " +
                           (dst.wants_image ? "Image" : "PointCloud2");
      return;
    }
    dst.topic = t.name;
    dst.datatype = t.datatype;
    std::cout << "[setup] " << dst.label << " <- " << t.name << " ("
              << t.datatype << ")" << std::endl;
    updateSlotLabels();
  }

  // ------------------------------------------------------------ selection
  void selectType(CalibType t) {
    type_ = t;
    cfg_ = makeTypeConfigs(t, pkg_);
    for (auto &s : cfg_.slots)
      s.topic.clear();

    ensureParamPanel(t);
    // show only the params panel of the current type
    for (auto &kv : paramPanels_)
      kv.second->Show(kv.first == t && kv.first != CalibType::None);

    setupVars_->type = cfg_.name;
    setupVars_->status = "2) pick topic index, assign -> A (then B)";
    updateSlotLabels();
    std::cout << "[setup] type selected: " << cfg_.name << std::endl;
  }

  void ensureParamPanel(CalibType t) {
    if (paramPanels_.count(t))
      return;
    ParamVars pv;
    std::string ns;
    switch (t) {
    case CalibType::LidarLidar:
    case CalibType::RadarLidar:
    case CalibType::LidarCamera:
    case CalibType::RadarCamera: {
      ns = (t == CalibType::LidarLidar) ? "p.ll"
           : (t == CalibType::RadarLidar) ? "p.rl"
           : (t == CalibType::LidarCamera) ? "p.lc" : "p.rc";
      // extrinsic as roll/pitch/yaw + translation
      pv.has_extrinsic = true;
      pv.roll.create(ns + ".roll (deg)", 0, -180, 180);
      pv.pitch.create(ns + ".pitch (deg)", 0, -180, 180);
      pv.yaw.create(ns + ".yaw (deg)", 0, -180, 180);
      pv.tx.create(ns + ".x (m)", 0, -50, 50);
      pv.ty.create(ns + ".y (m)", 0, -50, 50);
      pv.tz.create(ns + ".z (m)", 0, -50, 50);
      pv.loadExt.reset(
          new pangolin::Var<bool>(ns + ".load extrinsic json", false, false));
      if (t == CalibType::LidarCamera || t == CalibType::RadarCamera) {
        // camera intrinsic
        pv.has_intrinsic = true;
        pv.fx.create(ns + ".fx", 0, 0, 5000);
        pv.fy.create(ns + ".fy", 0, 0, 5000);
        pv.cx.create(ns + ".cx", 0, 0, 4000);
        pv.cy.create(ns + ".cy", 0, 0, 3000);
        pv.k1.create(ns + ".k1", 0, -1, 1);
        pv.k2.create(ns + ".k2", 0, -1, 1);
        pv.p1.create(ns + ".p1", 0, -1, 1);
        pv.p2.create(ns + ".p2", 0, -1, 1);
        pv.k3.create(ns + ".k3", 0, -1, 1);
        pv.imgw.create(ns + ".img width", 0, 0, 4000);
        pv.imgh.create(ns + ".img height", 0, 0, 3000);
        pv.loadInt.reset(
            new pangolin::Var<bool>(ns + ".load intrinsic json", false, false));
      }
      if (t == CalibType::RadarCamera) {
        // image <-> ground homography matrix (row major H00..H22)
        pv.has_homography = true;
        const char *names[9] = {"H00", "H01", "H02", "H10", "H11",
                                "H12", "H20", "H21", "H22"};
        for (int i = 0; i < 9; i++)
          pv.h[i].create(ns + "." + names[i], (i == 0 || i == 4 || i == 8) ? 1 : 0,
                         -5, 5);
        pv.loadHom.reset(new pangolin::Var<bool>(
            ns + ".load homography json", false, false));
      }
      break;
    }
    case CalibType::CameraIntrinsic:
      ns = "p.ci";
      pv.has_board = true;
      pv.boardRows.create(ns + ".board rows", 8, 2, 30);
      pv.boardCols.create(ns + ".board cols", 11, 2, 30);
      pv.boardSquare.create(ns + ".square size(m)", 0.015, 0.001, 0.5);
      break;
    default:
      return;
    }
    pangolin::View &panel = ensurePanel(ns, 320, 660);
    paramPanels_[t] = &panel;
    paramVars_[t] = std::move(pv);
    // prefill numeric widgets from the default json files (if present)
    loadExtrinsicWidgets(t, true);
    loadIntrinsicWidgets(t, true);
    loadHomographyWidgets(t, true);
    loadBoardWidgets(t);
    // newly created panel must not show before the user selected the type
    panel.Show(false);
  }

  // ---- json -> widget loaders (prefill & explicit load button) ----
  void loadExtrinsicWidgets(CalibType t, bool quiet = false) {
    if (!paramVars_.count(t))
      return;
    ParamVars &pv = paramVars_[t];
    if (!pv.has_extrinsic)
      return;
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    if (!cfg_.default_extrinsic_json.empty())
      LoadExtrinsic(cfg_.default_extrinsic_json, m);
    pv.ext_loaded = m;
    pv.ext_dirty = false;
    double r, p, y, x, ty, tz;
    matrixToRpy(m, r, p, y, x, ty, tz);
    pv.roll.set(r);
    pv.pitch.set(p);
    pv.yaw.set(y);
    pv.tx.set(x);
    pv.ty.set(ty);
    pv.tz.set(tz);
    std::cout << "[setup] extrinsic json -> widgets: roll=" << pv.roll.get()
              << " pitch=" << pv.pitch.get() << " yaw=" << pv.yaw.get()
              << " t=(" << pv.tx.get() << "," << pv.ty.get() << ","
              << pv.tz.get() << ")" << std::endl;
    (void)quiet;
  }

  void loadIntrinsicWidgets(CalibType t, bool quiet = false) {
    if (!paramVars_.count(t))
      return;
    ParamVars &pv = paramVars_[t];
    if (!pv.has_intrinsic)
      return;
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    std::vector<double> dist;
    int w = 0, h = 0;
    LoadIntrinsic(pkg_ + "/data/intrinsic_camera.json", K, dist, w, h);
    pv.fx.set(K(0, 0));
    pv.fy.set(K(1, 1));
    pv.cx.set(K(0, 2));
    pv.cy.set(K(1, 2));
    auto d = [&](int i) { return i < (int)dist.size() ? dist[i] : 0.0; };
    pv.k1.set(d(0));
    pv.k2.set(d(1));
    pv.p1.set(d(2));
    pv.p2.set(d(3));
    pv.k3.set(d(4));
    pv.imgw.set(w);
    pv.imgh.set(h);
    std::cout << "[setup] intrinsic json -> widgets: fx=" << pv.fx.get()
              << " fy=" << pv.fy.get() << " cx=" << pv.cx.get()
              << " cy=" << pv.cy.get() << " " << w << "x" << h
              << " dist=[" << pv.k1.get() << "," << pv.k2.get() << ","
              << pv.p1.get() << "," << pv.p2.get() << "," << pv.k3.get()
              << "]" << std::endl;
    (void)quiet;
  }

  void loadHomographyWidgets(CalibType t, bool quiet = false) {
    if (!paramVars_.count(t))
      return;
    ParamVars &pv = paramVars_[t];
    if (!pv.has_homography)
      return;
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
    LoadHomography(pkg_ + "/data/camera_homography.json", H);
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
        pv.h[3 * r + c].set(H(r, c));
    if (!quiet)
      std::cout << "[setup] homography json -> widgets" << std::endl;
  }

  void loadBoardWidgets(CalibType t) {
    if (!paramVars_.count(t))
      return;
    ParamVars &pv = paramVars_[t];
    if (!pv.has_board)
      return;
    // defaults from camera_intrinsic_calibration_config.json if present
    int rows = 8, cols = 11;
    double square = 0.015;
    std::ifstream in(pkg_ + "/data/camera_intrinsic_calibration_config.json");
    if (in.good()) {
      Json::Reader reader;
      Json::Value root;
      if (reader.parse(in, root, false) && root.isMember("camera_calibration")) {
        Json::Value c = root["camera_calibration"];
        if (c.isMember("num_row"))
          rows = c["num_row"].asInt();
        if (c.isMember("num_col"))
          cols = c["num_col"].asInt();
        if (c.isMember("square_size"))
          square = c["square_size"].asDouble();
      }
    }
    pv.boardRows.set(rows);
    pv.boardCols.set(cols);
    pv.boardSquare.set(square);
  }

  void updateSlotLabels() {
    auto fmt = [](const SensorSlot &s) {
      return s.label + " : " +
             (s.topic.empty() ? std::string("(pick a topic)") : s.topic);
    };
    if (cfg_.slots.size() > 0)
      setupVars_->slotA = fmt(cfg_.slots[0]);
    if (cfg_.slots.size() > 1)
      setupVars_->slotB = fmt(cfg_.slots[1]);
    else
      setupVars_->slotB = "- unused -";
    bool all = !cfg_.slots.empty();
    for (auto &s : cfg_.slots)
      all = all && !s.topic.empty();
    if (all)
      setupVars_->status = "ready -> press START CALIBRATION";
  }

  // ------------------------------------------------------------ setup tick
  void setupTick() {
    if (pangolin::Pushed(setupVars_->btnLL))
      selectType(CalibType::LidarLidar);
    if (pangolin::Pushed(setupVars_->btnRL))
      selectType(CalibType::RadarLidar);
    if (pangolin::Pushed(setupVars_->btnLC))
      selectType(CalibType::LidarCamera);
    if (pangolin::Pushed(setupVars_->btnRC))
      selectType(CalibType::RadarCamera);
    if (pangolin::Pushed(setupVars_->btnCI))
      selectType(CalibType::CameraIntrinsic);
    // param panel load buttons (only for the selected type)
    if (type_ != CalibType::None && paramVars_.count(type_)) {
      ParamVars &pv = paramVars_[type_];
      if (pv.has_extrinsic) {
        NumVar *extSliders[6] = {&pv.roll, &pv.pitch, &pv.yaw,
                                 &pv.tx,  &pv.ty,  &pv.tz};
        for (NumVar *nv : extSliders)
          if (nv->v->GuiChanged())
            pv.ext_dirty = true;
      }
      if (pv.loadExt && pangolin::Pushed(*pv.loadExt))
        loadExtrinsicWidgets(type_);
      if (pv.loadInt && pangolin::Pushed(*pv.loadInt))
        loadIntrinsicWidgets(type_);
      if (pv.loadHom && pangolin::Pushed(*pv.loadHom))
        loadHomographyWidgets(type_);
    }
    updateTopicLabel();
    if (pangolin::Pushed(setupVars_->assignA))
      assignTopicToSlot(0);
    if (pangolin::Pushed(setupVars_->assignB))
      assignTopicToSlot(1);
    if (pangolin::Pushed(setupVars_->refresh))
      refreshTopics();
    if (pangolin::Pushed(setupVars_->clearSlots)) {
      for (auto &s : cfg_.slots)
        s.topic.clear();
      updateSlotLabels();
      setupVars_->status = "slots cleared";
    }
    if (pangolin::Pushed(setupVars_->start))
      startCalibration();
  }

  void startCalibration() {
    if (type_ == CalibType::None) {
      setupVars_->status = "select a calibration type first!";
      return;
    }
    for (auto &s : cfg_.slots) {
      if (s.topic.empty()) {
        setupVars_->status = "slot '" + s.label + "' has no topic";
        return;
      }
    }
    // topic type must still match
    ros::master::V_TopicInfo infos;
    ros::master::getTopics(infos);
    for (auto &s : cfg_.slots) {
      bool found = false;
      for (auto &ti : infos) {
        if (ti.name != s.topic)
          continue;
        bool is_img = ti.datatype == "sensor_msgs/Image";
        bool is_pc2 = ti.datatype == "sensor_msgs/PointCloud2";
        bool is_mrk = ti.datatype == "visualization_msgs/MarkerArray";
        if ((s.wants_image && is_img) ||
            (!s.wants_image && (is_pc2 || is_mrk)))
          found = true;
      }
      if (!found) {
        setupVars_->status =
            "topic " + s.topic + " not publishing " +
            std::string(s.wants_image ? "Image" : "PointCloud2/MarkerArray");
        return;
      }
    }

    // gather parameters from the widgets (no file access here)
    CalibParams params;
    ParamVars &pv = paramVars_[type_];
    if (pv.has_extrinsic) {
      // untouched widgets -> exact json matrix; edited -> rebuild from RPY
      bool untouched =
          !pv.ext_dirty && pv.ext_loaded != Eigen::Matrix4d::Zero();
      params.extrinsic =
          untouched
              ? pv.ext_loaded
              : rpyToMatrix(pv.roll.get(), pv.pitch.get(), pv.yaw.get(),
                            pv.tx.get(), pv.ty.get(), pv.tz.get());
      std::cout << "[setup] initial extrinsic ("
                << (untouched ? "json" : "widgets") << "):\n"
                << params.extrinsic << std::endl;
    }
    if (pv.has_intrinsic) {
      params.fx = pv.fx.get();
      params.fy = pv.fy.get();
      params.cx = pv.cx.get();
      params.cy = pv.cy.get();
      params.k1 = pv.k1.get();
      params.k2 = pv.k2.get();
      params.p1 = pv.p1.get();
      params.p2 = pv.p2.get();
      params.k3 = pv.k3.get();
      params.img_w = static_cast<int>(pv.imgw.get());
      params.img_h = static_cast<int>(pv.imgh.get());
      if (params.fx < 10 || params.fy < 10 || params.img_w < 2 ||
          params.img_h < 2) {
        setupVars_->status =
            "invalid intrinsic: set fx/fy/img size (or load json)";
        return;
      }
    }
    if (pv.has_homography) {
      for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
          params.homography(r, c) = pv.h[3 * r + c].get();
      if (std::abs(params.homography(2, 2)) < 1e-9) {
        setupVars_->status = "invalid homography: H22 must not be 0";
        return;
      }
    }
    if (pv.has_board) {
      params.board_rows = static_cast<int>(pv.boardRows.get());
      params.board_cols = static_cast<int>(pv.boardCols.get());
      params.board_square_size = pv.boardSquare.get();
      if (params.board_rows < 2 || params.board_cols < 2 ||
          params.board_square_size <= 0) {
        setupVars_->status = "invalid chessboard parameters";
        return;
      }
    }

    // leave the setup page
    setupPanel_->Show(false);
    for (auto &kv : paramPanels_)
      kv.second->Show(false);

    SharedViews views = views_;
    switch (type_) {
    case CalibType::LidarLidar:
      calibrator_.reset(new LidarLidarCalibrator(nh_, pkg_, views));
      break;
    case CalibType::RadarLidar:
      calibrator_.reset(new RadarLidarCalibrator(nh_, pkg_, views));
      break;
    case CalibType::LidarCamera:
      calibrator_.reset(new LidarCameraCalibrator(nh_, pkg_, views));
      break;
    case CalibType::RadarCamera:
      calibrator_.reset(new RadarCameraCalibrator(nh_, pkg_, views));
      break;
    case CalibType::CameraIntrinsic:
      calibrator_.reset(new CameraIntrinsicCalibrator(nh_, pkg_, views));
      break;
    default:
      break;
    }
    calibrator_->start(cfg_, params);
    pangolin::DisplayBase().Resize(pangolin::DisplayBase().GetBounds());
    page_ = Page::Calibration;
    std::cout << "\n=> calibration started: " << cfg_.name << "\n"
              << "   (topics: ";
    for (size_t i = 0; i < cfg_.slots.size(); i++)
      std::cout << cfg_.slots[i].topic
                << (i + 1 < cfg_.slots.size() ? ", " : "");
    std::cout << ")\n"
              << std::endl;
  }

  void backToSetup() {
    std::cout << "[setup] back to setup page" << std::endl;
    if (calibrator_)
      std::cout << "final matrix:\n" << calibrator_->result() << std::endl;
    calibrator_.reset(); // destructor stops subscriptions and hides views
    page_ = Page::Setup;
    setupPanel_->Show();
    for (auto &kv : paramPanels_)
      kv.second->Show(kv.first == type_);
    refreshTopics();
  }

  ros::NodeHandle &nh_;
  std::string pkg_;
  Page page_ = Page::Setup;
  CalibType type_ = CalibType::None;
  TypeConfig cfg_;
  int last_topic_count_ = -1;
  std::vector<TopicRow> topic_rows_;
  CalibType preset_type_ = CalibType::None;

  pangolin::View *setupPanel_ = nullptr;
  ExitButtonView *exitBtn_ = nullptr;
  pangolin::OpenGlRenderState *cam3d_ = nullptr;
  SharedViews views_;
  std::unique_ptr<SetupVars> setupVars_;
  std::map<CalibType, pangolin::View *> paramPanels_;
  std::map<CalibType, ParamVars> paramVars_;
  std::unique_ptr<CalibratorBase> calibrator_;
};

} // namespace

int main(int argc, char **argv) {
  signal(SIGSEGV, fatalSignalHandler);
  signal(SIGABRT, fatalSignalHandler);
  ros::init(argc, argv, "multi_sensor_calibration");

  CalibType preset = CalibType::None;
  for (int i = 1; i < argc - 1; i++) {
    if (std::string(argv[i]) != "--type")
      continue;
    std::string t = argv[i + 1];
    if (t == "lidar-lidar")
      preset = CalibType::LidarLidar;
    else if (t == "radar-lidar")
      preset = CalibType::RadarLidar;
    else if (t == "lidar-camera")
      preset = CalibType::LidarCamera;
    else if (t == "radar-camera")
      preset = CalibType::RadarCamera;
    else if (t == "camera-intrinsic")
      preset = CalibType::CameraIntrinsic;
    else
      std::cerr << "[arg] unknown --type '" << t << "' (lidar-lidar | "
                   "radar-lidar | lidar-camera | radar-camera | "
                   "camera-intrinsic)"
                << std::endl;
  }

  std::string pkg = ros::package::getPath("multi_sensor_calibration");
  ros::NodeHandle nh;
  CalibrationApp app(nh, pkg, preset);
  return app.run();
}
