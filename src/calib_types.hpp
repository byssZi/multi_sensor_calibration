#pragma once

// Calibration type definitions and runtime configuration for the unified
// calibration node. Everything lives in a single translation unit
// (calibration_node.cpp), so these headers only ever get included once.

#include <string>
#include <vector>

#include <Eigen/Core>

namespace msc {

enum class CalibType {
  None = 0,
  LidarLidar,      // target lidar <-> source lidar   (NDT)
  RadarLidar,      // radar <-> lidar                 (ICP)
  LidarCamera,     // lidar <-> camera
  RadarCamera,     // radar <-> camera (bird view)
  CameraIntrinsic, // camera intrinsic only
};

// One sensor input slot to be filled with a ROS topic.
struct SensorSlot {
  std::string label;    // e.g. "target lidar", "radar", "camera"
  bool wants_image = false; // true: sensor_msgs/Image, false: point-like
  std::string topic;    // chosen topic name, empty if unassigned
  std::string datatype; // "sensor_msgs/PointCloud2" or
                        // "visualization_msgs/MarkerArray" for point slots
};

struct TypeConfig {
  CalibType type = CalibType::None;
  std::string name;                       // display name
  std::vector<SensorSlot> slots;          // filled topics at start
  std::string default_extrinsic_json;     // "" when type has no extrinsic
  bool needs_intrinsic = false;
  bool needs_homography = false;
  bool needs_chessboard = false;
};

inline TypeConfig makeTypeConfigs(CalibType t, const std::string &pkg_loc) {
  TypeConfig c;
  c.type = t;
  switch (t) {
  case CalibType::LidarLidar:
    c.name = "lidar <-> lidar";
    c.slots = {{"A target lidar", false, ""}, {"B source lidar", false, ""}};
    c.default_extrinsic_json = pkg_loc + "/data/extrinsic_lidar2lidar.json";
    break;
  case CalibType::RadarLidar:
    c.name = "radar <-> lidar";
    c.slots = {{"A lidar", false, ""}, {"B radar", false, ""}};
    c.default_extrinsic_json = pkg_loc + "/data/extrinsic_radar2lidar.json";
    break;
  case CalibType::LidarCamera:
    c.name = "lidar <-> camera";
    c.slots = {{"A lidar", false, ""}, {"B camera", true, ""}};
    c.default_extrinsic_json = pkg_loc + "/data/extrinsic_lidar2camera.json";
    c.needs_intrinsic = true;
    break;
  case CalibType::RadarCamera:
    c.name = "radar <-> camera";
    c.slots = {{"A radar", false, ""}, {"B camera", true, ""}};
    c.default_extrinsic_json = pkg_loc + "/data/extrinsic_radar2camera.json";
    c.needs_intrinsic = true;
    c.needs_homography = true;
    break;
  case CalibType::CameraIntrinsic:
    c.name = "camera intrinsic";
    c.slots = {{"A camera", true, ""}};
    break;
  default:
    break;
  }
  return c;
}

// Parameters collected from the setup-page widgets (values only, no file
// paths - files are only used to prefill the widgets).
struct CalibParams {
  // initial extrinsic: built from roll/pitch/yaw [deg] + translation [m]
  Eigen::Matrix4d extrinsic = Eigen::Matrix4d::Identity();
  // camera intrinsic
  double fx = 0, fy = 0, cx = 0, cy = 0;
  double k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;
  int img_w = 0, img_h = 0;
  // image <-> ground homography (radar-camera bird view)
  Eigen::Matrix3d homography = Eigen::Matrix3d::Identity();
  // chessboard (camera intrinsic calibration)
  int board_rows = 9;         // chessboard inner corners (rows)
  int board_cols = 6;         // chessboard inner corners (cols)
  double board_square_size = 15.0; // mm
};

inline std::string fileBaseName(const std::string &path) {
  auto pos = path.find_last_of('/');
  return pos == std::string::npos ? path : path.substr(pos + 1);
}

} // namespace msc
