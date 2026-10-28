#pragma once

// Shared helpers used by all calibrators: intensity colouring, terminal
// keyboard handling, point cloud GL buffers, the manual extrinsic adjuster
// (the 12 rotation/translation modification matrices) and result saving.

#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace msc {

struct RGB {
  unsigned char r, g, b;
};

inline RGB GreyToColorMix(int val) {
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
  return {static_cast<unsigned char>(r), static_cast<unsigned char>(g),
          static_cast<unsigned char>(b)};
}

// Non-blocking stdin check (manual calibration keys work from the terminal).
inline bool kbhit() {
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

// Owns the OpenGL vertex/colour buffers for one point cloud.
class CloudGlBuffers {
public:
  ~CloudGlBuffers() { release(); }

  // Upload a PointXYZI cloud.
  //  colour_mode: intensity-coloured vs grey
  //  red_tint:    grey mode -> (I,0,0), colour mode -> swapped G/B channel
  //               (visual distinction of the transformed "source" cloud)
  void upload(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
              bool colour_mode, bool red_tint = false) {
    rebuild(cloud->points.size());
    const int n = cloud->points.size();
    if (n == 0)
      return;
    std::vector<float> verts(3 * n);
    std::vector<unsigned char> cols(3 * n);
    for (int i = 0; i < n; i++) {
      verts[3 * i + 0] = cloud->points[i].x;
      verts[3 * i + 1] = cloud->points[i].y;
      verts[3 * i + 2] = cloud->points[i].z;
      unsigned char g =
          static_cast<unsigned char>(cloud->points[i].intensity);
      if (colour_mode) {
        RGB c = GreyToColorMix(cloud->points[i].intensity);
        if (red_tint) { // preserve original source-cloud colouring
          cols[3 * i + 0] = c.r;
          cols[3 * i + 1] = c.b;
          cols[3 * i + 2] = c.g;
        } else {
          cols[3 * i + 0] = c.r;
          cols[3 * i + 1] = c.g;
          cols[3 * i + 2] = c.b;
        }
      } else if (red_tint) {
        cols[3 * i + 0] = g;
        cols[3 * i + 1] = 0;
        cols[3 * i + 2] = 0;
      } else {
        cols[3 * i + 0] = g;
        cols[3 * i + 1] = g;
        cols[3 * i + 2] = g;
      }
    }
    vertex_->Upload(verts.data(), sizeof(float) * 3 * n, 0);
    color_->Upload(cols.data(), sizeof(unsigned char) * 3 * n, 0);
    count_ = n;
  }

  // Upload a plain PointXYZ cloud with a fixed colour.
  void upload(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
              unsigned char r, unsigned char g, unsigned char b) {
    rebuild(cloud->points.size());
    const int n = cloud->points.size();
    if (n == 0)
      return;
    std::vector<float> verts(3 * n);
    std::vector<unsigned char> cols(3 * n);
    for (int i = 0; i < n; i++) {
      verts[3 * i + 0] = cloud->points[i].x;
      verts[3 * i + 1] = cloud->points[i].y;
      verts[3 * i + 2] = cloud->points[i].z;
      cols[3 * i + 0] = r;
      cols[3 * i + 1] = g;
      cols[3 * i + 2] = b;
    }
    vertex_->Upload(verts.data(), sizeof(float) * 3 * n, 0);
    color_->Upload(cols.data(), sizeof(unsigned char) * 3 * n, 0);
    count_ = n;
  }

  void draw(int gl_point_size) const {
    if (count_ == 0)
      return;
    glPointSize(gl_point_size);
    color_->Bind();
    glColorPointer(color_->count_per_element, color_->datatype, 0, 0);
    glEnableClientState(GL_COLOR_ARRAY);
    vertex_->Bind();
    glVertexPointer(vertex_->count_per_element, vertex_->datatype, 0, 0);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_POINTS, 0, count_);
    glDisableClientState(GL_VERTEX_ARRAY);
    vertex_->Unbind();
    glDisableClientState(GL_COLOR_ARRAY);
    color_->Unbind();
  }

private:
  void rebuild(size_t n) {
    release();
    vertex_ = new pangolin::GlBuffer(pangolin::GlArrayBuffer, n, GL_FLOAT, 3,
                                     GL_DYNAMIC_DRAW);
    color_ = new pangolin::GlBuffer(pangolin::GlArrayBuffer, n,
                                    GL_UNSIGNED_BYTE, 3, GL_DYNAMIC_DRAW);
    count_ = 0;
  }
  void release() {
    delete vertex_;
    delete color_;
    vertex_ = nullptr;
    color_ = nullptr;
  }

  pangolin::GlBuffer *vertex_ = nullptr;
  pangolin::GlBuffer *color_ = nullptr;
  int count_ = 0;
};

// The 12-step manual extrinsic adjuster shared by every extrinsic calibrator.
class ExtrinsicAdjuster {
public:
  void init(const Eigen::Matrix4d &extrinsic) {
    matrix_ = extrinsic;
    origin_ = extrinsic;
    rebuild();
  }
  void setStep(double degree, double trans_m) {
    degree_step_ = degree;
    trans_step_ = trans_m;
    rebuild();
  }
  void rebuild() {
    modification_list_.assign(12, Eigen::Matrix4d::Identity());
    for (int32_t i = 0; i < 12; i++) {
      std::vector<int> flag(6, 0);
      flag[i / 2] = (i % 2) ? (-1) : 1;
      Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
      Eigen::Matrix3d rot =
          (Eigen::AngleAxisd(flag[0] * degree_step_ / 180.0 * M_PI,
                             Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(flag[1] * degree_step_ / 180.0 * M_PI,
                             Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(flag[2] * degree_step_ / 180.0 * M_PI,
                             Eigen::Vector3d::UnitZ()))
              .toRotationMatrix();
      tmp.block(0, 0, 3, 3) = rot;
      tmp(0, 3) = flag[3] * trans_step_;
      tmp(1, 3) = flag[4] * trans_step_;
      tmp(2, 3) = flag[5] * trans_step_;
      modification_list_[i] = tmp;
    }
  }
  void apply(int idx) { matrix_ = matrix_ * modification_list_[idx]; }
  void reset() { matrix_ = origin_; }
  // Keyboard manual calibration (q/a w/s e/d r/f t/g y/h).
  bool applyKey(int key) {
    static const char table[] = {'q', 'a', 'w', 's', 'e', 'd',
                                 'r', 'f', 't', 'g', 'y', 'h'};
    for (int32_t i = 0; i < 12; i++) {
      if (key == table[i]) {
        apply(i);
        return true;
      }
    }
    return false;
  }
  double degree_step_ = 0.3;
  double trans_step_ = 0.06;
  Eigen::Matrix4d matrix_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d origin_ = Eigen::Matrix4d::Identity();
  std::vector<Eigen::Matrix4d> modification_list_;
};

// Save an extrinsic matrix in the same txt format as the original nodes.
inline void saveExtrinsicTxt(const std::string &file_name,
                             const Eigen::Matrix4d &m,
                             const Eigen::Matrix3d &K = Eigen::Matrix3d(),
                             const std::vector<double> *dist = nullptr,
                             bool has_intrinsic = false) {
  std::ofstream f(file_name);
  if (!f.is_open()) {
    std::cerr << "open file " << file_name << " failed." << std::endl;
    return;
  }
  f << "Extrinsic:" << std::endl;
  f << "R:\n"
    << m(0, 0) << " " << m(0, 1) << " " << m(0, 2) << "\n"
    << m(1, 0) << " " << m(1, 1) << " " << m(1, 2) << "\n"
    << m(2, 0) << " " << m(2, 1) << " " << m(2, 2) << std::endl;
  f << "t: " << m(0, 3) << " " << m(1, 3) << " " << m(2, 3) << std::endl;

  f << "************* json format *************" << std::endl;
  f << "Extrinsic:" << std::endl;
  for (int r = 0; r < 4; r++) {
    f << "[" << m(r, 0) << "," << m(r, 1) << "," << m(r, 2) << "," << m(r, 3)
      << "]";
    if (r < 3)
      f << ",";
    f << std::endl;
  }

  if (has_intrinsic) {
    f << "\nIntrinsic:" << std::endl;
    for (int r = 0; r < 3; r++) {
      f << "[" << K(r, 0) << "," << K(r, 1) << "," << K(r, 2) << "]";
      if (r < 2)
        f << ",";
      f << std::endl;
    }
    if (dist && !dist->empty()) {
      f << "\nDistortion:" << std::endl;
      f << "[";
      for (size_t i = 0; i < dist->size(); i++) {
        f << (*dist)[i];
        if (i != dist->size() - 1)
          f << ",";
      }
      f << "]" << std::endl;
    }
  }
  f.close();
}

} // namespace msc
