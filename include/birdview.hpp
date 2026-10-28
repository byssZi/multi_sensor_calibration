#pragma once

// Bird-view grid generation for radar<->camera calibration.
//
// The lane points are picked directly in the Pangolin image view (the
// original OpenCV picker window was removed: mixing OpenCV/GTK windows with
// the Pangolin X11 window broke the mouse cursor on WSLg). This file now
// only contains the homography/grid math.

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <vector>

class BirdView {
public:
  Eigen::Matrix3d ground2img_hmat_;
  Eigen::Matrix3d img2ground_hmat_;
  cv::Mat img2bv_hmat_;
  cv::Mat ground2bv_hmat_;
  int width_;
  int height_;

  cv::Size origin_img_size_;
  cv::Mat grid_image_;

  // Pick 4 lane points in the IMAGE (order: left-down, left-up, right-down,
  // right-up); they are mapped to the ground via img2ground_hmat and define
  // the bird-view region. Returns false unless exactly 4 points are given.
  bool initFromImagePoints(const cv::Mat &img, const int &width,
                           const int &height,
                           const Eigen::Matrix3d &img2ground_hmat,
                           const std::vector<cv::Point2f> &picked_pts) {
    if (picked_pts.size() != 4)
      return false;
    origin_img_size_ = img.size();
    width_ = width;
    height_ = height;
    img2ground_hmat_ = img2ground_hmat;

    std::vector<cv::Point2f> ground_pts;
    for (int i = 0; i < 4; i++) {
      Eigen::Vector3d ground_pt =
          img2ground_hmat_ *
          Eigen::Vector3d(picked_pts[i].x, picked_pts[i].y, 1);
      ground_pt /= ground_pt(2);
      ground_pts.push_back(cv::Point2f(ground_pt(0), ground_pt(1)));
    }
    return computeGroundHomo(ground_pts);
  }

  // Automatic initialisation over the default ground rectangle
  // x in [-5, 5] m, y in [0, 100] m.
  void initAuto(const cv::Mat &img, const int &width, const int &height,
                const Eigen::Matrix3d &img2ground_hmat) {
    origin_img_size_ = img.size();
    width_ = width;
    height_ = height;
    img2ground_hmat_ = img2ground_hmat;
    std::vector<cv::Point2f> ground_pts;
    ground_pts.push_back(cv::Point2f(-5, 0));
    ground_pts.push_back(cv::Point2f(-5, 100));
    ground_pts.push_back(cv::Point2f(5, 0));
    ground_pts.push_back(cv::Point2f(5, 100));
    computeGroundHomo(ground_pts);
  }

  // Compute the bird-view homography from a ground-plane rectangle given in
  // the order (left-down, left-up, right-down, right-up) and build the grid.
  bool computeGroundHomo(const std::vector<cv::Point2f> &ground_pts) {
    std::vector<cv::Point2f> bv_pts;
    float left = width_ / 2.8;
    float right = width_ - width_ / 2.8;
    float up = height_ / 2.5;
    float down = height_ - height_ / 25.0;
    bv_pts.push_back(cv::Point2f(left, down));
    bv_pts.push_back(cv::Point2f(left, up));
    bv_pts.push_back(cv::Point2f(right, down));
    bv_pts.push_back(cv::Point2f(right, up));

    ground2bv_hmat_ = cv::findHomography(ground_pts, bv_pts);
    buildGrid(left, right);
    return true;
  }

  void buildGrid(float left, float right) {
    grid_image_ =
        cv::Mat(cv::Size(width_, height_), CV_8UC3, cv::Scalar(235, 235, 235));
    // draw lines
    int gap = (width_ + 10.0) / 10.0;
    for (int i = 0; i < 10; i++) {
      cv::Point start = cv::Point(gap * i, 0);
      cv::Point end = cv::Point(gap * i, height_);
      cv::line(grid_image_, start, end, cv::Scalar(140, 140, 140), 2);
    }
    gap = (height_ + 10.0) / 15.0;
    for (int i = 0; i < 15; i++) {
      cv::Point start = cv::Point(0, gap * i);
      cv::Point end = cv::Point(width_, gap * i);
      cv::line(grid_image_, start, end, cv::Scalar(140, 140, 140), 2);
    }

    // draw lane lines
    cv::line(grid_image_, cv::Point(int(left), 0),
             cv::Point(int(left), height_), cv::Scalar(225, 80, 0), 4);
    cv::line(grid_image_, cv::Point(int(right), 0),
             cv::Point(int(right), height_), cv::Scalar(225, 80, 0), 4);
  }

  cv::Mat drawProjectBirdView(const int &point_size,
                              const std::vector<cv::Point2f> grount_pts) {
    cv::Mat output;
    if (!grid_image_.empty())
      output = grid_image_.clone();
    else
      output = cv::Mat(cv::Size(width_, height_), CV_8UC3,
                       cv::Scalar(235, 235, 235));

    Eigen::Matrix3d hmat;
    cv::cv2eigen(ground2bv_hmat_, hmat);
    for (size_t i = 0; i < grount_pts.size(); i++) {
      Eigen::Vector3d pt(grount_pts[i].x * 0.2, grount_pts[i].y * 0.2, 1.0);
      Eigen::Vector3d bv_pt = hmat * pt;
      bv_pt /= bv_pt(2);
      int xbv = cvRound(bv_pt(0));
      int ybv = cvRound(bv_pt(1));
      cv::Scalar color = cv::Scalar(0, 0, 255);
      cv::circle(output, cv::Point(xbv, ybv), point_size, color, -1);
    }
    return output;
  }
};
