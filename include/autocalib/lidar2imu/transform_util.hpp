/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
#ifndef TRANSFORM_UTIL_HPP_
#define TRANSFORM_UTIL_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

inline double rad2deg(double radians) { return radians * 180.0 / M_PI; }
inline double deg2rad(double degrees) { return degrees * M_PI / 180.0; }

class TransformUtil {
public:
  TransformUtil() = default;
  ~TransformUtil() = default;

  static Eigen::Matrix4d GetMatrix(const Eigen::Vector3d &translation,
                                   const Eigen::Matrix3d &rotation) {
    Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
    ret.block<3, 1>(0, 3) = translation;
    ret.block<3, 3>(0, 0) = rotation;
    return ret;
  }

  static Eigen::Matrix4d Matrix4FloatToDouble(const Eigen::Matrix4f &matrix) {
    Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
    ret << matrix(0), matrix(4), matrix(8), matrix(12), matrix(1), matrix(5),
        matrix(9), matrix(13), matrix(2), matrix(6), matrix(10), matrix(14),
        matrix(3), matrix(7), matrix(11), matrix(15);
    return ret;
  }

  static Eigen::Matrix4d GetDeltaT(const float var[6]) {
    auto deltaR = Eigen::Matrix3d(
        Eigen::AngleAxisd(deg2rad(var[2]), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(deg2rad(var[1]), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(deg2rad(var[0]), Eigen::Vector3d::UnitX()));
    Eigen::Matrix4d deltaT = Eigen::Matrix4d::Identity();
    deltaT.block<3, 3>(0, 0) = deltaR;
    deltaT(0, 3) = var[3];
    deltaT(1, 3) = var[4];
    deltaT(2, 3) = var[5];
    return deltaT;
  }

  static Eigen::Matrix3d GetQua(const double yaw, const double pitch, const double roll) {
    double yaw_ = -yaw;
    // if(yaw_ > 180)
    // {
    //   yaw_ -=360;
    // }
    // else if(yaw < -180)
    // {
    //   yaw_ +=360;
    // }
    double cx = cos(deg2rad(-pitch));
    double sx = sin(deg2rad(-pitch));
    double cy = cos(deg2rad(-roll));
    double sy = sin(deg2rad(-roll));
    double cz = cos(deg2rad(yaw_));
    double sz = sin(deg2rad(yaw_));
    Eigen::Matrix3d R;
    R << cy*cz + sx*sy*sz, -cy*sz + sx*sy*cz, cx*sy,
         cx*sz, cx*cz, -sx,
         -sy*cz + cy*sx*sz, sy*sz + cy*sx*cz, cx*cy;
    return R;
    // Eigen::Quaterniond q(R);
    // // Eigen::Quaterniond q = Eigen::Quaterniond(
    // //   Eigen::AngleAxisd(deg2rad(yaw_), Eigen::Vector3d::UnitZ()) *
    // //   Eigen::AngleAxisd(deg2rad(-pitch), Eigen::Vector3d::UnitX()) * 
    // //   Eigen::AngleAxisd(deg2rad(-roll), Eigen::Vector3d::UnitY()));
    // q.normalize();
    // return q;

    // double srx = sin(pitch);
    // double crx = cos(pitch);
    // double sry = sin(roll);
    // double cry = cos(roll);
    // double srz = sin(yaw);
    // double crz = cos(yaw);

    // Eigen::Matrix<double, 3, 3> Rzyx;
    // Rzyx << cry * crz, srx * sry * crz - crx * srz, srx * srz + crx * sry * crz,
    //         cry * srz, crx * crz + srx * sry * srz, crx * sry * srz - srx * crz,
    //         -sry     , srx * cry                  , crx * cry;
    // Eigen::Quaterniond quaternion(Rzyx);
    // quaternion.normalize();
    // return quaternion;

  }
};

#endif // TRANSFORM_UTIL_HPP_
