#include <boost/math/constants/constants.hpp>

#include "Utility.hpp"

namespace perception
{
TransformInfo getTransformInfo(const Eigen::Matrix4d& matrix)
{

    TransformInfo info;
    // 提取平移向量
    info[0] = matrix(0, 3);
    info[1] = matrix(1, 3);
    info[2] = matrix(2, 3);

    // 提取旋转矩阵
    Eigen::Matrix3d rotationMatrix = matrix.block<3, 3>(0, 0);

    // 将旋转矩阵转换为欧拉角
    Eigen::Vector3d eulerAngles = rotationMatrix.eulerAngles(0, 1, 2); // XYZ顺序

    info[3] = eulerAngles[0];
    info[4] = eulerAngles[1];
    info[5] = eulerAngles[2];

    return info;
}
}  // namespace perception
