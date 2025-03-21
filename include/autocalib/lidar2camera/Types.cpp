#include "Types.hpp"

namespace perception
{
CameraInfo::CameraInfo(const Eigen::Matrix3d cameraInfo)
    : m_K(Eigen::Matrix3d::Zero())
{
    m_K = cameraInfo;
}

CameraInfo::~CameraInfo()
{
}

const Eigen::Matrix3d& CameraInfo::K() const
{
    return m_K;
}
}  // namespace perception
