#include <boost/math/constants/constants.hpp>

#include "CalibrationHandler.hpp"

namespace perception
{
CalibrationHandlerParam getCalibrationHandlerParam(const Eigen::Matrix4d& InitialGuess,
                                                   const std::vector<cv::Mat>& Images,
                                                   const std::vector<pcl::PointCloud<pcl::PointXYZI>>& PointClouds,
                                                   const Eigen::Matrix3d& CameraInfo)
{
    CalibrationHandlerParam param;

    param.InitialGuess = InitialGuess;
    param.Images = Images;
    param.PointClouds = PointClouds;
    param.CameraInfo = CameraInfo;

    param.numBins = 256;
    param.deltaTrans = 0.01;
    param.deltaRotRad = 1.0;
    param.deltaStepFactor = 1.05;
    param.deltaThresh = 0.00001;
    param.gammaTransU = 0.1;
    param.gammaTransL = 0.01;
    param.gammaRotU = 5.0 * boost::math::double_constants::degree;
    param.gammaRotL = 1.0 * boost::math::double_constants::degree;
    param.gammaStepFactor = 1.05;
    param.maxIter = 300;
    param.xMin = 0.0;
    param.xMax = 100.0;
    param.yMin = -50.0;
    param.yMax = 50.0;
    param.zMin = -50.0;
    param.zMax = 50.0;
    param.filterInputImage = true;
    param.filterDiameter = 15;
    param.sigmaColor = 75.0;
    param.sigmaSpace = 75.0;
    param.normalizeMI = true;
    param.probabilityEstimatorType = 1;
    param.useBayes = false;

    return param;
}

}  // namespace perception
