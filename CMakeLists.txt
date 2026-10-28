cmake_minimum_required(VERSION 3.0.2)
project(multi_sensor_calibration)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  tf
  visualization_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Pangolin REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)

# bundled ceres (needed by the camera-intrinsic refinement)
add_subdirectory(ceres)
include_directories(${PROJECT_SOURCE_DIR}/ceres/include)

catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport pcl_ros roscpp rospy sensor_msgs
    tf visualization_msgs geometry_msgs std_msgs nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${JSONCPP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)
link_directories(${Pangolin_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${JSONCPP_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${EIGEN_DEFINITIONS})
add_definitions(${PCL_DEFINITIONS})

# unified calibration node (single executable with setup UI + all five
# calibration workflows: lidar-lidar / radar-lidar / lidar-camera /
# radar-camera / camera-intrinsic)
add_executable(calibration_node
  src/calibration_node.cpp
  include/autocalib/lidar2camera/CalibrationHandler.cpp
  include/autocalib/lidar2camera/HistogramHandler.cpp
  include/autocalib/lidar2camera/ProbabilityHandler.cpp
  include/autocalib/lidar2camera/Types.cpp
  include/autocalib/lidar2camera/utils/Utility.cpp
  include/autocalib/lidar2cameraV2/Calibrator.cc
  include/autocalib/lidar2cameraV2/Constants.cc
  include/autocalib/lidar2cameraV2/Core.cc
  include/autocalib/lidar2cameraV2/Rand.cc
  include/autocalib/lidar2cameraV2/StopWatch.cc
  include/autocalib/lidar2cameraV2/Types.cc
  include/autocalib/camera_intrinsic/CalibrationHandler.cpp
  include/autocalib/camera_intrinsic/Extrinsics.cpp
  include/autocalib/camera_intrinsic/Homography.cpp
  include/autocalib/camera_intrinsic/LensDistortionModel.cpp
  include/autocalib/camera_intrinsic/TotalRefinement.cpp
  include/autocalib/camera_intrinsic/Undistortion.cpp
  include/autocalib/camera_intrinsic/utils/CalibUtils.cpp
)
target_link_libraries(calibration_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Pangolin_LIBRARIES}
  jsoncpp_lib
  ${OpenCV_LIBS}
  ceres
)

install(TARGETS calibration_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
