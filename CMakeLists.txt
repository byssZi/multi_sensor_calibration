cmake_minimum_required(VERSION 3.0.2)
project(multi_sensor_calibration)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
# set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
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
  message_filters
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Pangolin REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)

add_subdirectory(ceres)
add_subdirectory(geographiclib-2.1.2)
include_directories(${PROJECT_SOURCE_DIR}/ceres/include)
include_directories(${PROJECT_SOURCE_DIR}/geographiclib-2.1.2/include)



## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   visualization_msgs
# )
add_message_files(
  FILES
  Object.msg
  ObjectList.msg
  Cluster.msg
  ClusterList.msg
  CGI610GNSSMsg.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)
################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES radar_lidar_static_calibration
  CATKIN_DEPENDS cv_bridge image_transport message_generation message_runtime message_filters pcl_ros roscpp rospy sensor_msgs tf visualization_msgs geometry_msgs std_msgs nav_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
include_directories(include ${EIGEN3_INCLUDE_DIR})
include_directories(include ${JSONCPP_INCLUDE_DIRS})
include_directories(include ${OpenCV_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})
include_directories(include ${PCL_INCLUDE_DIRS})
include_directories(include ${Pangolin_INCLUDE_DIRS})
link_directories(${Pangolin_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${JSONCPP_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/radar_lidar_static_calibration.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/radar_lidar_static_calibration_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_radar_lidar_static_calibration.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


add_executable(radar_lidar_calibration_node src/radar_lidar_calibration_node.cpp)
target_link_libraries(radar_lidar_calibration_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Pangolin_LIBRARIES}
  jsoncpp_lib
)
add_dependencies(radar_lidar_calibration_node  ${PROJECT_NAME}_generate_messages_cpp)

add_executable(lidar_lidar_calibration_node src/lidar_lidar_calibration_node.cpp)
target_link_libraries(lidar_lidar_calibration_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Pangolin_LIBRARIES}
  jsoncpp_lib
)

add_executable(lidar_camera_calibration_node src/lidar_camera_calibration_node.cpp
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
                                             include/autocalib/lidar2cameraV2/Types.cc)
target_link_libraries(lidar_camera_calibration_node 
  jsoncpp_lib 
  ${PCL_LIBRARIES} 
  ${Pangolin_LIBRARIES} 
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES}
)

add_executable(radar_camera_calibration_node src/radar_camera_calibration_node.cpp)
target_link_libraries(radar_camera_calibration_node 
  jsoncpp_lib 
  ${PCL_LIBRARIES} 
  ${Pangolin_LIBRARIES} 
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES}
)

add_executable(lidar_imu_calibration_node src/lidar_imu_calibration_node.cpp)
target_link_libraries(lidar_imu_calibration_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Pangolin_LIBRARIES}
  jsoncpp_lib
  ceres
  GeographicLib
)
add_dependencies(lidar_imu_calibration_node  ${PROJECT_NAME}_generate_messages_cpp)

add_executable(camera_intrinsic_calibration_node src/camera_intrinsic_calibration_node.cpp
                                             include/autocalib/camera_intrinsic/CalibrationHandler.cpp
                                             include/autocalib/camera_intrinsic/Extrinsics.cpp
                                             include/autocalib/camera_intrinsic/Homography.cpp
                                             include/autocalib/camera_intrinsic/LensDistortionModel.cpp
                                             include/autocalib/camera_intrinsic/TotalRefinement.cpp
                                             include/autocalib/camera_intrinsic/Undistortion.cpp
                                             include/autocalib/camera_intrinsic/utils/CalibUtils.cpp)
target_link_libraries(camera_intrinsic_calibration_node 
  jsoncpp_lib 
  ${Pangolin_LIBRARIES} 
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES}
  ceres
  GeographicLib
)
