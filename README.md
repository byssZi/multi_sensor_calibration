# multi_sensor_calibration

ROS 下的多传感器标定工具（**统一单节点版本**）：一个节点、一个界面完成
lidar↔lidar、radar↔lidar、lidar↔camera、radar↔camera、camera 内参
共 5 种标定。原 lidar↔imu 标定及其依赖（GeographicLib）已移除。

# 环境准备

- ROS Noetic（Ubuntu 20.04）
- OpenCV 4.2、Eigen、PCL、jsoncpp
- Pangolin v0.6（本机若无系统安装，可编译安装到用户目录，编译时通过
  `CMAKE_PREFIX_PATH` 指定）
- ceres：随包源码编译（`ceres/` 目录，相机内参精标定依赖）

# 使用方法

## Step1 编译

```bash
catkin_make -j8 -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="<pangolin安装前缀>;/opt/ros/noetic" \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

## Step2 启动（不再需要改 launch 选话题）

```bash
roslaunch multi_sensor_calibration calibration.launch
```

启动后先进入**主界面**（左侧为控制面板，中间与右侧初始为黑屏）：

1. 左侧面板点选标定类型（1 lidar↔lidar / 2 radar↔lidar /
   3 lidar↔camera / 4 radar↔camera / 5 camera intrinsic）；
2. 话题选择（同样在左侧面板，话题列表每 2 秒自动刷新）：
   拖动 `topic index` 滑条选择话题，`topic` 一行实时显示所选话题
   名称及类型（`[PC2]`/`[MRK]`/`[IMG]`），然后点 `assign topic -> A` /
   `assign topic -> B` 填入传感器槽位；点类槽位（激光/雷达）可接受
   `sensor_msgs/PointCloud2` 或 `visualization_msgs/MarkerArray`
   （MarkerArray 每个 marker 的 pose.position 转为一个点，与原雷达
   节点一致）；
3. 中间参数面板按类型显示**数值参数控件**（选定类型后出现，初值自动
   从 `data/` 下默认 json 预填，可直接拖动滑条修改，也可点 load 按钮
   重新读取文件）：
   - 外参类（全部 4 种）：roll/pitch/yaw(°) + x/y/z(m) 六个滑条 +
     `load extrinsic json` 按钮（未改动时标定使用 json 原矩阵，无转换
     损失；改动后按 RPY 重建）
   - lidar↔camera / radar↔camera：fx/fy/cx/cy、畸变 k1/k2/p1/p2/k3、
     图像宽高 + `load intrinsic json` 按钮
   - radar↔camera：单应矩阵 H00–H22 + `load homography json` 按钮
   - camera intrinsic：棋盘格行/列数、方格边长(m)（默认 8×11、0.015）
4. 点击 `START CALIBRATION` 进入对应标定界面（雷达类话题同样接收
   PointCloud2）。启动时可加 `--type lidar-lidar|radar-lidar|
   lidar-camera|radar-camera|camera-intrinsic` 预选标定类型。

## Step3 标定

标定界面与原各节点一致：左侧控制面板 + 右侧点云/图像视图，支持按钮或
键盘手动微调外参、自动标定（NDT/ICP/优化器）、保存结果；
点 `Back to Setup` 可返回主界面换一种标定，**点右上角红色 Exit 按钮
即可退出程序**（无需终端 Ctrl-C）。

   | 外参调整 | 键盘输入 | 外参调整 | 键盘输入 |
   | :--------------: | :------------: | :--------------: | :------------: |
   |    +x degree     |       q        |    -x degree     |       a        |
   |    +y degree     |       w        |    -y degree     |       s        |
   |    +z degree     |       e        |    -z degree     |       d        |
   |     +x trans     |       r        |     -x trans     |       f        |
   |     +y trans     |       t        |     -y trans     |       g        |
   |     +z trans     |       y        |     -z trans     |       h        |

# 与旧版的差异

- 5 个独立节点合并为 `calibration_node`（旧节点源码已删除）；
- 雷达类话题统一接收 `sensor_msgs/PointCloud2`（旧 radar 节点接收
  MarkerArray）；
- radar↔camera 标定开始后，图像会**冻结为首帧静止画面**，直接在
  图像视图上依次**点选 4 个车道线点**（左下、左上、右下、右上，选点
  实时叠加显示），选满 4 个自动计算鸟瞰图单应并恢复实时画面；
  `repick points` 会冻结当前帧重新选点；左侧面板 `repick points` 重新选点、`default region`
  使用默认地面区域（x∈[-5,5]m、y∈[0,100]m）；原版的 OpenCV 弹窗选点
  已移除（GTK 弹窗与 Pangolin 混用会导致鼠标指针消失）；
- 移除 lidar↔imu 标定、全部自定义 msg、GeographicLib 依赖；
- 相机内参标定时棋盘格参数可直接在主界面填写，结果文件仍在
  `data/<type>_extrinsic_calibration/` 下。

# 调试截图

需要留存界面画面（排查显示问题时）：

```bash
MSC_SCREENSHOT=/tmp/shot rosrun multi_sensor_calibration calibration_node
# 约 2 秒后生成 /tmp/shot.png
```
