# multi_sensor_calibration

一个ros下用于手动-自动结合调整x,y,z,roll,pitch,yaw动态标定lidar，radar，camera的工程，分别为lidar2lidar，radar2lidar，lidar2camera，radar2camera，lidar2imu

# 环境准备

- Cmake
- opencv 
- eigen 
- PCL 
- Pangolin
# 参数输入
将传感器间初始外参与相机内参填入`/data`下对应json文件中
# lidar2lidar
## Step1
```bash
catkin_make
source devel/setup.bash
```
## Step2
发布target_lidar与source_lidar的ros话题，格式均为sensor_msgs::PointCloud2格式，修改launch文件下对应target_lidar与source_lidar话题。
```bash
roslaunch multi_sensor_calibration calibration.launch calib_lidar2lidar:=true
```
## Step3
播放用于标定的rosbag包,进入标定界面：</br>
标定界面由用于手动校准的左侧控制面板和右侧点云界面组成。用户可以通过单击面板中的相应按钮或使用键盘作为输入来调整外部参数，来检查两点云是否对齐。当两个点云对齐时，校准结束，单击保存按钮保存结果。

   | 外参调整 | 键盘输入 | 外参调整 | 键盘输入 |
   | :--------------: | :------------: | :--------------: | :------------: |
   |    +x degree     |       q        |    -x degree     |       a        |
   |    +y degree     |       w        |    -y degree     |       s        |
   |    +z degree     |       e        |    -z degree     |       d        |
   |     +x trans     |       r        |     -x trans     |       f        |
   |     +y trans     |       t        |     -y trans     |       g        |
   |     +z trans     |       y        |     -z trans     |       h        |

   ```Intensity Color```:  此按钮可以将显示模式更改为强度图显示模式。这有助于检查地面车道线是否对齐。

   ``deg step`` ``t step `` :  这两个按钮更改每次单击或键盘输入的调整步骤。

   ``point size``: 调整雷达点云的大小

   ``ndt_auto_calibrate``:  用于ndt配准点云

   ``Reset``:  按下按钮重置所有手动调整

   ``Save Result``:  用于保存标定结果

# radar2lidar
## Step1
```bash
catkin_make
source devel/setup.bash
```
## Step2
发布lidar与radar的ros话题，lidar格式为sensor_msgs::PointCloud2格式，radar格式为visualization_msgs::MarkerArray，修改launch文件下对应lidar与radar话题名称。
```bash
roslaunch multi_sensor_calibration calibration.launch calib_radar2lidar:=true
```
## Step3
播放用于标定的rosbag包,进入标定界面：</br>
标定界面由用于手动校准的左侧控制面板和右侧点云界面组成。用户可以通过单击面板中的相应按钮或使用键盘作为输入来调整外部参数，来检查radar点云与lidar点云是否对齐。当两个点云对齐时，校准结束，单击保存按钮保存结果。

   | 外参调整 | 键盘输入 | 外参调整 | 键盘输入 |
   | :--------------: | :------------: | :--------------: | :------------: |
   |    +x degree     |       q        |    -x degree     |       a        |
   |    +y degree     |       w        |    -y degree     |       s        |
   |    +z degree     |       e        |    -z degree     |       d        |
   |     +x trans     |       r        |     -x trans     |       f        |
   |     +y trans     |       t        |     -y trans     |       g        |
   |     +z trans     |       y        |     -z trans     |       h        |

   ```Intensity Color```:  此按钮可以将显示模式更改为强度图显示模式。这有助于检查地面车道线是否对齐。

   ``deg step`` ``t step `` :  这两个按钮更改每次单击或键盘输入的调整步骤。

   ``radar line size``:  用于调整radar红线的大小

   ``icp_auto_calibrate``:  用于icp配准点云

   ``Reset``:  按下按钮重置所有手动调整

   ``Save Result``:  用于保存标定结果


# lidar2camera
## Step1
```bash
catkin_make
source devel/setup.bash
```
## Step2
发布camera与lidar的ros话题，camera格式为sensor_msgs::Image，lidar格式为sensor_msgs::PointCloud2，修改launch文件下对应camera与lidar话题。
```bash
roslaunch multi_sensor_calibration calibration.launch calib_lidar2camera:=true
```
## Step3
播放用于标定的rosbag包,进入标定界面：</br>
标定界面由用于手动校准的左侧控制面板和右侧图像界面组成。用户可以通过单击面板中的相应按钮或使用键盘作为输入来调整外部参数，来检查点云与图像是否对齐。当点云投影到图像上对齐时，校准结束，单击保存按钮保存结果。

   | 外参调整 | 键盘输入 | 外参调整 | 键盘输入 |
   | :--------------: | :------------: | :--------------: | :------------: |
   |    +x degree     |       q        |    -x degree     |       a        |
   |    +y degree     |       w        |    -y degree     |       s        |
   |    +z degree     |       e        |    -z degree     |       d        |
   |     +x trans     |       r        |     -x trans     |       f        |
   |     +y trans     |       t        |     -y trans     |       g        |
   |     +z trans     |       y        |     -z trans     |       h        |

   | 内参调整 | 键盘输入 | 内参调整 | 键盘输入 |
   | :--------------: | :------------: | :--------------: | :------------: |
   |       + fy       |       i        |       - fy       |       k        |
   |       + fx       |       u        |       - fx       |       j        |

   ```Intensity Color```:  此按钮可以将显示模式更改为强度图显示模式。这有助于检查地面车道线是否对齐。

   ```Overlap Filter```:  消除重叠的激光雷达点 

   ``deg step`` ``t step `` ``fxfy scale`` :  这三个按钮更改每次单击或键盘输入的调整步骤。

   ``point size``: 调整激光雷达点云投影到图像上的大小

   ``auto_calibrate``:  用于点云图像自动标定

   ``Reset``:  按下按钮重置所有手动调整

   ``Save Result``:  用于保存标定结果

# radar2camera
## Step1
```bash
catkin_make
source devel/setup.bash
```
## Step2
发布camera与radar的ros话题，camera格式为sensor_msgs::Image，radar格式为visualization_msgs::MarkerArray，修改launch文件下对应camera与radar话题。
```bash
roslaunch multi_sensor_calibration calibration.launch calib_radar2camera:=true
```
## Step3
播放用于标定的rosbag包,在弹出的图像中，按钮点选4个点，（两两一组，两条车道线），形成一组平行线</br>
通过单应性矩阵将图像的车道线转向鸟瞰图。校准结束时，确保道路沿线的雷达点与车道线平行。
## Step4
标定界面由用于手动校准的左侧控制面板和中间侧图像界面以及右侧鸟瞰图组成。用户可以通过单击面板中的相应按钮或使用键盘作为输入来调整外部参数，来检查点云与图像是否对齐。当点云投影到图像上对齐时，校准结束，单击保存按钮保存结果。

   | 外参调整 | 键盘输入 | 外参调整 | 键盘输入 |
   | :--------------: | :------------: | :--------------: | :------------: |
   |    +x degree     |       q        |    -x degree     |       a        |
   |    +y degree     |       w        |    -y degree     |       s        |
   |    +z degree     |       e        |    -z degree     |       d        |
   |     +x trans     |       r        |     -x trans     |       f        |
   |     +y trans     |       t        |     -y trans     |       g        |
   |     +z trans     |       y        |     -z trans     |       h        |

   ``deg step`` ``t step `` :  这两个按钮更改每次单击或键盘输入的调整步骤。

   ``point size``: 调整毫米波雷达点云投影到图像上的大小

   ``Reset``:  按下按钮重置所有手动调整

   ``Save Result``:  用于保存标定结果

# lidar2imu
## Step1
```bash
catkin_make
source devel/setup.bash
```
## Step2
发布lidar与imu的ros话题，格式如下:</br>
imu话题：CGI610GNSSMsg自定义消息</br>
lidar点云话题：sensor_msgs::PointCloud2
|参数名称|功能描述|格式|
|---|---|---|
|x|x坐标|float32|
|y|y坐标|float32|
|z|z坐标|float32|
|intensity|点云反射强度|float32|
|ring|点云所属线束信息|uint16|
|timestamp|点云时间|double|

```bash
roslaunch multi_sensor_calibration calibration.launch calib_lidar2imu:=true
```
## Step3
播放用于标定的rosbag包,等待一定时间采集足够多组数据后方可进行自动标定</br>
若要进行自动标定，在录制rosbag包的时候需尽可能使车辆在结构化道路上（周边有建筑物）以10km/h的速度绕8字形式，并且周围尽量没有移动障碍物。
## Step4
标定界面由用于手动校准的左侧控制面板和中间侧图像界面以及右侧鸟瞰图组成。用户可以通过单击面板中的相应按钮或使用键盘作为输入来调整外部参数，来检查点云是否能建图。当点云建图正确时，校准结束，单击保存按钮保存结果。

   | 外参调整 | 键盘输入 | 外参调整 | 键盘输入 |
   | :--------------: | :------------: | :--------------: | :------------: |
   |    +x degree     |       q        |    -x degree     |       a        |
   |    +y degree     |       w        |    -y degree     |       s        |
   |    +z degree     |       e        |    -z degree     |       d        |
   |     +x trans     |       r        |     -x trans     |       f        |
   |     +y trans     |       t        |     -y trans     |       g        |
   |     +z trans     |       y        |     -z trans     |       h        |

   ``deg step`` ``t step `` :  这两个按钮更改每次单击或键盘输入的调整步骤。

   ``point size``: 调整激光雷达点云的大小

   ``Auto Calibration``: 进行自动标定（需等待一段时间后即可）

   ``Show All Point``: 显示点云建图结果

   ``Reset``:  按下按钮重置所有手动调整

   ``Save Result``:  用于保存标定结果