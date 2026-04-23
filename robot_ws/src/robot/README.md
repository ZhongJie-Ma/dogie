# robot

一个移动机器人功能包。

这个仓库的核心职责是：通过串口与下位机底盘通信，发布里程计、IMU、电源与回充相关状态，并接收速度控制、安全控制和机械臂关节控制指令。同时，仓库中还包含了 TF、地图、相机以及 `move_base` 导航相关的启动文件和参数配置。

## 功能概览

- 底盘串口驱动
  - 与下位机通过串口通信
  - 接收底盘速度、IMU、电压、回充状态等数据
  - 下发速度控制、自动回充和机械臂控制指令
- 状态发布
  - 发布里程计 `/odom`
  - 发布 IMU `/imu`
  - 发布电池电压 `/power`
  - 发布回充状态、充电电流、红外充电桩检测标志等
- TF 与基础启动
  - 提供机器人底盘相关静态 TF
  - 提供底盘启动入口 `start.launch`
- 导航配置
  - 提供 `move_base`、全局规划器、代价地图配置
  - 同时保留 TEB 与 DWA 两套局部规划器配置
- 传感器与地图
  - 提供 USB 相机启动文件
  - 提供地图加载与保存相关 launch 文件

## 目录结构

```text
robot/
├─ CMakeLists.txt
├─ package.xml
├─ include/robot/
│  ├─ robot.h
│  └─ Quaternion_Solution.h
├─ src/robot/
│  ├─ robot.cpp
│  ├─ robot_backup.cpp
│  ├─ twist_transform.cpp
│  └─ Quaternion_Solution.cpp
├─ launch/
│  ├─ start.launch
│  ├─ tf_robot.launch
│  ├─ usb_cam.launch
│  ├─ map.launch
│  ├─ map_saver.launch
│  └─ include/
│     ├─ base_serial.launch
│     ├─ move_base.launch
│     └─ dwa_local_planner.launch
├─ map/
│  ├─ 1.yaml
│  └─ 1.pgm
├─ msg/
│  └─ supersonic.msg
├─ params_costmap_car/
│  └─ costmap_car_params.yaml
├─ params_costmap_common/
│  ├─ costmap_common_params.yaml
│  ├─ global_costmap_params.yaml
│  └─ local_costmap_params.yaml
└─ params_nav_common/
   ├─ base_global_planner_param.yaml
   ├─ dwa_local_planner_params.yaml
   ├─ move_base_params.yaml
   └─ teb_local_planner_params.yaml
```


## 编译方式

将仓库放入 catkin 工作空间的 `src` 目录后编译：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 生成的节点

编译后会生成两个可执行节点：

- `robot_node`
- `twist_transform`

## 主要节点说明

### 1. robot_node

`robot_node` 是本仓库的主节点，负责底盘串口通信与状态发布。

#### 默认参数

- `~usart_port_name`：串口设备名，默认 `/dev/controller`
- `~serial_baud_rate`：串口波特率，默认 `115200`
- `~odom_frame_id`：里程计坐标系，默认 `odom_combined`
- `~robot_frame_id`：机器人底盘坐标系，默认 `base_footprint`
- `~gyro_frame_id`：IMU 坐标系，默认 `gyro_link`
- `~arm_cmd`：机械臂控制话题名，默认 `arm_cmd`
- `~odom_x_scale`：X 方向里程计修正系数，默认 `1.0`
- `~odom_y_scale`：Y 方向里程计修正系数，默认 `1.0`
- `~odom_z_scale_positive`：正向角速度修正系数，默认 `1.0`
- `~odom_z_scale_negative`：负向角速度修正系数，默认 `1.0`

#### 发布话题

- `/odom` (`nav_msgs/Odometry`)
- `/imu` (`sensor_msgs/Imu`)
- `/power` (`std_msgs/Float32`)
- `/robot_charging_flag` (`std_msgs/Bool`)
- `/robot_charging_current` (`std_msgs/Float32`)
- `/robot_red_flag` (`std_msgs/UInt8`)
- `/robot_selfcheck` (`std_msgs/UInt32`)

#### 订阅话题

- `/cmd_vel` (`geometry_msgs/Twist`)
- `/red_vel` (`geometry_msgs/Twist`)
- `/robot_recharge_flag` (`std_msgs/Int8`)
- `/chassis_security` (`std_msgs/Int8`)
- `/arm_cmd`（默认名，可通过 `~arm_cmd` 修改，消息类型为 `std_msgs/Float32MultiArray`）

#### 服务

- `/set_charge`
  - 当前复用了 `turtlesim/Spawn` 服务类型
  - 代码中通过 `request.x` 传递自动回充模式：
    - `0`：取消自动回充
    - `1` / `2`：设置不同的自动回充状态

### 2. twist_transform

这个节点用于将 `geometry_msgs/TwistStamped` 转换为 `geometry_msgs/Twist`。

#### 订阅话题

- `/twist_raw` (`geometry_msgs/TwistStamped`)

#### 发布话题

- `/cmd_vel` (`geometry_msgs/Twist`)

适用于上游控制器输出 `TwistStamped`，而底盘只接收 `Twist` 的场景。

## 启动文件说明

### `launch/start.launch`

基础启动入口，会启动：

- `launch/include/base_serial.launch`
- `launch/tf_robot.launch`

适合作为底盘驱动与基础 TF 的统一入口。

### `launch/include/base_serial.launch`

启动 `robot_node`，并设置串口、波特率、TF 名称以及里程计修正参数。

默认参数：

- 串口：`/dev/ttyACM0`
- 波特率：`115200`
- `odom_frame_id`：`odom_combined`
- `robot_frame_id`：`base_footprint`
- `gyro_frame_id`：`gyro_link`

### `launch/tf_robot.launch`

发布两个静态 TF：

- `move -> base_footprint`
- `base_footprint -> base_link`

### `launch/usb_cam.launch`

启动 USB 摄像头节点，默认：

- 设备：`/dev/video0`
- 分辨率：`640x480`
- 像素格式：`yuyv`

### `launch/map.launch`

用于加载地图文件，默认地图为 `map/1.yaml`。

### `launch/map_saver.launch`

用于保存地图。

### `launch/include/move_base.launch`

提供基于 `move_base` 的导航启动配置，默认使用：

- 全局规划器：`global_planner/GlobalPlanner`
- 局部规划器：`teb_local_planner/TebLocalPlannerROS`

支持通过参数传入：

- `odom_topic`，默认 `odom`
- `laser_topic`，默认 `/scan`

### `launch/include/dwa_local_planner.launch`

提供 DWA 局部规划器配置，适合需要切换局部规划算法时使用。

## 参数文件说明

### 导航相关参数

位于 `params_nav_common/`：

- `move_base_params.yaml`：`move_base` 整体行为参数
- `base_global_planner_param.yaml`：全局规划器参数
- `teb_local_planner_params.yaml`：TEB 局部规划器参数
- `dwa_local_planner_params.yaml`：DWA 局部规划器参数

### 代价地图参数

位于 `params_costmap_common/`：

- `costmap_common_params.yaml`：静态层、障碍层等公共参数
- `global_costmap_params.yaml`：全局代价地图参数
- `local_costmap_params.yaml`：局部代价地图参数

### 机器人外形参数

位于 `params_costmap_car/costmap_car_params.yaml`：

- 机器人 footprint
- inflation layer 参数

## 自定义消息

### `msg/supersonic.msg`

定义了 8 路超声波距离字段：

- `distanceA`
- `distanceB`
- `distanceC`
- `distanceD`
- `distanceE`
- `distanceF`
- `distanceG`
- `distanceH`

说明：仓库中已经定义了该消息，但从当前主节点实现来看，超声波自定义消息发布链路尚未完整接通到主流程中。

## 快速开始

### 1. 启动底盘与 TF

```bash
roslaunch robot start.launch
```

### 2. 按需启动相机

```bash
roslaunch robot usb_cam.launch
```

### 3. 加载地图

```bash
roslaunch robot map.launch
```

### 4. 启动导航

在确认导航相关路径和话题配置已与本机环境一致后，可尝试：

```bash
roslaunch robot include/move_base.launch odom_topic:=/odom laser_topic:=/scan
```

如果上游控制输出的是 `TwistStamped`，可以额外启动：

```bash
rosrun robot twist_transform
```

## 适用场景

这个仓库适合作为以下项目的基础：

- ROS1 小车底盘驱动包
- 带串口下位机的移动机器人集成
- 需要里程计、IMU、电池状态发布的机器人项目
- 基于 `move_base` 的基础导航实验或工程整合


