# pointcloud_to_laserscan

`pointcloud_to_laserscan` 是一个点云转激光扫描包，用于将 `sensor_msgs/PointCloud2` 转换为 `sensor_msgs/LaserScan`，方便接入 2D 建图、定位与导航算法。

基于当前仓库代码，这个项目除了点云转激光扫描功能外，还额外包含：

- 基于 **IMU 重力方向对齐** 的点云处理选项
- 一个 `body_to_move` 辅助节点，用于发布 `body -> move` 的 TF
- 一个现成的启动文件 `launch/point_to_scan.launch`

## 功能概览

- 订阅点云话题 `cloud_in`
- 输出激光扫描话题 `scan`
- 支持将点云变换到指定 `target_frame`
- 支持高度过滤、角度范围过滤、距离过滤
- 支持使用 IMU 做重力对齐

## 适用场景

该包适合以下场景：

- 3D LiDAR 点云转 2D 激光，用于 2D SLAM
- 将 Livox / 其他点云雷达数据接入依赖 `LaserScan` 的旧系统
- 结合 IMU 与 TF，在平面场景中生成更稳定的 2D 扫描数据

## 项目结构

```text
pointcloud_to_laserscan/
├── CMakeLists.txt
├── package.xml
├── nodelets.xml
├── launch/
│   └── point_to_scan.launch
├── include/
│   └── pointcloud_to_laserscan/
│       └── pointcloud_to_laserscan_nodelet.h
└── src/
    ├── pointcloud_to_laserscan_node.cpp
    ├── pointcloud_to_laserscan_nodelet.cpp
    ├── body_to_move.cpp
    └── laserscan_to_pointcloud_nodelet.txt
```

## 编译方式



### 使用 catkin_make

```bash
cd ~/catkin_ws/src
git clone https://github.com/zhenyu-zy/pointcloud_to_laserscan.git
cd ..
catkin_make
source devel/setup.bash
```

### 使用 catkin build

```bash
cd ~/catkin_ws
catkin build pointcloud_to_laserscan
source devel/setup.bash
```

## 运行方式

### 方式 1：直接使用 launch 文件

```bash
roslaunch pointcloud_to_laserscan point_to_scan.launch
```

该启动文件会同时启动：

- `pointcloud_to_laserscan_node`
- `body_to_move`
- `map -> camera_init` 的静态 TF

### 方式 2：直接运行节点

```bash
rosrun pointcloud_to_laserscan pointcloud_to_laserscan_node
```

然后根据你的数据流自行 remap 话题和设置参数。

## 默认话题与 TF

### pointcloud_to_laserscan_nodelet / pointcloud_to_laserscan_node

- 输入点云：`cloud_in`
- 输出激光：`scan`
- 可选 IMU：`/livox/imu`
- 可选目标坐标系：`target_frame`

### launch 文件中的 remap

`launch/point_to_scan.launch` 中默认配置为：

- `cloud_in` -> `/cloud_registered_body`
- `scan` -> `/scan`
- `target_frame` -> `move`

### body_to_move 默认 TF 参数

- `parent_frame`: `camera_init`
- `body_frame`: `body`
- `move_frame`: `move`

## pointcloud_to_laserscan 主要参数

以下参数由 `pointcloud_to_laserscan_nodelet` 读取：

| 参数名 | 默认值 | 说明 |
|---|---:|---|
| `target_frame` | `""` | 将点云转换到该坐标系后再投影 |
| `imu_topic` | `/livox/imu` | IMU 输入话题 |
| `use_imu_gravity_alignment` | `false` | 是否启用 IMU 重力对齐 |
| `transform_tolerance` | `0.01` | TF 变换容差 |
| `min_height` | 极小值 | 最低高度过滤 |
| `max_height` | 极大值 | 最高高度过滤 |
| `angle_min` | `-π` | 扫描起始角 |
| `angle_max` | `π` | 扫描结束角 |
| `angle_increment` | `π/180` | 角分辨率 |
| `scan_time` | `1/30` | 扫描周期 |
| `range_min` | `0.0` | 最小量程 |
| `range_max` | 极大值 | 最大量程 |
| `use_inf` | `true` | 无回波时是否填充 `inf` |
| `inf_epsilon` | `1.0` | `use_inf=false` 时的超量程补偿 |
| `concurrency_level` | `1` | 并发线程配置 |