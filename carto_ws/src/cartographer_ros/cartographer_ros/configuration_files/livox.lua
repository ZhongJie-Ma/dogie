-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- 引入基础模块，包含 map_builder 和 trajectory_builder 的默认配置
include "map_builder.lua"
include "trajectory_builder.lua"

-- 主配置表
options = {
  -- 地图构建器配置（来自 map_builder.lua）
  map_builder = MAP_BUILDER,
  -- 轨迹构建器配置（来自 trajectory_builder.lua）
  trajectory_builder = TRAJECTORY_BUILDER,
  -- 全局地图的坐标系名称
  map_frame = "map",
  -- 机器人本体的坐标系名称（通常为 base_link 或 body）
  tracking_frame = "body_2d",
  -- 里程计坐标系名称，用于接收 /odom 消息
  odom_frame = "camera_init",
  -- 是否由 Cartographer 内部发布里程计数据（若设为 false，则使用外部里程计）
  provide_odom_frame = false,
  -- 是否将发布的位姿投影到 2D 平面（用于 2D 建图时忽略高度和横滚俯仰）
  publish_frame_projected_to_2d = false,
  -- 是否使用位姿外推器（在传感器数据短暂缺失时预测当前位姿）
  use_pose_extrapolator = true,
  -- 是否使用里程计数据（需要提供 odom 话题）
  use_odometry = true,
  -- 是否使用 GPS/卫星导航数据
  use_nav_sat = false,
  -- 是否使用路标数据（如视觉二维码等）
  use_landmarks = false,
  -- 单回波激光雷达的数量（通常为 1）
  num_laser_scans = 1,
  -- 多回波激光雷达的数量（若雷达支持多回波且需要处理，则设置 >0）
  num_multi_echo_laser_scans = 0,
  -- 每帧激光扫描被细分的次数（用于提高匹配精度，1 表示不细分）
  num_subdivisions_per_laser_scan = 1,
  -- 点云（如深度相机）的数量
  num_point_clouds = 0,
  -- 查找 TF 变换的超时时间（秒）
  lookup_transform_timeout_sec = 0.2,
  -- 子图发布周期（秒），控制 rviz 中显示子图的频率
  submap_publish_period_sec = 0.3,
  -- 机器人位姿发布周期（秒），控制实时位姿的更新频率
  pose_publish_period_sec = 5e-3,   -- 5ms，即 200Hz
  -- 轨迹发布周期（秒）
  trajectory_publish_period_sec = 30e-3, -- 30ms
  -- 测距数据（激光/点云）的采样比例，1.0 表示全部使用
  rangefinder_sampling_ratio = 1.,
  -- 里程计数据的采样比例
  odometry_sampling_ratio = 1.,
  -- 固定坐标系（如 GPS）位姿的采样比例
  fixed_frame_pose_sampling_ratio = 1.,
  -- IMU 数据的采样比例
  imu_sampling_ratio = 1.,
  -- 路标数据的采样比例
  landmarks_sampling_ratio = 1.,
  published_frame = "",
}

-- 强制使用 2D 轨迹构建器（true 为 2D，false 为 3D）
MAP_BUILDER.use_trajectory_builder_2d = true

-- 以下是 TRAJECTORY_BUILDER_2D 的具体参数调整

-- 每个子图包含的激光扫描帧数（达到该值后子图将被优化并插入）
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 20

-- 激光雷达的最小有效距离（米），小于此值的点被忽略
TRAJECTORY_BUILDER_2D.min_range = 0.3
-- 激光雷达的最大有效距离（米），大于此值的点被忽略
TRAJECTORY_BUILDER_2D.max_range = 8.

-- 当激光束没有打到任何物体时，赋予的默认长度（用于处理缺失数据）
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.

-- 是否使用 IMU 数据（此处设为 false，若机器人有 IMU 可改为 true）
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 是否启用实时相关扫描匹配（提高定位精度，计算量较大）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- 实时相关扫描匹配器的搜索窗口参数
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.05  -- 线性搜索窗口（米）
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.  -- 平移代价权重
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1   -- 旋转代价权重

-- 位姿图优化相关参数

-- 优化问题中 Huber 核函数的尺度参数（用于降低异常值的影响）
POSE_GRAPH.optimization_problem.huber_scale = 1e2

-- 每累积 N 个节点（关键帧）执行一次全局优化
POSE_GRAPH.optimize_every_n_nodes = 100

-- 约束构建器的最小匹配分数（低于此分数的约束将被拒绝）
POSE_GRAPH.constraint_builder.min_score = 0.7

-- 返回配置表供 Cartographer 加载
return options
