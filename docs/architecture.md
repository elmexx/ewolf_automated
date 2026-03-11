# E-Wolf ROS2 架构文档（基于现有代码）

> 范围：仅描述当前仓库中**已经实现**的架构与数据流，不改动核心逻辑。

## 1. 总体分层

当前系统可分成 5 层：

1. **传感器与外设输入层**：GPS、Realsense、LiDAR、Speedgoat UDP。  
   - GPS 来源为 `/fix`（`gps_odometry_node` 订阅）。(src/gps_odometry_package/src/gps_odometry_node.cpp:27)
   - 相机来源为 `/camera/color/image_raw`（YOLO 与车道检测共用）。(src/yolo_trt_ros2/yolo_trt_ros2/yolo_trt.py:38-45, src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py:127-134)
   - 车辆动态原始数据来自 UDP 端口 5501。(src/udp_receiver/src/udp_receiver_node.cpp:10-13,28-40)

2. **感知与预处理层**：
   - IMU 坐标系转换 `/camera/imu -> /imu_enu`。(src/imu_transform_package/imu_transform_package/imu_transform.py:9-15,20-30)
   - YOLO 2D 检测发布检测框与可视化图像。(src/yolo_trt_ros2/yolo_trt_ros2/yolo_trt.py:48-52,78-133)
   - 车道线网络推理与投影后点发布。(src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py:146-160,310-369)

3. **状态估计层**：
   - GPS+IMU 转本地里程计 `/odometry/gps`，并发布 `odom_gps -> base_link_gps` TF。(src/gps_odometry_package/src/gps_odometry_node.cpp:33-35,110-135,179-195)
   - 车辆动态转里程计 `/odometry/vehicle`，并发布 `odom_vehicle -> base_link_vehicle` TF。(src/vehicle_odometry_package/src/vehicle_odometry_node.cpp:19-22,60-64,106-124)
   - EKF 使用 `/imu_enu + /odometry/vehicle` 输出 `/odometry/filtered`（由 `ekf_node` 运行）。(src/robot_localization/launch/ekf.launch.py:28-34, src/robot_localization/params/ekf.yaml:16-47)

4. **规划/参考轨迹层**：
   - 将车道投影点变换到 `odom`，融合历史并生成 `/reference_path`。(src/lane_boundary_transformer/src/lane_boundary_transformer.cpp:31-39,42-69,145-241)

5. **控制与执行层**：
   - `speedgoat_controller` 订阅 `/reference_path` 与 `/odometry/filtered`，计算转角/加速度并 UDP 回发控制量到 Speedgoat（10.42.0.10:5500）。(src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py:553-555,586-610)

---

## 2. 模块完成度评估

> 判定标准：
> - **已完成**：主流程可运行，输入/输出与核心算法闭环明确。
> - **半完成**：可运行但存在明显占位、硬编码或集成缺口。
> - **未完成**：当前主流程中不可用/未接入，或关键实现缺失。

### 2.1 已完成模块

- **UDP 接收链路（udp_receiver -> vehicle_dynamic_pkg）**  
  已实现 UDP 解包、10 通道映射与 ROS 消息发布。(src/udp_receiver/src/udp_receiver_node.cpp:51-68, src/vehicle_dynamic_pkg/src/vehicle_dynamic_node.cpp:20-38)

- **车辆动态里程计**  
  使用轮速推导车速与 yaw rate，并积分位置姿态，发布 odom 与 TF。(src/vehicle_odometry_package/src/vehicle_odometry_node.cpp:46-80,96-124)

- **IMU ENU 变换**  
  线加速度/角速度/姿态四元数均有转换并保留协方差。(src/imu_transform_package/imu_transform_package/imu_transform.py:20-51)

- **车道到参考路径转换**  
  已包含 TF 变换、历史融合、平滑与低通滤波，最终发布 `nav_msgs/Path`。(src/lane_boundary_transformer/src/lane_boundary_transformer.cpp:71-114,116-143,194-241)

- **控制节点主闭环**  
  已实现 Pure Pursuit / Stanley / PID / MPC 四种控制器并统一输出 ROS 与 UDP 控制命令。(src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py:508-538,595-619)

### 2.2 半完成模块

- **GPS+IMU 融合里程计（gps_odometry_package）**  
  节点可运行并发布 odom+TF，但参数 `gps_weight/imu_weight` 当前未真正参与融合（融合公式被注释），`theta_` 也未更新回成员状态，表现为“IMU 积分朝向+GPS位置”的简化实现。(src/gps_odometry_package/src/gps_odometry_node.cpp:24-25,101-104,208-211)

- **车道检测节点（lanedet_ros2）**  
  核心推理链路可运行，但模型路径硬编码到 `~/ros2_ws/...`，可移植性有限；此外 `header.frame_id =='base_link'` 使用了比较运算而非赋值，frame_id 实际未设置。(src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py:154-160,318-322,341-343)

- **TF 主干一致性**  
  lane transformer 固定查询 `odom <- base_link`，但车辆/GPS里程计发布的是 `odom_vehicle/base_link_vehicle` 与 `odom_gps/base_link_gps`，依赖额外静态 TF 手工桥接；readme 里也以手动命令方式维护，说明集成仍偏人工。(src/lane_boundary_transformer/src/lane_boundary_transformer.cpp:46, src/vehicle_odometry_package/src/vehicle_odometry_node.cpp:62-63, src/gps_odometry_package/src/gps_odometry_node.cpp:112-113, src/trajectry_tracking_readme.md:13-18)

### 2.3 未完成模块

- **`speedgoat_ros2_node.py` 中的 `ApproximateTimeSynchronizer` 联合同步方案**  
  已有代码骨架但整段被注释，当前未启用“路径+里程计时间同步回调”。(src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py:543-552,575-584)

- **vehicle_description 中 map/odom/base_link 静态 TF 发布**  
  launch 文件中创建了 static_transform_publisher 节点对象，但最终 LaunchDescription 里被注释，不会实际启动。(src/vehicle_description/launch/display.launch.py:25-35,53-54)

---

## 3. 架构关键风险（当前代码可见）

1. **坐标系命名不统一**：`odom` vs `odom_vehicle`/`odom_gps`、`base_link` vs `base_link_vehicle`/`base_link_gps`。这会直接影响 lane transformer 的 TF 查询成功率。  
2. **硬编码部署路径与网络参数较多**：如模型路径、UDP IP/端口、LiDAR HostIP，迁移环境需逐项改参。  
3. **可视化依赖 GUI**：YOLO 与 lane 节点都调用 `cv2.imshow`，在无显示环境部署时需处理。  
(证据：src/lane_boundary_transformer/src/lane_boundary_transformer.cpp:46, src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py:561-563, src/yolo_trt_ros2/yolo_trt_ros2/yolo_trt.py:123-124, src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py:305-307)

