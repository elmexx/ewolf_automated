# 控制流水线说明（Control Pipeline）

## 1. 运行目标

控制节点 `speedgoat_controller` 以 `/odometry/filtered` 作为当前状态、以 `/reference_path` 作为参考轨迹，输出转角和加速度，并通过 UDP 下发给 Speedgoat。  
(代码：src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py:553-555,586-610)

## 2. 上游依赖

1. **参考路径来源**：lane transformer。  
   - 输入 `/detection/lane/lane_markings_projected`。  
   - 输出 `/reference_path`。  
   (src/lane_boundary_transformer/src/lane_boundary_transformer.cpp:31-36)

2. **状态估计来源**：robot_localization ekf。  
   - 关键输入 `/imu_enu`、`/odometry/vehicle`。  
   - 关键坐标参数：`odom_frame=odom`、`base_link_frame=base_link`。  
   (src/robot_localization/params/ekf.yaml:11-13,16-47)

3. **车辆里程计来源**：`vehicle_dynamic_data -> /odometry/vehicle`。  
   (src/vehicle_odometry_package/src/vehicle_odometry_node.cpp:15-21)

## 3. 控制器实现

`speedgoat_ros2_node.py` 内置四类控制器：
- Pure Pursuit
- Stanley
- PID（横向 PID + heading）
- MPC（SLSQP 求解）

并通过 `control_method` 参数选择。  
(代码：src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py:33-123,125-221,223-340,343-489,496-538)

## 4. 控制回调行为

在 `odom_callback`：
1. 若参考路径为空，直接返回并告警；
2. 读取当前位姿与速度；
3. 调用 `compute_steering_angle` 与 `compute_acceleration`；
4. 发布 ROS 控制话题 `/steering_angle`、`/acceleration`；
5. 打包 `np.float64` 数组并 UDP 发送到 `10.42.0.10:5500`。

(代码：src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py:587-619,561-563)

## 5. 完成度评估（控制域）

### 5.1 已完成

- 多控制器主逻辑与参数化选择已完成。  
- ROS 输出和 UDP 输出双通道已完成。  
(代码：src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py:508-538,558-563,611-619)

### 5.2 半完成

- 控制输入依赖 `/odometry/filtered` 与 `/reference_path`；其中 `/reference_path` 强依赖 TF `odom<-base_link` 可用性，当前仓库存在 frame 体系不统一（见 TF 文档）。  
(代码：src/lane_boundary_transformer/src/lane_boundary_transformer.cpp:46, src/robot_localization/params/ekf.yaml:11-13)

### 5.3 未完成

- 同步式控制回调（`ApproximateTimeSynchronizer`）已预留但被注释，尚未投入实际链路。  
(代码：src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py:543-552,575-584)

