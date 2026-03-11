# ROADMAP（按当前代码基线）

> 说明：本路线图基于仓库现状，分为“已完成 / 半完成 / 未完成”，用于后续工程化推进。

## 一、已完成（可作为当前稳定基线）

- [x] **车辆动态数据通道**：UDP 收包 -> `VehicleDynamic` 消息映射。  
  证据：`udp_receiver_node.cpp`、`vehicle_dynamic_node.cpp`。(src/udp_receiver/src/udp_receiver_node.cpp:51-68, src/vehicle_dynamic_pkg/src/vehicle_dynamic_node.cpp:20-38)

- [x] **车辆里程计发布**：`/odometry/vehicle` 与 `odom_vehicle->base_link_vehicle`。  
  证据：`vehicle_odometry_node.cpp`。(src/vehicle_odometry_package/src/vehicle_odometry_node.cpp:60-64,96-124)

- [x] **IMU ENU 转换节点**：`/camera/imu -> /imu_enu`。  
  证据：`imu_transform.py`。(src/imu_transform_package/imu_transform_package/imu_transform.py:9-15,20-51)

- [x] **控制核心**：PurePursuit / Stanley / PID / MPC + UDP 控制下发。  
  证据：`speedgoat_ros2_node.py`。(src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py:508-538,586-610)

- [x] **车道路径转换**：投影点变换、平滑、低通、发布 `/reference_path`。  
  证据：`lane_boundary_transformer.cpp`。(src/lane_boundary_transformer/src/lane_boundary_transformer.cpp:42-69,145-241)

## 二、半完成（已具备主链路，但工程化不足）

- [~] **GPS+IMU 融合里程计**：节点可运行，但融合权重与部分融合逻辑未生效（注释/未回写状态）。  
  证据：`gps_odometry_node.cpp`。(src/gps_odometry_package/src/gps_odometry_node.cpp:24-25,101-104,208-211)

- [~] **Lane 检测发布规范化**：有发布链路，但 frame_id 赋值写成比较运算；模型路径硬编码。  
  证据：`lanedet_node_helper.py`。(src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py:154-160,318-322,341-343)

- [~] **TF 统一治理**：运行依赖手工 static TF 命令；自动化 launch 未完全接管。  
  证据：`trajectry_tracking_readme.md`、`display.launch.py`。(src/trajectry_tracking_readme.md:13-18, src/vehicle_description/launch/display.launch.py:53-54)

## 三、未完成（建议优先级）

- [ ] **控制输入时间同步回调启用**：`message_filters` 同步逻辑存在但未启用。  
  证据：`speedgoat_ros2_node.py` 注释块。(src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py:543-552,575-584)

- [ ] **启动编排一体化**：当前流程主要靠手工命令顺序执行（readme），建议沉淀统一 launch 编排。  
  证据：轨迹 readme 的步骤式手工启动。(src/trajectry_tracking_readme.md:28-83)

## 四、建议里程碑（不改核心算法前提）

### M1（短期，1~2 周）
- 完成 TF 命名统一策略与静态 TF 自动启动；
- 修正 lane 消息 frame_id 赋值错误；
- 参数化硬编码路径/IP（通过 launch/param 注入）。

### M2（中期，2~4 周）
- 启用并验证控制输入同步回调；
- 增加端到端 topic/TF 自检脚本（启动后自动验证关键 topic 与 transform）。

### M3（中长期，>4 周）
- 完成 GPS/IMU 真融合（参数真正生效，状态一致更新）；
- 建立可复现实车/仿真回放流程与性能指标看板（时延、稳定性、轨迹误差）。

