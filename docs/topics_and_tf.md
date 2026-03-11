# Topics 与 TF 现状梳理

## 1. 关键 Topics（按链路）

### 1.1 车辆状态链路

| 输入 | 处理节点 | 输出 | 状态 |
|---|---|---|---|
| `udp_receive_data` | `vehicle_dynamic_node` | `vehicle_dynamic_data` (`VehicleDynamic`) | 已完成 |
| `/vehicle_dynamic_data` | `vehicle_odometry_node` | `/odometry/vehicle` + `odom_vehicle->base_link_vehicle` | 已完成 |
| `/imu_enu` + `/odometry/vehicle` | `ekf_node` | `/odometry/filtered` | 已完成 |

代码依据：
- UDP -> vehicle_dynamic 映射。(src/vehicle_dynamic_pkg/src/vehicle_dynamic_node.cpp:12-15,20-37)
- 车辆里程计与 TF 发布。(src/vehicle_odometry_package/src/vehicle_odometry_node.cpp:15-21,60-64,96-124)
- EKF 订阅配置。(src/robot_localization/params/ekf.yaml:16-47)

### 1.2 感知与路径链路

| 输入 | 处理节点 | 输出 | 状态 |
|---|---|---|---|
| `/camera/color/image_raw` | `lanedet_node` | `/detection/lane/lane_markings_projected` + 可视化图像 | 半完成 |
| `/detection/lane/lane_markings_projected` + TF | `lane_boundary_transformer_node` | `/reference_path` | 已完成 |
| `/camera/color/image_raw` | `yolodetection_node` | `/detection/object/yolo2ddetection` 等 | 已完成（独立链路） |

代码依据：
- Lane 输入输出与发布器定义。(src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py:127-133,146-152,312-369)
- Lane->Path 变换发布。(src/lane_boundary_transformer/src/lane_boundary_transformer.cpp:31-36,42-69,232-241)
- YOLO 输入输出定义。(src/yolo_trt_ros2/yolo_trt_ros2/yolo_trt.py:38-52)

### 1.3 控制链路

| 输入 | 处理节点 | 输出 | 状态 |
|---|---|---|---|
| `/reference_path` + `/odometry/filtered` | `speedgoat_controller` | `/steering_angle`、`/acceleration`、UDP 下发 | 已完成 |

代码依据：
- 订阅和发布。(src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py:553-560)
- 控制计算与下发 UDP。(src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py:586-610)

---

## 2. TF 树现状

## 2.1 已动态发布的 TF

- `odom_gps -> base_link_gps`（gps_odometry_package）。(src/gps_odometry_package/src/gps_odometry_node.cpp:183-185)
- `odom_vehicle -> base_link_vehicle`（vehicle_odometry_package）。(src/vehicle_odometry_package/src/vehicle_odometry_node.cpp:111-113)

## 2.2 消费侧需要的 TF

- lane_boundary_transformer 查询 `odom <- base_link`。(src/lane_boundary_transformer/src/lane_boundary_transformer.cpp:46)

## 2.3 当前缺口（半完成）

- 由于发布侧使用 `odom_vehicle/base_link_vehicle` 与 `odom_gps/base_link_gps`，而消费侧直接查 `odom/base_link`，需额外静态桥接。readme 明确靠手工执行 static transform 命令补齐。  
  (src/trajectry_tracking_readme.md:13-18,28-35,51-55)

- `vehicle_description/display.launch.py` 中 static TF 节点未加入最终启动列表（被注释）。  
  (src/vehicle_description/launch/display.launch.py:25-35,53-54)

---

## 3. 发现的 topic/消息细节问题

1. `lanedet_node` 里 `combined_lanes.header.frame_id =='base_link'` 与 left/right header 同样使用 `==`，未真正赋值。  
   (src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py:318-322,341-343)
2. `speedgoat` 的 message_filters 同步方案已写但注释掉，当前使用“路径缓存 + 里程计回调”模式。  
   (src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py:543-552,568-589)

