# Lane-Following 专项优化路线图

> 目标不是补齐整个系统工程化，而是优先提升 **lane following 主链路稳定性**：减少漏检、压制连续帧跳动、稳定参考路径、降低 steering angle 左右晃动，并处理 camera 延迟造成的控制滞后。当前代码里虽然已经存在车道平滑、reference path 平滑、控制器低通和时延观测节点，但这些“已有步骤”并不能直接证明实际稳定性已足够，因为它们仍缺少质量门槛、异常退化策略和闭环验收指标。.【F:src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py†L278-L285】【F:src/lane_boundary_transformer/src/lane_boundary_transformer.cpp†L62-L68】【F:src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py†L194-L201】【F:src/node_delay_package/node_delay_package/node_delay.py†L15-L38】

## 总体优先级

### 必须先做
1. **阶段 1：观测与诊断**。没有量化数据，就无法区分问题主要来自漏检、路径生成还是控制抖动。.【F:src/node_delay_package/node_delay_package/node_delay.py†L19-L38】
2. **阶段 2：Reference path 稳定化**。当前控制器直接消费 `/reference_path`，因此参考轨迹不稳会直接传递到 steering 抖动。.【F:src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py†L553-L555】【F:src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py†L586-L597】
3. **阶段 3：延迟补偿**。当前 lane 检测链路是图像驱动、控制是 odometry 驱动，如果不先估算并补偿相机到控制的总时延，后续 controller 调参容易“治标不治本”。.【F:src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py†L201-L210】【F:src/node_delay_package/node_delay_package/node_delay.py†L26-L38】

### 可以后做
4. **阶段 4：Controller 抗抖优化**。应建立在“输入路径已稳定 + 延迟已量化”的前提下，否则会把 controller 调成掩盖前级问题的补丁。.【F:src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py†L148-L209】【F:src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py†L256-L340】
5. **阶段 5：验证与回归**。需要在前四阶段形成明确改动项后，固化成回归机制。.【F:src/node_delay_package/node_delay_package/node_delay.py†L26-L38】【F:src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py†L370-L371】

---

## 阶段 1：观测与诊断

### 目标
- 把“lane 漏检、lane 跳动、reference path 不平滑、steering 晃动、camera 延迟”拆成可量化问题。
- 明确问题主要出现在 **lane detection**、**path generation** 还是 **controller/output**。

### 涉及模块 / 文件
- `src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py`：lane 检测、左右车道筛选、滑动均值、消息发布时间。.【F:src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py†L201-L218】【F:src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py†L237-L285】【F:src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py†L315-L371】
- `src/lane_boundary_transformer/src/lane_boundary_transformer.cpp`：reference path 生成、插值、平滑、低通。.【F:src/lane_boundary_transformer/src/lane_boundary_transformer.cpp†L54-L69】【F:src/lane_boundary_transformer/src/lane_boundary_transformer.cpp†L145-L230】
- `src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py`：steering 输出与控制日志。.【F:src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py†L586-L619】
- `src/node_delay_package/node_delay_package/node_delay.py`、`src/sys_time_package/sys_time_package/sys_time.py`：时延观测基础。.【F:src/node_delay_package/node_delay_package/node_delay.py†L11-L38】【F:src/sys_time_package/sys_time_package/sys_time.py†L8-L23】

### 建议修改内容
- 在 `lanedet_node_helper.py` 增加诊断统计：
  - 每帧检测到的 lane 数量、ego-left/ego-right 是否缺失；
  - 左右 lane 参数 `a/b/c` 的帧间差分；
  - 从原始图像时间戳到 lane 消息发布时间的延迟。  
  当前代码虽然有 `delaylane = time.time()-start_time`，但只打印总推理耗时，缺少分项统计与 ROS 时戳对齐。.【F:src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py†L203-L210】【F:src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py†L370-L371】
- 在 `lane_boundary_transformer.cpp` 增加 path 质量指标：
  - 相邻点曲率变化、heading 变化、帧间 path 起点/终点偏移；
  - 当左右边界数量不足或历史回退触发时，输出明确诊断标志。  
  当前仅在左边界为空时退化到 `last_valid_markings_`，没有暴露退化比例。.【F:src/lane_boundary_transformer/src/lane_boundary_transformer.cpp†L56-L60】
- 在 `speedgoat_ros2_node.py` 增加 steering 质量指标：
  - steering angle 一阶差分、二阶差分；
  - 每秒零交叉次数（左右晃动指标）；
  - 同一路段不同控制方法下的输出统计。.【F:src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py†L602-L619】
- 扩展 `node_delay.py`：不仅观测 system/camera/lane，还要加入 `/reference_path` 与 `/odometry/filtered` 的对齐统计，形成 lane-following 全链路延迟画像。当前节点只观测 `system_time + camera + leftlanedetection`，覆盖不够。.【F:src/node_delay_package/node_delay_package/node_delay.py†L15-L24】

### 风险点
- 日志太多会反过来增加实时链路负担，尤其 `lanedet_node_helper.py` 已经在每帧做 CUDA 推理与 `cv2.imshow`。.【F:src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py†L210-L218】【F:src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py†L305-L307】
- 如果诊断只打印文本、不落盘，就很难做长时间比较。

### 验收指标
- 能输出至少 5 类时间序列：漏检率、lane 参数帧间跳变量、path 曲率变化、steering 零交叉频率、camera→lane→path→control 总延迟。
- 对同一段道路回放，能清晰判断抖动是源于 lane 漏检、path 平滑不足，还是 controller 增益过高。

---

## 阶段 2：Reference Path 稳定化

### 目标
- 把当前“检测到的左右边界中心线”升级为**可退化、可持续、对缺失鲁棒**的 reference path。
- 重点解决：漏检后 path 断裂、连续帧 lane 跳动导致 path 摆动。

### 涉及模块 / 文件
- `src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py`。.【F:src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py†L237-L285】【F:src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py†L322-L360】
- `src/lanedet_ros2/lanedet_ros2/tools/lane_parameter.py`。.【F:src/lanedet_ros2/lanedet_ros2/tools/lane_parameter.py†L255-L282】【F:src/lanedet_ros2/lanedet_ros2/tools/lane_parameter.py†L531-L531】
- `src/lane_boundary_transformer/src/lane_boundary_transformer.cpp`。.【F:src/lane_boundary_transformer/src/lane_boundary_transformer.cpp†L116-L143】【F:src/lane_boundary_transformer/src/lane_boundary_transformer.cpp†L145-L230】

### 建议修改内容
- **先稳 lane，再稳 path**：
  - 现在 `moving_average` 只是对多项式参数做窗口均值，但没有剔除异常帧，也没有按置信度加权。建议在 `lanedet_node_helper.py` 中新增“validity gate”：只有当 lane 点数、横向位置、曲率满足约束时才进入窗口。否则保留上一帧有效结果。.【F:src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py†L248-L285】
  - `get_fit_param()` 当前直接 `fit()`，异常统一落入裸 `except` 返回旧值，建议改成显式质量检查：点数阈值、RANSAC 内点比例、左右车道宽度范围。当前逻辑对“错误但可拟合”的假阳性约束不够。.【F:src/lanedet_ros2/lanedet_ros2/tools/lane_parameter.py†L261-L282】
- **在 lane_boundary_transformer 中引入 path 级历史模型**：
  - 当前 `fuseWithHistory()` 只是对同 index 的左右点做线性融合，默认点数和对应关系稳定；一旦左右点采样分布变化，就可能把错位点硬融合。建议改为按弧长重采样后再融合。.【F:src/lane_boundary_transformer/src/lane_boundary_transformer.cpp†L122-L143】
  - `smoothPath()` 当前是简单滑窗均值，`lowpassFilter()` 是点坐标一阶低通，建议把“平滑对象”从 `x/y` 点列升级为 `path heading + curvature`，避免拐弯处被过度拉直。.【F:src/lane_boundary_transformer/src/lane_boundary_transformer.cpp†L194-L230】
- **建立丢线退化策略**：
  - 当前 lane detection 在单侧缺失时会人工平移另一侧 4 米生成补边界，这能保 continuity，但未验证在弯道和变道场景下是否稳定。建议限定该策略仅在短时间窗口内使用，并叠加 vehicle yaw / 上一帧曲率约束。.【F:src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py†L265-L270】
  - `lane_boundary_transformer` 当前仅判断 `markings_left.empty()`，没有同时评估右边界质量，建议升级成“左右各自评分 + centerline 评分”的退化逻辑。.【F:src/lane_boundary_transformer/src/lane_boundary_transformer.cpp†L56-L60】

### 风险点
- 过强平滑会造成 path 滞后，特别是在急弯和 T 字路口入口。
- 单侧补线如果不做时间限制，会把暂时性错误观测放大为持续性偏置。
- 需要避免 path 稳定化和后续延迟补偿相互抵消。

### 验收指标
- 连续 100 帧内，reference path 起点横向偏差标准差下降明显（建议目标：下降 30% 以上）。
- 单次漏检持续 1~3 帧时，不出现 path 断裂、不出现中心线瞬时跳变超过固定阈值（例如 >0.3 m）。
- 弯道场景中，path heading 一阶差分连续，无明显 saw-tooth 形态。

---

## 阶段 3：延迟补偿

### 目标
- 量化并补偿 camera -> lane detection -> path -> control 的总延迟，降低“看到车道时已经晚了”的现象。

### 涉及模块 / 文件
- `src/node_delay_package/node_delay_package/node_delay.py`。.【F:src/node_delay_package/node_delay_package/node_delay.py†L15-L38】
- `src/sys_time_package/sys_time_package/sys_time.py`。.【F:src/sys_time_package/sys_time_package/sys_time.py†L8-L23】
- `src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py`。.【F:src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py†L315-L371】
- `src/lane_boundary_transformer/src/lane_boundary_transformer.cpp`。.【F:src/lane_boundary_transformer/src/lane_boundary_transformer.cpp†L42-L69】
- `src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py`。.【F:src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py†L586-L610】
- `src/vehicle_odometry_package/src/vehicle_odometry_node.cpp`。.【F:src/vehicle_odometry_package/src/vehicle_odometry_node.cpp†L28-L40】【F:src/vehicle_odometry_package/src/vehicle_odometry_node.cpp†L67-L80】

### 建议修改内容
- **先把时间戳链路做对**：
  - lane detection 现在给 left/right/projected lane 统一使用 `self.get_clock().now()`，而不是沿用 camera 原始时间戳，这会掩盖相机到检测的真实延迟。应保留原图像时间戳，并额外附带 processing latency。.【F:src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py†L315-L321】
  - vehicle dynamic 消息也使用 `now()` 而不是 UDP 原始采样时刻，如果 Speedgoat 端能提供采样时间，建议一并透传；否则至少记录接收时刻与发布时刻。.【F:src/vehicle_dynamic_pkg/src/vehicle_dynamic_node.cpp†L22-L24】
- **做 path 前馈预测**：
  - 在 `lane_boundary_transformer` 或 `speedgoat` 侧，根据当前 `odometry/filtered` 的速度和 yaw rate，将 reference path 前移 `delay * v` 的距离，形成“时刻对齐后的控制目标”。
- **做控制输入对齐**：
  - `speedgoat` 当前在每次 odom callback 中直接消费最近缓存的 `reference_path`，但没比较 path 时间戳和 odom 时间戳差值。建议加入最大允许时差与预测补偿分支。.【F:src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py†L568-L589】
- **保留 node_delay 但升级为专项工具**：
  - 输出 P50/P95/P99 延迟，而不只是一帧一帧打印毫秒值。.【F:src/node_delay_package/node_delay_package/node_delay.py†L32-L38】

### 风险点
- 如果 time base 不统一（camera stamp、ROS time、controller time 混用），补偿公式会变成“错上加错”。
- 过度前移 path 会造成转向提前，低速时尤其明显。

### 验收指标
- 能稳定给出 camera->lane、lane->path、path->control、总延迟的 P95。
- 在相同场景下，补偿后 steering 峰值相位滞后明显减小；车辆过弯切线点更接近视觉中心线。
- 延迟补偿开启后，低速直道不产生额外左右摆动。

---

## 阶段 4：Controller 抗抖优化

### 目标
- 在“输入路径已更稳定、延迟已被补偿”的前提下，降低 steering angle 左右晃动，同时尽量不牺牲跟踪精度。

### 涉及模块 / 文件
- `src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py` 中的 Pure Pursuit、Stanley、PID、MPC 控制器。.【F:src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py†L33-L123】【F:src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py†L125-L221】【F:src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py†L223-L340】【F:src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py†L343-L489】

### 建议修改内容
- **先限定控制器使用范围，再调参**：
  - 当前仓库同时保留四类控制器，但 lane following 场景应先选 1~2 个主力方案（建议 Stanley + Pure Pursuit 或 Stanley + PID），避免并行维护过多调参空间。.【F:src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py†L496-L538】
- **增强输入预处理**：
  - 为 controller 使用的 reference path 增加固定 lookahead 截取与 heading 预览，不直接对整条 path 做最近点搜索，以减少局部毛刺对 steering 的放大。当前 Stanley/PID 都对 path 原始点列做 nearest-point 搜索。.【F:src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py†L135-L147】【F:src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py†L243-L255】
- **显式 steering rate limit**：
  - 当前 Stanley/PID 只有一阶低通和 deadband，没有明确的方向盘角速度约束。建议增加 `max_steer_rate_deg_per_s`，防止相邻周期输出翻转。.【F:src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py†L194-L201】【F:src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py†L320-L327】
- **速度相关增益调度**：
  - 低速允许更大 steering，车速上升后逐步降低 lateral gain，减少高速蛇形。当前 Stanley 的 `k=5.0`、PID 参数是固定值。.【F:src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py†L516-L529】
- **加入 path 曲率前馈**：
  - 若 reference path 已能提供局部 heading/curvature，可在反馈量外增加前馈项，降低 controller 仅靠误差修正带来的左右来回打方向。

### 风险点
- 如果前级 path 仍抖，controller rate limit 只会把抖动“拉平”，但不一定改善跟踪误差。
- 增益调度过强时，不同速度区间会出现控制风格突变。
- MPC 计算量较大，在 camera 延迟已经偏高时，可能进一步加重闭环滞后。.【F:src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py†L466-L488】

### 验收指标
- steering 一阶差分 RMS 下降明显（建议目标：下降 30% 以上）。
- 单位时间 steering 零交叉次数显著减少。
- 在直道、小曲率弯道、较急弯三类场景下，横向误差不高于现基线，同时方向盘动作更平顺。

---

## 阶段 5：验证与回归

### 目标
- 建立 lane-following 专项回归，而不是只验证“节点能启动”。

### 涉及模块 / 文件
- `docs/architecture.md`、`docs/control_pipeline.md`、`ROADMAP.md`（用于同步专项结论）。.【F:docs/architecture.md†L32-L83】【F:docs/control_pipeline.md†L34-L62】【F:ROADMAP.md†L33-L55】
- 运行链路相关文件：`lanedet_node_helper.py`、`lane_boundary_transformer.cpp`、`speedgoat_ros2_node.py`、`node_delay.py`。.【F:src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py†L201-L371】【F:src/lane_boundary_transformer/src/lane_boundary_transformer.cpp†L42-L241】【F:src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py†L586-L619】【F:src/node_delay_package/node_delay_package/node_delay.py†L26-L38】

### 建议修改内容
- 固化 3 组回归场景：
  - 直道稳定跟随；
  - 中等曲率弯道；
  - 短时漏检/单侧缺线场景。
- 每组场景输出统一指标：
  - 漏检率；
  - lane 参数帧间波动；
  - reference path 曲率/heading 平滑度；
  - steering RMS、零交叉次数；
  - camera→control P95 延迟；
  - 横向误差 P95。
- 建议把这些结果写回文档，而不是只留终端日志。

### 风险点
- 如果没有固定数据回放或固定测试路线，调参结果很难横向比较。
- 只看均值会掩盖长尾问题，必须保留 P95/P99。

### 验收指标
- 每次修改 lane-following 主链路后，都能在固定场景上复现并比较上述指标。
- 至少能证明：
  - 漏检不再直接导致控制大幅摆动；
  - reference path 的平滑改造没有明显增加转弯滞后；
  - 延迟补偿与 controller 抗抖联合后，横向误差和 steering 抖动同时改善。

---

## 推荐执行顺序（严格）

### 第一优先级
- 阶段 1：观测与诊断
- 阶段 2：Reference path 稳定化

### 第二优先级
- 阶段 3：延迟补偿

### 第三优先级
- 阶段 4：Controller 抗抖优化

### 持续进行
- 阶段 5：验证与回归

## 最后结论

当前 lane-following 主链路并不是“没有功能”，而是**有基本功能但稳定性闭环不完整**：lane detection 侧只有简单窗口均值与单侧补线，reference path 侧只有几何均值/插值/低通，controller 侧只有有限低通与 deadband，delay 侧只有粗粒度观测节点；这些都说明系统具备优化抓手，但距离“可稳定量产/稳定实车跟随”还有明显差距。后续工作必须以“数据观测 -> path 稳定 -> 延迟补偿 -> controller 抗抖 -> 回归验证”的顺序推进，避免直接跳到 controller 调参。.【F:src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py†L278-L285】【F:src/lanedet_ros2/lanedet_ros2/lanedet_node_helper.py†L265-L270】【F:src/lane_boundary_transformer/src/lane_boundary_transformer.cpp†L62-L68】【F:src/speedgoat_package/speedgoat_package/speedgoat_ros2_node.py†L194-L201】【F:src/node_delay_package/node_delay_package/node_delay.py†L15-L38】
