# 导航全链路"丝滑"调参指南

> 适用项目：pb2025_sentry_nav（哨兵全向车导航）
> 目标：在电控调好的前提下，让车辆行驶平稳流畅（不画龙、不顿挫、不犹豫、不点头）
> 核心结论：**丝滑不是只调 MPPI，而是全链路每一环都不引入跳变**

---

## 一、全链路数据流概览

丝滑 = 每一环都不引入跳变。任何一环输出突变，都会传导到最终运动。

```
FAST-LIO定位 → odometry/TF → 全局代价地图(5Hz) → ThetaStar规划(1Hz重规划)
                                                        ↓
                                              SimpleSmoother平滑
                                                        ↓
本地代价地图(25Hz) → MPPI跟踪(20Hz) → velocity_smoother(20Hz) → fake_vel_transform → 底盘
```

**八大影响因素**（按影响优先级排序）：

| 序号 | 环节 | 主要影响的"不丝滑"现象 |
|---|---|---|
| 1 | 反馈层：定位/TF | 抽搐式运动（最隐蔽） |
| 2 | 行为层：replanning 频率 | 路径突然跳变 |
| 3 | 行为层：recovery 触发 | "停→转→走"顿挫 |
| 4 | 路径层：smoother | 锯齿/折线转弯 |
| 5 | 代价地图层 | 绕障轨迹突变 |
| 6 | MPPI 跟踪层 | 画龙/振荡/犹豫 |
| 7 | velocity_smoother 整形层 | 起步冲/刹车闯/转向突兀 |
| 8 | fake_vel_transform 层 | 云台解耦抖动（RM 特有） |

---

## 二、反馈层：定位/TF 抖动（最高优先级）

**现状**：定位来自 `FAST-LIO`，经 `loam_interface` 发布到 `odom`，再由 `small_gicp_relocalization` 做 map 系重定位。

**根因**：MPPI 每 0.05s 用当前位姿做采样起点。若 odometry 有跳变，采样起点不连续，车表现为**抽搐式**运动——这是最容易被误判为"MPPI 没调好"的病根。

**现象识别**：原地静止时，RViz 里 `base_footprint` 是否在抖动？静止都抖则问题在定位，调 MPPI 无用。

**参数调整方案**：

| 环节 | 参数 | 现状 | 调整建议 |
|---|---|---|---|
| FAST-LIO | `acc_cov` / `gyr_cov` | 0.1 / 0.1 | 定位噪声大时增大到 0.15-0.2，让滤波更平滑 |
| FAST-LIO | `extrinsic_est_en` | false | 保持 false，外参估计开启会引入漂移 |
| relocalization | `tf_future_offset_sec` | 0.2 | 若 map→odom 跳变，检查时间补偿是否导致位姿突变 |
| relocalization | `max_dist_sq` | 2.5 | 配准距离阈值，频繁跳变可适当收紧 |

**关键动作**：先用 `ros2 topic echo /odometry` 或 RViz 观察定位是否平滑，确认这层干净后再往下调。

---

## 三、行为层：replanning 频率与 recovery 触发

### 3.1 全局路径每 1 秒重规划 = 参考路径突然跳变

在 `navigate_to_pose_w_replanning_and_recovery.xml` 中：

```xml
<RateController hz="1.0">
  <RecoveryNode name="ComputePathToPose" number_of_retries="1">
    <ComputePathToPose .../>
```

**根因**：`RateController hz=1.0` 意味着每 1 秒才重新计算全局路径。若新路径与旧路径差异大（障碍物移动、位姿漂移），MPPI 跟踪的参考线会**瞬间跳变**，导致车身猛地转向。

**调整方案**：
- `hz` 从 `1.0` 提高到 `2.0` 或 `3.0`，让路径更新更连续（NUC 算力足够，代价可接受）
- 配合 smoother 让新路径与旧路径之间平滑过渡

### 3.2 recovery 触发 = "停→转→走"的顿挫

recovery 动作为 `ClearEntireCostmap` 和 `BackUp`（后退 0.5m，速度 1.0）。

**根因**：`progress_checker` 判停太敏感，或路径被障碍挡住，会频繁触发 `BackUp`，产生明显顿挫。

**相关参数**：

```yaml
failure_tolerance: 0.15          # 控制器失败容错时间
required_movement_radius: 0.5    # 前进检测：需移动的最小距离
movement_time_allowance: 10.0    # 允许的移动时间窗口
```

**调整方案**：
- `failure_tolerance` 从 `0.15` 提高到 `0.3-0.5`，避免控制器短暂卡顿就触发 recovery
- `required_movement_radius` 从 `0.5` 降到 `0.3`，让"是否前进"判断更宽松，减少误判停滞

---

## 四、路径层：smoother 是否真的让路径平滑

**现状**：`SimpleSmoother`，且 BT 中已挂 `nav2_smooth_path_action_bt_node`。

```yaml
simple_smoother:
  plugin: "nav2_smoother::SimpleSmoother"
  tolerance: 1.0e-10
  max_its: 1000
  do_refinement: True
```

**分析**：
- `tolerance: 1.0e-10` 很小，平滑器会迭代到几乎完全收敛，路径会非常平滑（本身是好的）
- 但 `SimpleSmoother` 只做几何平滑，不考虑曲率约束和动力学。Theta* 规划的尖角路径，平滑后仍可能有过急转角

**Theta* 规划器参数**：

```yaml
how_many_corners: 8     # 邻居节点数，越高路径越"直"
w_euc_cost: 1.0         # 欧氏距离权重
w_traversal_cost: 15.0  # 障碍代价权重
w_heuristic_cost: 1.0   # 启发式权重
```

**调整方案**：
- `how_many_corners: 8` 已较高（路径更直、更少折线），保持即可
- 若转弯仍有"折线感"，可换 `nav2_smoother::SavitzkyGolaySmoother`（更强平滑）

**判断信号**：RViz 里看 `/plan` 全局路径。路径是光滑弧线则本层没问题；有锯齿则问题在 smoother。

---

## 五、代价地图层：障碍信息抖动

**现状**：local_costmap 更新 25Hz，发布 8Hz。

**根因**：`publish_frequency: 8.0` 意味着代价地图 8Hz 才发布一次。MPPI 的 ObstaclesCritic/CostCritic 依赖代价地图打分，更新不及时会导致**避障轨迹突然改变**。

**调整方案**：

```yaml
# local_costmap
publish_frequency: 8.0    # 建议提高到 15.0-20.0
update_frequency: 25.0    # 可保持

# inflation 参数（影响贴障和绕障平滑度）
cost_scaling_factor: 5.5  # 越大越惩罚贴障，转弯越"绕"
inflation_radius: 0.65    # 膨胀半径，越大越早开始绕
```

**关键点**：`cost_scaling_factor: 5.5` 偏大，会让机器人**提前大幅绕开障碍**，转弯更急。想更丝滑可降到 `3.0-4.0`。

---

## 六、MPPI 跟踪层：画龙/振荡的根源

> ⚠️ 务必先确认前五层没问题再调这里。

### 6.1 关键矛盾：MPPI 加速度上限 vs velocity_smoother 上限不一致

这是配置里**最隐蔽的丝滑杀手**：

```yaml
# MPPI
ax_max: 3.5
ay_max: 3.5

# velocity_smoother（在 MPPI 之后！）
max_accel: [2.5, 2.5, 5.0]
max_decel: [-2.5, -2.5, -5.0]
```

**根因**：MPPI 采样时认为"我最高能加速到 3.5"，但最终输出被 velocity_smoother 掐到 2.5。导致：

```
MPPI 预测：3.5 加速 → 2.5s 后到达某位置
真实执行：2.5 加速 → 2.5s 后到达更近的位置
→ 预测和现实不符 → MPPI 每个周期都在"纠错" → 车表现犹豫、小步修正
```

**调整方案（二选一，务必对齐）**：

方案 A（推荐）：统一到 2.5，让 MPPI 知道真实能力

```yaml
ax_max: 2.5
ay_max: 2.5
ax_min: -2.5
ay_min: -2.5
```

方案 B：统一到 3.5，让 smoother 放宽

```yaml
max_accel: [3.5, 3.5, 5.0]
max_decel: [-3.5, -3.5, -5.0]
```

> 推荐方案 A：2.5 的加速度对丝滑更友好，velocity_smoother 是最后一道闸，让它更保守更安全。

### 6.2 参数逐项调整（针对"画龙/振荡/犹豫"）

| 参数 | 现状 | 若出现"画龙/振荡" | 若出现"犹豫/迟滞" |
|---|---|---|---|
| `temperature` | 0.5 | 调大到 0.7-0.8（更谨慎，减少抖） | 调小到 0.3-0.4（更果断） |
| `vx_std` | 0.6 | 降到 0.3-0.4（减少指令噪声） | 保持或微增 |
| `vy_std` | 0.6 | 降到 0.3-0.4 | 保持 |
| `wz_std` | 0.2 | 保持 | 微增到 0.3 增强转向探索 |
| `PathFollowCritic.cost_weight` | 6.0 | 贴线过头导致振荡，降到 4-5 | 乱跑则升到 7-8 |
| `PathAlignCritic.cost_weight` | 4.0 | 接近目标画圈，降到 2-3 | 保持 |
| `GoalCritic.cost_weight` | 20.0 | 保持 | 保持 |
| `time_steps` × `model_dt` | 50×0.05=2.5s | 过度前瞻绕路，降到 30-40 | 目光短浅急转，升到 60-80 |

### 6.3 "停下时来回点头"专项

**根因**：接近目标时，`GoalCritic` 和 `PathFollowCritic` 相互拉扯，MPPI 在"继续前进"和"已到目标"之间摇摆。

**调整方案**：放大 goal_checker 到达容差，让车"提前认为到了"：

```yaml
general_goal_checker:
  xy_goal_tolerance: 0.25  # 放大到 0.35-0.4
```

---

## 七、velocity_smoother 整形层：起步冲/刹车闯的根源

**现状**：

```yaml
smoothing_frequency: 20.0
feedback: "OPEN_LOOP"
max_velocity: [2.5, 2.5, 3.0]
max_accel: [2.5, 2.5, 5.0]
max_decel: [-2.5, -2.5, -5.0]
```

`feedback: "OPEN_LOOP"` 表示开环整形，只对速度指令限幅+平滑，已是最平滑模式。

**调整方案**：

| 现象 | 参数 | 调整 |
|---|---|---|
| 起步猛冲 | `max_accel` | 降低到 `[1.5, 1.5, 4.0]` |
| 刹车前闯 | `max_decel` | 绝对值降低到 `[-2.0, -2.0, -4.0]` |
| 方向突变（yaw 猛转） | `max_velocity[2]` 和 `max_accel[2]` | yaw 加速度 `5.0 → 3.0` |

> **注意**：yaw 的 `max_accel: 5.0` 远大于 x/y 的 2.5，是**转向突兀**的常见来源。建议降到 `3.0-4.0` 统一。

---

## 八、fake_vel_transform 层：云台解耦的隐藏抖动源

**现状**：

```yaml
fake_vel_transform:
  odom_topic: "odometry"
  robot_base_frame: "gimbal_yaw"
  fake_robot_base_frame: "gimbal_yaw_fake"
  input_cmd_vel_topic: "cmd_vel_nav2_result"
  output_cmd_vel_topic: "cmd_vel_base_real_yaw"
  cmd_spin_topic: "cmd_spin"
```

**根因**：该节点把 Nav2 输出的 `cmd_vel`（在 `gimbal_yaw_fake` 假坐标系下）转换成真实底盘 yaw 下的 `cmd_vel_base_real_yaw`。这是**云台独立旋转 + 底盘全向移动**的解耦关键。

**怎么影响丝滑**：若 `gimbal_yaw`（真实云台角）和 `gimbal_yaw_fake`（导航假角）之间的变换有噪声或延迟，转换后的速度指令会**周期性抖动**。这是 RM 机器人特有抖动源，普通 Nav2 调参文章不会提到。

**调整方案**：
- 确认 `gimbal_yaw` 的 TF 来源（云台反馈）是否平滑，云台角度噪声大会直接传导到 `cmd_vel_base_real_yaw`
- 检查该节点是否对云台角度做了**低通滤波**，若没有，在这里加滤波是治本

---

## 九、诊断优先级总结

按此顺序排查，**不要一上来就拧 MPPI**：

```
第 1 步：原地看 odometry/TF 抖不抖 → 抖则修定位，别碰导航
第 2 步：看 /plan 全局路径是否平滑 → 锯齿则调 smoother/planner
第 3 步：看是否频繁触发 BackUp/自旋 → 是则调 progress_checker 和 failure_tolerance
第 4 步：检查 MPPI 加速度上限 vs velocity_smoother 上限是否对齐 → 不对齐先统一
第 5 步：看 cmd_vel_base_real_yaw 是否周期性抖动 → 抖则查 fake_vel_transform/云台TF
第 6 步：最后才细调 MPPI 的 temperature/std/critics
```

---

## 十、最小改动集（快速见效）

按"改动少、见效快"排序：

| 序号 | 改动 | 文件 | 作用 |
|---|---|---|---|
| 1 | MPPI `ax_max/ay_max` 3.5 → 2.5 | nav2_params.yaml | 消除"预测 vs 现实"矛盾 |
| 2 | velocity_smoother `max_accel[2]` 5.0 → 3.0 | nav2_params.yaml | 消除转向突兀 |
| 3 | `xy_goal_tolerance` 0.25 → 0.35 | nav2_params.yaml | 消除目标点"点头" |
| 4 | `RateController hz` 1.0 → 2.0 | navigate_to_pose_w_replanning_and_recovery.xml | 减少路径跳变 |
| 5 | MPPI `temperature` 0.5 → 0.7，`vx_std` 0.6 → 0.4 | nav2_params.yaml | 消除画龙（最后才动） |

---

## 附：调参方法论

- **单变量法**：一次只改一个参数，其余固定，记录效果再决定保留或回滚
- **评估指标**：用数据而非感觉判断丝滑
  - `jerk`（加速度变化率）：越小越平稳，是"丝滑"的核心指标
  - 收敛时间：越快越好
  - 路径贴合度：实际轨迹 vs 全局路径偏差
- **服务调参命令**：

```bash
ros2 param get /controller_server FollowPath.temperature
ros2 param set /controller_server FollowPath.temperature 0.7
```
