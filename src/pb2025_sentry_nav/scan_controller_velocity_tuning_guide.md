# 扫描控制器速度平滑调参指南

## 1. 文档目的

本文面向负责导航、电控和现场调试的工程人员，目标是让扫描控制器向下位机发布连续、稳定、可执行的速度指令，使全向底盘能够平稳完成自主导航。

本文以当前仓库的现实导航链路为依据。调参时应优先保证安全、连续和可控，再逐步提高速度和路径跟踪性能。文中参数均以 SI 单位为准：

- 线速度：`m/s`
- 角速度：`rad/s`
- 加速度：`m/s^2`
- 角加速度：`rad/s^2`
- 距离：`m`

## 2. 当前实际速度链路

现实导航中，速度指令经过以下链路后才到达下位机：

```text
定位与里程计
  -> Nav2 控制器
  -> cmd_vel_controller
  -> velocity_smoother
  -> cmd_vel_nav2_result
  -> fake_vel_transform
  -> cmd_vel_base_real_yaw
  -> rm_serial_driver
  -> 下位机底盘控制
```

各环节职责如下：

| 环节 | 主要接口 | 作用 |
|---|---|---|
| 定位与里程计 | `odometry` | 提供机器人位姿和速度反馈 |
| Nav2 控制器 | `cmd_vel_controller` | 根据局部路径计算原始速度 |
| 速度平滑器 | `velocity_smoother` | 限制速度、加速度和减速度 |
| 云台解耦 | `fake_vel_transform` | 将导航坐标系速度转换到底盘真实坐标系 |
| 串口驱动 | `/cmd_vel_base_real_yaw` | 将最终速度打包发送给下位机 |
| 下位机 | 底盘电机控制 | 执行速度闭环、限幅、急停和电机输出 |

当前现实导航配置文件为：

[`nav2_params.yaml`](/home/shit/RMUC_2/src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml)

当前配置中的 `FollowPath` 为：

```yaml
FollowPath:
  plugin: "nav2_mppi_controller::MPPIController"
```

因此，当前现场首先应调 MPPI、`velocity_smoother`、局部代价地图和 `fake_vel_transform`。仓库中的 `pb_omni_pid_pursuit_controller` 只有在配置切换到该插件后才会生效。

### 2.1 关键话题和坐标系

| 名称 | 类型或含义 | 调试重点 |
|---|---|---|
| `odometry` | `nav_msgs/msg/Odometry` | 位姿、速度和时间戳是否连续 |
| `cmd_vel_controller` | 控制器原始速度 | 控制器是否正常输出 |
| `cmd_vel_nav2_result` | 平滑后的导航速度 | 是否满足加速度限制 |
| `/cmd_vel_base_real_yaw` | 发往底盘驱动的最终速度 | 方向、符号、频率和丢帧 |
| `gimbal_yaw` | 云台真实参考坐标系 | 云台旋转时会发生变化 |
| `gimbal_yaw_fake` | 导航使用的伪底盘坐标系 | yaw 固定用于稳定导航 |
| `local_plan` | 控制器局部路径 | 与里程计的同步是否正常 |
| `cmd_spin` | 扫描或固定旋转速度 | 是否被叠加到最终角速度 |

## 3. 调参前安全检查

调参前必须先完成以下检查。任何一项不满足时，不要通过提高控制器增益或速度上限来掩盖问题。

### 3.1 启动和插件检查

- 确认只启动一套 `controller_server`、`velocity_smoother` 和 `fake_vel_transform`。
- 确认当前加载的 `FollowPath.plugin` 是预期插件。
- 如果使用组合节点和普通节点两种启动方式，确认没有重复加载同名节点。
- 确认 `controller_server` 的输出 remap 到 `cmd_vel_controller`。
- 确认 `velocity_smoother` 的输入为 `cmd_vel_controller`，输出为 `cmd_vel_nav2_result`。
- 确认 `fake_vel_transform` 输入为 `cmd_vel_nav2_result`，输出为 `cmd_vel_base_real_yaw`。

### 3.2 话题和频率检查

建议先检查话题是否存在、类型是否正确、发布频率是否稳定：

```bash
ros2 topic info /cmd_vel_controller
ros2 topic info /cmd_vel_nav2_result
ros2 topic info /cmd_vel_base_real_yaw
ros2 topic info /odometry

ros2 topic hz /cmd_vel_controller
ros2 topic hz /cmd_vel_nav2_result
ros2 topic hz /cmd_vel_base_real_yaw
```

控制器和速度平滑器的目标频率当前均约为 `20 Hz`。现场应重点观察是否存在长时间无消息、周期性卡顿或频率大幅波动。

### 3.3 定位、TF 和时间检查

- 机器人静止时，RViz 中的 `gimbal_yaw_fake` 和机器人位姿不应持续跳动。
- `odometry` 的位置、姿态和线速度应连续，不能出现明显的单帧突变。
- 检查 `gimbal_yaw -> gimbal_yaw_fake` 的 TF 是否持续发布。
- 检查 `local_plan`、`odometry` 和速度消息的时间戳是否处于同一时间源。
- 现实导航配置使用 `use_sim_time: False`，现场不要混用仿真时钟。

### 3.4 下位机检查

在闭环测试前确认：

- 线速度和角速度的单位与 ROS 约定一致。
- `x`、`y`、`z` 轴方向和正负号一致。
- 下位机限幅不应与上位机限幅严重冲突。
- 下位机具备通信超时停车和急停逻辑。
- 下位机实际接收频率稳定，串口没有丢包、校验错误或阻塞。
- 低速死区、静摩擦补偿和电机速度环已经单独调好。

## 4. 推荐调参顺序

每次只修改一组同类参数，并记录修改前后值、测试路线和结果。

1. **下位机速度闭环与限幅**：先确认给定速度和实际速度方向、比例、响应时间正确。
2. **ROS 速度话题与频率**：确认控制器、平滑器、云台解耦和串口链路没有丢帧或重复发布。
3. **`velocity_smoother`**：先调起步、减速、角速度和角加速度。
4. **MPPI**：再调预测时域、采样噪声、速度上限和路径跟随代价。
5. **局部代价地图**：最后调障碍物更新、膨胀和避障惩罚。
6. **`fake_vel_transform`**：在云台静止和连续自旋两种工况下分别验证。
7. **自研 PID Pursuit**：只有切换控制器插件后才进行 PID、前视距离和曲率限速调节。
8. **综合验收**：使用固定路线、窄通道、动态障碍和连续导航点复测。

## 5. 速度平滑器调参

当前配置位于 `velocity_smoother`：

```yaml
velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0
    feedback: "OPEN_LOOP"
    max_velocity: [2.5, 2.5, 3.0]
    min_velocity: [-2.5, -2.5, -3.0]
    max_accel: [2.5, 2.5, 5.0]
    max_decel: [-2.5, -2.5, -5.0]
    odom_topic: "odometry"
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0
```

### 5.1 参数含义

| 参数 | 当前值 | 调参作用 |
|---|---:|---|
| `smoothing_frequency` | `20.0` | 输出速度的整形频率，通常与控制器频率一致 |
| `feedback` | `OPEN_LOOP` | 根据指令进行整形，不使用实际速度闭环修正 |
| `max_velocity` | `[2.5, 2.5, 3.0]` | `x`、`y`、yaw 最大速度 |
| `min_velocity` | `[-2.5, -2.5, -3.0]` | `x`、`y`、yaw 最小速度 |
| `max_accel` | `[2.5, 2.5, 5.0]` | 正向加速度上限 |
| `max_decel` | `[-2.5, -2.5, -5.0]` | 减速度上限，负值表示减速 |
| `odom_duration` | `0.1` | 速度反馈使用的里程计时间窗口 |
| `deadband_velocity` | 全为 `0.0` | 小速度死区，过大可能导致低速断续 |
| `velocity_timeout` | `1.0` | 长时间没有输入速度时进入安全处理 |

### 5.2 现象和调整方法

| 现象 | 优先检查 | 调整方向 |
|---|---|---|
| 起步冲击明显 | `max_accel` | 线速度可从 `[2.5, 2.5]` 降至 `[1.5, 1.5]` 附近 |
| 刹车过猛 | `max_decel` | 减小绝对值，例如从 `-2.5` 调至 `-2.0` |
| 转向突然甩头 | yaw 的 `max_accel` | 从 `5.0` 降至 `3.0` 至 `4.0` |
| 转弯角速度过高 | `max_velocity[2]` | 降低 yaw 最大速度 |
| 低速运动断续 | `deadband_velocity` | 保持较小或为 `0.0`，检查下位机死区 |
| 速度消息中断后仍保持运动 | `velocity_timeout` | 缩短超时时间，并确认下位机也有超时停车 |

`velocity_smoother` 是最终导航速度的整形层。它的加速度限制应与控制器的运动模型保持一致。当前 MPPI 使用：

```yaml
ax_max: 3.5
ay_max: 3.5
az_max: 3.0
```

而速度平滑器的线加速度限制为 `2.5`。这意味着 MPPI 预测的加速能力大于最终实际输出能力，可能造成预测与执行不一致。以稳定为优先时，建议将 MPPI 的 `ax_max/ay_max` 与速度平滑器统一到经过下位机验证的保守值；不要只放宽速度平滑器来追求响应速度。

## 6. MPPI 控制器调参

### 6.1 主要运动参数

当前配置：

```yaml
time_steps: 50
model_dt: 0.05
batch_size: 1200
vx_std: 0.6
vy_std: 0.6
wz_std: 0.2
vx_max: 2.5
vy_max: 2.5
wz_max: 1.5
vx_min: -2.5
vy_min: -2.5
wz_min: -1.5
ax_max: 3.5
ay_max: 3.5
az_max: 3.0
ax_min: -3.5
ay_min: -3.5
az_min: -3.0
temperature: 0.5
prune_distance: 1.7
transform_tolerance: 0.3
```

`time_steps * model_dt` 为预测时域，当前为：

```text
50 * 0.05 = 2.5 s
```

一般规律：

- 预测时域太短：遇到弯道或障碍时反应急，容易临近障碍才转向。
- 预测时域太长：可能过早绕障，计算量增加，路径跟随显得犹豫。
- `batch_size` 越大，采样结果通常越稳定，但计算量线性增加。
- `vx_std`、`vy_std`、`wz_std` 越大，探索范围越广，但输出可能更活跃。
- `temperature` 越小，选择更果断；越大，输出更平均、更保守。
- `prune_distance` 用于剪去已通过的路径，过小可能重复跟踪后方路径，过大可能提前丢失有效参考。

### 6.2 MPPI 现象调节表

| 现象 | 优先参数 | 建议 |
|---|---|---|
| 直线左右摆动 | `vx_std`、`vy_std`、`PathFollowCritic.cost_weight` | 先减小采样噪声；若仍摆动，再适当减小路径跟随权重 |
| 轨迹跟随不够果断 | `temperature`、`PathFollowCritic` | 小幅降低 `temperature` 或提高路径跟随权重 |
| 转弯前反应太晚 | `time_steps`、`model_dt`、`PathAlignCritic` | 适当增加预测时域或路径方向代价 |
| 转弯过早、绕路明显 | 预测时域、障碍物代价 | 缩短预测时域或降低非必要的障碍排斥 |
| 角速度变化过快 | `wz_max`、`az_max`、`wz_std` | 先降角加速度，再降角速度上限 |
| 接近目标点头或来回修正 | `GoalCritic`、目标容差、接近速度 | 降低接近目标时线速度，检查目标判定是否过严 |
| 障碍附近反复犹豫 | `ObstaclesCritic`、`CostCritic`、代价地图频率 | 先确认代价地图连续更新，再调整代价权重 |

### 6.3 Critics 调参原则

当前启用：

```yaml
critics:
  - ConstraintCritic
  - CostCritic
  - GoalCritic
  - PathFollowCritic
  - PathAlignCritic
  - ObstaclesCritic
```

- `ConstraintCritic`：限制速度和运动约束，不应为了追求速度而关闭。
- `GoalCritic`：提供到达目标的驱动力。过低可能到不了目标，过高可能临近目标时动作过激。
- `PathFollowCritic`：控制轨迹跟随路径的程度。过高容易贴线修正、产生摆动；过低可能偏离路径。
- `PathAlignCritic`：使运动方向与路径一致。全向底盘使用 `use_path_orientations: false`，表示优先使用运动方向而不是车体朝向。
- `ObstaclesCritic`：提供额外的障碍排斥。过高会在障碍附近犹豫或绕远，过低会增加擦碰风险。
- `CostCritic`：使用代价地图梯度进行避障。应与障碍层更新频率和机器人实际外形共同验证。

## 7. 局部代价地图调参

当前局部代价地图配置：

```yaml
update_frequency: 25.0
publish_frequency: 8.0
global_frame: odom
robot_base_frame: gimbal_yaw_fake
width: 15
height: 15
```

调参重点：

- `update_frequency` 决定代价地图内部更新速度，当前为 `25 Hz`。
- `publish_frequency` 决定代价地图对外发布频率，当前为 `8 Hz`。
- 如果障碍物信息明显滞后或 MPPI 在障碍附近轨迹跳变，可在算力允许时提高 `publish_frequency`，但必须观察 CPU 占用和控制器频率。
- `robot_base_frame` 应保持为 `gimbal_yaw_fake`，避免云台旋转直接改变导航参考方向。
- 局部地图尺寸必须覆盖控制器的有效预测范围；尺寸过小会导致路径或碰撞检查超出地图。
- 动态障碍预测时间过短时，可能出现来不及避让的问题；过长则可能过度绕行。

代价地图问题的典型特征是：RViz 中障碍物或膨胀层本身跳变，随后 MPPI 输出同步跳变。此时应先修复传感器、点云、TF 和代价地图更新，再调整 MPPI 权重。

## 8. 云台解耦和扫描速度调参

`fake_vel_transform` 的作用是：导航在 `gimbal_yaw_fake` 坐标系中计算速度，再根据当前 yaw 将线速度旋转到真实底盘坐标系，并将 `cmd_spin` 叠加到 `angular.z`。

当前链路：

```text
cmd_vel_nav2_result
  -> fake_vel_transform
  -> cmd_vel_base_real_yaw
```

关键配置：

```yaml
fake_vel_transform:
  odom_topic: "odometry"
  robot_base_frame: "gimbal_yaw"
  fake_robot_base_frame: "gimbal_yaw_fake"
  input_cmd_vel_topic: "cmd_vel_nav2_result"
  output_cmd_vel_topic: "cmd_vel_base_real_yaw"
  cmd_spin_topic: "cmd_spin"
  init_spin_speed: 0.0
```

注意事项：

- `cmd_spin` 会直接叠加到最终角速度，不能把扫描角速度重复加到导航控制器输出中。
- 云台自旋时，必须保证 `odometry` 和 `local_plan` 能够近似同步，否则速度变换可能使用过期 yaw。
- `fake_robot_base_frame` 的 yaw 被设计为固定方向，导航应使用该坐标系而不是直接使用持续旋转的真实云台坐标系。
- 零速度指令会被直接转换并发布；控制器激活超时后，未同步的速度也会走直接发布路径。
- 云台自旋测试应同时观察 `cmd_vel_nav2_result` 和 `cmd_vel_base_real_yaw`，区分是导航控制器抖动还是坐标变换后的方向抖动。

## 9. 自研 Omni PID Pursuit 控制器

自研控制器位于：

[`pb_omni_pid_pursuit_controller`](/home/shit/RMUC_2/src/pb2025_sentry_nav/pb_omni_pid_pursuit_controller)

切换插件后，配置形式为：

```yaml
FollowPath:
  plugin: "pb_omni_pid_pursuit_controller::OmniPidPursuitController"
```

### 9.1 控制逻辑

控制器每个控制周期执行以下步骤：

1. 将全局路径转换到机器人局部坐标系并裁剪。
2. 根据当前速度计算前视距离。
3. 获取前视点。
4. 使用平移 PID 计算线速度大小。
5. 使用旋转 PID 根据路径末端朝向误差计算角速度。
6. 根据路径曲率限制线速度。
7. 根据到目标点的距离进一步降低线速度。
8. 将线速度分解为 `linear.x` 和 `linear.y`。
9. 进行局部路径碰撞检查并输出速度。

其中：

```text
linear.x = lin_vel * cos(theta_dist)
linear.y = lin_vel * sin(theta_dist)
```

因此该控制器是全向底盘控制器，前视点方向直接决定平移速度方向。

### 9.2 参数调节

| 参数组 | 参数 | 作用 |
|---|---|---|
| 平移 PID | `translation_kp/ki/kd` | 调整平移误差响应、稳态误差和阻尼 |
| 旋转 PID | `rotation_kp/ki/kd` | 调整朝向误差响应、稳态误差和阻尼 |
| 速度限幅 | `v_linear_min/max` | 限制平移 PID 输出 |
| 角速度限幅 | `v_angular_min/max` | 限制旋转 PID 输出 |
| 前视距离 | `lookahead_dist` | 未启用速度缩放时使用的固定距离 |
| 速度前视 | `use_velocity_scaled_lookahead_dist` | 是否根据当前速度动态调整前视距离 |
| 前视范围 | `min_lookahead_dist/max_lookahead_dist` | 动态前视距离的上下限 |
| 前视时间 | `lookahead_time` | 动态前视距离约为速度乘以前视时间 |
| 前视插值 | `use_interpolation` | 是否在路径点之间插值，提高前视点连续性 |
| 原地对齐 | `use_rotate_to_heading` | 是否先旋转到路径方向再平移 |
| 对齐阈值 | `use_rotate_to_heading_treshold` | 触发原地对齐的角度阈值 |
| 接近目标 | `min_approach_linear_velocity` | 接近目标时的最低线速度 |
| 接近距离 | `approach_velocity_scaling_dist` | 开始线性降速的距离 |
| 曲率阈值 | `curvature_min/max` | 触发和加强曲率限速的阈值 |
| 曲率降速 | `reduction_ratio_at_high_curvature` | 高曲率时保留的速度比例 |
| 曲率采样 | `curvature_forward_dist/backward_dist` | 用于三点圆拟合的前后距离 |
| 限速变化率 | `max_velocity_scaling_factor_rate` | 限制曲率降速变化速度 |

推荐顺序：

1. 将 `translation_ki` 和 `rotation_ki` 暂设为较小值，先调 `kp`。
2. 增加 `kp` 直到跟踪响应足够快但未出现持续振荡。
3. 使用 `kd` 抑制过冲和左右摆动。
4. 只有存在明显稳态误差时再增加 `ki`，并观察积分饱和。
5. 调整前视距离：摆动时增大前视距离，转弯切角或响应迟钝时减小前视距离。
6. 最后启用曲率限速和接近目标减速。

### 9.3 实现限制

- 控制器的 `setSpeedLimit()` 当前只打印警告，未真正实现速度限制。
- PID 输出通过 `v_linear_*` 和 `v_angular_*` 限幅。
- `PID::calculate()` 内部积分量限制在 `[-1, 1]`。
- 高曲率时线速度会逐周期变化，变化率由 `max_velocity_scaling_factor_rate` 限制。
- 控制器源码注册了动态参数回调，但修改 `translation_kp/ki/kd` 或 `rotation_kp/ki/kd` 时只更新控制器成员变量，没有重建已经创建的 PID 对象。现场动态修改 PID 增益后，应重启 `controller_server` 再验证；不要把动态修改立即生效当作已确认行为。
- 自研控制器的局部碰撞检查只将 `LETHAL_OBSTACLE` 视为碰撞，代价地图安全余量仍需由规划器和代价层保证。

## 10. 现象到参数速查

| 现象 | 优先检查 | 调整方向 |
|---|---|---|
| 起步冲击明显 | `velocity_smoother.max_accel` | 降低线速度加速度上限 |
| 刹车过猛 | `velocity_smoother.max_decel` | 降低减速度绝对值 |
| 直线左右摆动 | MPPI 路径跟随、`vx_std`、`vy_std` | 降低采样噪声或减小路径跟随的过度修正 |
| 转弯突然甩头 | `wz_max`、`az_max`、路径曲率 | 降低角速度或角加速度 |
| 遇障碍反复犹豫 | `ObstaclesCritic`、`CostCritic`、局部代价地图 | 先确认代价地图更新，再调整惩罚权重 |
| 到点前后点头 | `GoalCritic`、接近目标速度、目标容差 | 增大减速距离并降低接近速度 |
| 云台自旋时底盘方向抖动 | `fake_vel_transform`、TF、里程计 | 检查伪底盘坐标系和同步链路 |
| 速度突然归零 | 碰撞检查、`failure_tolerance`、超时 | 检查控制器日志、碰撞代价和速度发布者 |
| 低速走走停停 | 下位机死区、`deadband_velocity`、最小接近速度 | 先校准下位机死区，再调整上位机最小速度 |
| 到不了目标但一直有速度 | `GoalCritic`、目标容差、路径末端方向 | 检查目标判定和路径末端姿态 |
| 贴障过近 | `collision_margin_distance`、膨胀层、`ObstaclesCritic` | 增大安全余量或障碍惩罚 |
| 绕障距离过大 | 障碍排斥权重、膨胀半径、预测时间 | 在安全余量允许时逐步降低排斥或预测范围 |

## 11. 固定测试项目

每轮调参至少执行以下测试：

1. 空载直线加速和减速。
2. 原地旋转。
3. 直角转弯。
4. S 弯路线。
5. 窄通道避障。
6. 接近目标点停车。
7. 云台静止导航。
8. 云台连续自旋导航。
9. 连续多个导航点巡航。

测试时同时记录：

- `/cmd_vel_controller`
- `/cmd_vel_nav2_result`
- `/cmd_vel_base_real_yaw`
- `/odometry`
- `/local_plan`
- 控制器日志和 recovery 日志

## 12. 验收指标

建议将以下指标作为稳定版本的最低验收标准：

- 控制器和最终速度输出频率稳定在约 `20 Hz`。
- 最终下发速度没有明显丢帧和长时间超时。
- 速度变化率不超过对应的加速度和减速度上限。
- 直线跟踪没有持续左右摆动。
- 转弯没有明显甩尾、甩头或反复修正。
- 接近目标时线速度连续下降。
- 没有不必要的 recovery 触发。
- 测试过程中无碰撞、无越界、无速度符号异常。
- `/cmd_vel_base_real_yaw` 的运动方向与下位机实际执行方向一致。
- 云台连续自旋时，底盘平移方向仍保持稳定。
- 速度停止、通信超时和急停均能使底盘进入安全状态。

## 13. 调参记录模板

| 日期 | 测试场景 | 软件版本 | 参数文件 | 修改参数 | 修改前值 | 修改后值 | 观察现象 | 测量指标 | 是否保留 | 操作者备注 |
|---|---|---|---|---|---|---|---|---|---|---|
|  |  |  |  |  |  |  |  |  |  |  |
|  |  |  |  |  |  |  |  |  |  |  |
|  |  |  |  |  |  |  |  |  |  |  |

建议每次只改变一个主要因素。例如先固定 MPPI 参数，只改变 `max_accel`；确认起步稳定后，再调 `max_decel`；完成速度平滑后，再进入路径跟踪和避障参数。

## 14. 现场排障最短路径

出现速度不稳定时，按以下顺序定位：

1. 看 `/cmd_vel_base_real_yaw`：确认最终输出是否已经抖动。
2. 对比 `/cmd_vel_nav2_result`：若这里正常而最终输出抖动，检查 `fake_vel_transform`、TF 和云台 yaw。
3. 对比 `/cmd_vel_controller`：若平滑前抖动，检查 MPPI、局部路径和代价地图。
4. 看 `/odometry`：静止时若位姿或速度跳动，先处理定位和时间同步。
5. 看 `local_plan`：路径跳变时检查规划频率、代价地图和 recovery。
6. 看控制器日志：确认是否有碰撞检测、TF 超时、控制器失败或 progress checker 触发。
7. 最后才修改 PID、MPPI 权重或速度上限。

## 15. 相关源码和配置

- [现实导航参数](/home/shit/RMUC_2/src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml)
- [导航启动与速度 remap](/home/shit/RMUC_2/src/pb2025_sentry_nav/pb2025_nav_bringup/launch/navigation_launch.py)
- [自研全向 PID 控制器实现](/home/shit/RMUC_2/src/pb2025_sentry_nav/pb_omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp)
- [PID 实现](/home/shit/RMUC_2/src/pb2025_sentry_nav/pb_omni_pid_pursuit_controller/src/pid.cpp)
- [云台解耦速度变换实现](/home/shit/RMUC_2/src/pb2025_sentry_nav/fake_vel_transform/src/fake_vel_transform.cpp)
- [串口速度订阅实现](/home/shit/RMUC_2/src/rm_serial_driver/src/rm_serial_driver.cpp)
