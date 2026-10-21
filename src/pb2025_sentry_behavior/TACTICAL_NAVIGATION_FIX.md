# 决策导航目标与误急停修复说明

## 1. 问题表现

本次修改处理两个现场问题：

1. 决策终端显示推荐动作发生变化，但导航侧没有收到新的导航点。
2. 第一次动作选择后，其余周期频繁变为 `HOLD_SAFE`（急停/安全保持）。

旧的 `start_ros_terminals.sh` 使用通用启动入口。该入口与实车专用入口的 tick
频率、地图校验和导航 Action 配置不同，容易让现场运行方式与实车验收配置不一致。
同时，导航动作节点在第一次 tick 时立即检查 Action 服务；Nav2 尚在启动时会直接将
动作判为失败。裁判安全输入的原超时为 1000 ms，与约 1 Hz 的报文周期重合，少量调度
抖动也会被判断为 `stale_*`，随后评分器和执行器都会进入 `HOLD_SAFE`。

终端原日志中的 `reasons=3 rejected=6` 只是“3 条选择原因、6 个未选候选”的数量，
不是失败码。`TacticalScorer` 输出的是推荐动作，真正发给 Nav2 的动作应以
`/tactical_execution` 和 `/tactical_navigation_status` 为准。

## 2. 修改内容与原因

### 2.1 统一实车启动入口

`scripts/start_ros_terminals.sh` 改为启动：

```bash
ros2 launch pb2025_sentry_behavior sentry_decision_real.launch.py \
  navigation_action_name:=/navigate_to_pose team_color:=blue
```

使用绝对路径 `/navigate_to_pose`，避免决策节点使用 ROS namespace 后把相对名称解析成
`/<namespace>/navigate_to_pose`，而 Nav2 实际仍位于根命名空间。

### 2.2 等待 Nav2 Action 服务

新增参数：

```yaml
navigation_server_wait_timeout_ms: 10000
```

每个导航动作开始后先等待 Nav2 Action 服务，默认最多等待 10 秒。等待期间动作保持
`RUNNING`，不会被记为失败或进入动作冷却；服务就绪后目标只发送一次。超过门限后才以
`FAILED/navigation_server_unavailable` 结束。

服务等待时间和 `navigation_timeout_ms` 分开计时，因此 Nav2 启动慢不会消耗实际行驶
时间。原有取消屏障保持不变：旧目标尚未返回终态时，不允许发送新目标，防止两个控制器
任务重叠。

### 2.3 调整实车安全输入超时

`sentry_decision_real.launch.py` 对评分器和执行器统一覆盖：

```yaml
referee_data_timeout_ms: 3000
```

该值允许约 1 Hz 的裁判数据存在合理调度抖动，同时仍保留失联保护。任何必需输入
`game_status`、`robot_status` 或 `event_data` 连续 3 秒未刷新时，系统仍会取消当前导航并
进入 `HOLD_SAFE`。如果现场裁判链路有更严格且稳定的刷新周期，可以通过实车参数文件
调小该值，但不建议设为与报文周期完全相同。

### 2.4 动作切换策略

- 普通动作改变：先取消旧导航，确认旧目标终态后发送新目标。
- 取消未确认：保持等待，不发送重叠目标。
- 补给动作：普通进攻、巡逻或一般回防评分变化不打断补给。
- 安全事件：停赛、裁判/决策数据失效、外部取消仍立即打断补给。
- 地图校验：地图缺失、目标越界、未知或占用栅格均不发送目标，报告
  `goal_outside_free_map`。

## 3. 诊断信息

主要话题：

```bash
ros2 topic echo /tactical_decision
ros2 topic echo /tactical_execution
ros2 topic echo /tactical_navigation_status
ros2 topic echo /tactical_goal
```

- `/tactical_decision`：评分器推荐动作和实际首要选择原因。
- `/tactical_execution`：行为树实际动作、`STARTED/RUNNING/PREEMPTED/FAILED/SUCCEEDED`
  状态及原因。
- `/tactical_navigation_status`：等待服务、目标发送/接受、取消和 Nav2 终态。
- `/tactical_goal`：仅用于观察已发送的目标点；实际控制接口是
  `nav2_msgs/action/NavigateToPose`。

导航日志的 `goal_sent` 现在包含 `x;y;yaw` 或完整位姿文本，可以直接核对配置目标。

## 4. 实车检查与验收

启动导航后检查 Action 服务：

```bash
ros2 action list | grep navigate_to_pose
ros2 action info /navigate_to_pose
```

启动决策：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pb2025_sentry_behavior sentry_decision_real.launch.py \
  params_file:=/absolute/path/to/real_decision.yaml \
  team_color:=blue navigation_action_name:=/navigate_to_pose
```

验收时应依次看到：执行动作 `STARTED`、导航 `goal_sent target=...`、导航
`accepted`。动作切换时应先看到旧目标 `cancel_requested/cancel_sent` 和终态，再看到下一代
目标的 `goal_sent`。

暂停任一必需裁判话题不足 3 秒不应急停；连续暂停超过 3 秒应出现对应 `stale_*`，取消
当前导航并进入 `HOLD_SAFE`。

## 5. 常见原因排查

- `navigation_server_unavailable`：10 秒内未发现配置的 Action。检查 Nav2 是否启动、
  `ros2 action list` 的完整路径以及 `navigation_action_name`。
- `goal_outside_free_map`：检查 `map` 是否存在、坐标系是否为 `map`、目标是否位于已知
  自由栅格，以及实车地图和参数文件是否匹配。
- `stale_game_status` / `stale_robot_status` / `stale_event_data`：检查对应话题频率、串口
  驱动和 ROS 时间；不要仅通过无限增大超时掩盖真实断链。
- `navigation_cancel_unconfirmed`：Nav2 在取消门限内没有返回目标终态。检查 Nav2 Action
  状态、控制器是否卡住及回调执行是否正常。在问题解除前系统不会发送新目标。

## 6. 安全影响

修改没有取消任何安全门控，也没有让决策节点直接发布 `/cmd_vel`。裁判数据失联保护从
1 秒调整为 3 秒，以换取对正常 1 Hz 数据抖动的容忍；导航服务等待只影响首次目标发送，
不会绕过地图校验、取消确认或比赛状态门控。
