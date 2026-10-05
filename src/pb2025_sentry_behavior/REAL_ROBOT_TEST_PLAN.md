# 哨兵实车战术导航测试

## 启动与隔离

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pb2025_sentry_behavior sentry_tactical_real_test.launch.py \
  map:=/path/to/RMUC26.yaml gate_enabled:=false rviz:=true
```

该入口使用 `use_sim_time=false`、现有 `rm_navigation_reality_launch.py`、地图与 Nav2 参数。裁判模拟器发布到 `/test_referee/*`；真实视觉保留 `/tracker/target`，模拟器用 `publish_target:=false`。串口裁判输出不参与本轮决策。先用 `gate_enabled:=false` 静态检查话题和目标，再由现场安全员确认后改为 `true`。

速度门控输出位于坐标转换之后、串口输入之前，平移合速度不超过 0.3 m/s，角速度不超过 0.5 rad/s；停止、HOLD、决策/里程计/速度过期及导航取消都会输出零速。`cmd_spin` 被重映射到禁止话题，首轮不启用小陀螺。

启动后应确认每个裁判话题只有模拟器发布者、`/tracker/target` 只有视觉发布者，并确认 `/map`、`map→odom`、`odom→gimbal_yaw`、定位和 Nav2 lifecycle 均正常。启动前入口会检查所有战术目标在地图范围内且处于空闲栅格；掩护目标在行为树执行前仍由地图验证节点复核。

## 场景命令

```bash
ros2 param set /sentry_referee_simulator scenario gather_info
ros2 param set /sentry_referee_simulator scenario cover_teammate
ros2 param set /sentry_referee_simulator scenario attack_outpost
ros2 param set /sentry_referee_simulator scenario resupply_home
ros2 param set /sentry_referee_simulator scenario defend_base
ros2 param set /sentry_referee_simulator scenario defend_fortress
ros2 param set /sentry_referee_simulator scenario capture_point
ros2 param set /sentry_referee_simulator scenario hold_safe
```

现场使用覆盖参数触发边界：

```bash
ros2 param set /sentry_referee_simulator base_hp 5000
ros2 param set /sentry_referee_simulator fortress_state 2
ros2 param set /sentry_referee_simulator paused_topics game_status
ros2 param set /sentry_referee_simulator paused_topics ""
ros2 param set /sentry_referee_simulator scenario game_over
```

`base_hp=5100` 用于普通预警，`base_hp=5000` 用于硬防守；模拟器默认场景是 `hold_safe`。裁判发布器会持续发布 `GAME_OVER`，切回 `gather_info` 后可恢复。

## 验收顺序

1. 门控关闭：检查七动作诊断、场景数据一致性、地图和定位。
2. 门控开启：依次执行侦察、占点、掩护、前哨、补给、基地/堡垒防守；每项记录 Nav2 结果和最终位姿。
3. 普通切换：`gather_info → defend_base → capture_point → hold_safe`，确认旧目标终态后才发新目标。
4. 故障：暂停 `game_status`、停止评分器、切 `game_over`、发送临界 HP；确认只输出零速，恢复数据后能重新执行。
5. 连续演练：人工场景序列通过后再执行整场演练。

通过条件：导航成功且位置误差不超过 0.25 m；无重叠 Nav2 goal；取消请求到目标终态不超过 2 s；显式停止后最终零速命令在 0.2 s 内；裁判/决策断流后在配置超时加 0.2 s 内。车辆实际停车距离单独用现场测量记录。

## 记录

```bash
ros2 bag record -o sentry_real_test \
  /test_referee/game_status /test_referee/robot_status /test_referee/event_data \
  /test_referee/all_robot_hp /test_referee/ground_robot_position /test_referee/rfid_status \
  /tracker/target /tactical_decision /tactical_execution \
  /tactical_navigation_status /test_velocity_gate/status /odometry /cmd_vel_base_real_yaw
```

每项记录：日期、地图/参数版本、起止时间、场景、期望动作、实际动作、目标点、起止位姿、Nav2 结果、取消耗时、软件零速延迟、实际停车距离、异常日志和 rosbag 路径。自动回归结果与现场实测结果分开保存。
