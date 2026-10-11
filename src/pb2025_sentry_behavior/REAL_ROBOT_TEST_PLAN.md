# 哨兵实车战术导航测试

## 启动与隔离

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
# 使用真实裁判数据
ros2 launch pb2025_sentry_behavior sentry_tactical_real_test.launch.py \
  map:=/path/to/RMUC26.yaml rviz:=true

# 使用独立裁判模拟器（与真实裁判二选一）
ros2 launch pb2025_sentry_behavior sentry_referee_simulator.launch.py scenario:=match
ros2 launch rm_serial_driver serial_driver.launch.py publish_referee_topics:=false
ros2 launch pb2025_sentry_behavior sentry_tactical_real_test.launch.py \
  map:=/path/to/RMUC26.yaml rviz:=true
```

该入口使用 `use_sim_time=false`、现有 `rm_navigation_reality_launch.py`、地图与 Nav2 参数。评分器统一订阅标准 `/referee/*` 话题；模拟器联调时串口驱动必须使用 `publish_referee_topics:=false`，此时速度发送仍然启用。模拟器发现任意 `/referee/*` 重复发布者会立即报错退出。模拟器默认 `scenario:=match`，会自动经历准备、倒计时并进入 `RUNNING`；需要验证安全门时使用 `scenario:=hold_safe`。真实视觉保留 `/tracker/target`。

正式速度链路为 Nav2 → velocity_smoother → fake_vel_transform → `/cmd_vel_base_real_yaw` → `rm_serial_driver`。fake_vel_transform 不依赖 local_plan，以固定频率发布；命令超过 300 ms 或 odometry 超过 500 ms 未更新时持续发布零速。测试 gate 仅在独立回归测试中使用，不在此正式入口启动。

启动后应确认每个裁判话题只有模拟器发布者、`/tracker/target` 只有视觉发布者，并确认 `/map`、`map→odom`、`odom→gimbal_yaw_fake`、定位和 Nav2 lifecycle 均正常。启动前入口会检查所有战术目标在地图范围内且处于空闲栅格；掩护目标在行为树执行前仍由地图验证节点复核。

## MPPI 诊断

不增加常驻诊断节点，也不修改 MPPI 参数。复现 `Optimizer fail` 前启动一次按需记录，
切换目标后保留机器人位姿、目标、全局路径、局部代价地图、footprint、terrain、TF 和
三级速度链原始证据：

```bash
ros2 bag record -o mppi_failure \
  /tf /tf_static /tactical_goal /plan \
  /local_costmap/costmap_raw /local_costmap/published_footprint \
  /terrain_map /odometry /cmd_vel_controller /cmd_vel_nav2_result \
  /cmd_vel_base_real_yaw
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom gimbal_yaw_fake
ros2 topic hz /cmd_vel_controller
ros2 topic hz /cmd_vel_nav2_result
ros2 topic hz /cmd_vel_base_real_yaw
```

在 RViz 同时显示 local costmap、published footprint、plan 与 terrain：先检查当前
footprint 覆盖的最高代价，再沿路径前方一个 MPPI 预测时域检查 lethal、inscribed、
unknown 和越界栅格。当前 footprint 碰撞先查雷达自体裁剪、TF 和 footprint；路径首段
碰撞才调整导航点或增加中间点；unknown/越界较多时修局部 costmap 的更新、尺寸或
坐标系；只有环境与赛场地图不匹配时更换现场地图。不得通过删除 critic、缩小
footprint、放宽碰撞阈值或关闭避障来通过测试。

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
2. 依次执行侦察、占点、掩护、前哨、补给、基地/堡垒防守；每项记录 Nav2 结果和最终位姿。
3. 普通切换：`gather_info → defend_base → capture_point → hold_safe`，确认旧目标终态后才发新目标。
4. 故障：暂停 `game_status`、停止评分器、切 `game_over`、发送临界 HP；确认只输出零速，恢复数据后能重新执行。
5. 连续演练：人工场景序列通过后再执行整场演练。

通过条件：导航成功且位置误差不超过 0.25 m；无重叠 Nav2 goal；取消请求到目标终态不超过 2 s；显式停止后最终零速命令在 0.2 s 内；裁判/决策断流后在配置超时加 0.2 s 内。车辆实际停车距离单独用现场测量记录。

## 记录

```bash
ros2 bag record -o sentry_real_test \
  /referee/game_status /referee/robot_status /referee/event_data \
  /referee/all_robot_hp /referee/ground_robot_position /referee/rfid_status \
  /tracker/target /tactical_decision /tactical_execution \
  /tactical_goal /tactical_navigation_status /tf /tf_static \
  /plan /local_costmap/costmap_raw /local_costmap/published_footprint \
  /terrain_map /odometry /cmd_vel_controller /cmd_vel_nav2_result \
  /cmd_vel_base_real_yaw
```

每项记录：日期、地图/参数版本、起止时间、场景、期望动作、实际动作、目标点、起止位姿、Nav2 结果、取消耗时、软件零速延迟、实际停车距离、异常日志和 rosbag 路径。自动回归结果与现场实测结果分开保存。
