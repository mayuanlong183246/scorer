# 哨兵实车战术导航测试

## 独立启动与接口约定

由导航负责人先启动导航、定位和底盘。决策侧只运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pb2025_sentry_behavior sentry_decision_real.launch.py \
  params_file:=/absolute/path/to/real_decision.yaml \
  navigation_action_name:=/navigate_to_pose team_color:=blue
```

参数文件沿用 `params/sentry_behavior.yaml` 三个节点的结构，必须先按实车地图标定目标点。
入口不读取导航参数、不启动或修改导航节点；仅依赖标准 Nav2 消息和工具库，无需导航源码包。
旧 `sentry_tactical_real_test.launch.py` 为联合测试入口，仍保留导航包依赖，不用于独立部署。

导航提供 `nav2_msgs/action/NavigateToPose`、`map` 坐标系及 `map` 地图话题。
目标在行为树发送前验证，动态掩护目标同样校验；路径可达性和碰撞检测由导航负责。
取消当前目标后必须等待终态，超过 2 秒仅告警并继续阻止新目标，不以超时强行放行。
战术节点不发送零速度，取消后的停车及底盘残留速度处理由导航侧完成。

真实裁判与模拟器二选一，确认 `/referee/*` 不存在重复发布者。模拟器命令仅用于受控联调。
`namespace` 非空时相对输入话题跟随命名空间，Action 可用绝对名称指向导航服务。
以下诊断中的速度、TF 和导航话题按导航负责人提供的实际名称替换。

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

1. 未开赛和关键数据缺失时不导航；检查地图、定位、裁判输入和诊断。
2. 单目标：确认目标内容、UUID、导航结果及实际到达位置；再测试队友掩护动态目标。
3. 主动取消：停赛或安全等待触发取消；同时记录取消响应、目标终态和车辆实际停止。
4. 普通切换：确认旧目标终态后才发新目标；取消拒绝或无终态时必须阻止新目标。
5. 补给保持：血量 180、弹量 50 不提前退出；即使回防评分升高也继续补给，
   到血量 400、弹量 150 才完成；到点 30 秒未完成则失败冷却。热量单独升高不回家。
6. 输入故障：分别暂停裁判关键话题、停止评分器、切换停赛；必须取消导航并停止发送新目标。
7. 外部取消整个行为树，确认服务端继续处理导航取消，客户端不会自动重启任务。
8. 恢复输入和导航后检查调度恢复，再做连续演练。

软件通过条件：无重叠目标；取消未确认不能放行；非法目标不发送；无决策速度发布。
正常导航取消目标终态期望在 2 秒内，超过则记录告警和导航故障。
实际停车延迟、距离及允许误差由现场与导航负责人约定并实测，不能用 Action 响应代替实测。

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

## 本次软件回归记录（2026-10-18）

- 环境：ROS 2 Humble、BehaviorTree.CPP 4.9.0；从仅含决策及五个源码依赖包的独立工作区构建。
- 已确认 `pb2025_nav_bringup` 不可发现，决策独立入口仍正常启动三个决策节点。
- `test_tactical_scorer`：14 项通过；`test_tactical_execution`：31 项通过。
- `test_referee_gate`、`test_tactical_server`、`test_decision_launch` 三组通过。
- 覆盖延迟接受后的取消、取消拒绝时阻止新目标、树停止后的取消回调、
  外部取消不自动重启、补给保持、热量不回家、关键数据过期和目标校验。
- 测试使用 `ROS_LOCALHOST_ONLY=1`、`ROS_DOMAIN_ID=157` 和模拟 Action 服务，未连接实车。
- 本机用户 Python 依赖冲突通过 `PYTHONNOUSERSITE=1` 隔离，未修改系统环境。
- 尚未验证真实导航取消后的底盘停车延迟、距离及实际地图上的到达精度。

完成编译后可运行功能回归（使用独立 ROS 域，避免连接实车）：

```bash
source install/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=157 PYTHONNOUSERSITE=1 \
  colcon test --packages-select pb2025_sentry_behavior \
  --ctest-args -R '^test_(tactical_scorer|tactical_execution|tactical_server|referee_gate|decision_launch)$' -j1
colcon test-result --verbose
```
