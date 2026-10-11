# fake_vel_transform

本功能包启动时，Fake Velocity Transform 会创建一个 `fake_robot_base_frame` 的坐标系，其 x, y, z 与 `robot_base_frame` 的坐标系一致，但 yaw 固定指向正前方，然后将生成的该坐标系发布到 tf2。同时，Fake Velocity Transform 会订阅 `input_cmd_vel_topic` 话题，将接收到的速度指令转换到 `robot_base_frame` 坐标系下，并发布到 `output_cmd_vel_topic` 话题。

主要目的是用于适配 NAV2 局部路径规划器，当速度参考坐标系 `robot_base_frame` 变化剧烈时，如云台处于自旋扫描时，NAV2 局部路径规划器会将机器人的方向视为与当前路径规划方向一致，导致机器人无法正常运动。而使用 `fake_robot_base_frame` 可以规避这个问题，实现较稳定的轨迹跟踪效果。

节点以固定频率输出速度，不依赖 `local_plan`。只有速度命令和里程计都处于有效期内时才发布转换后的速度；命令超过 300 ms 或 odometry 超过 500 ms 未更新时持续发布零速。
Related issue: [Switch from Twist to TwistStamped for cmd_vel #1594](https://github.com/ros-navigation/navigation2/issues/1594)

## Published Topics

* `tf` (`tf2_msgs/msg/TFMessage`) - 与机器人可移动关节相对应的变换
* `output_cmd_vel_topic` (`geometry_msgs/msg/Twist`) - 转换后的速度指令

## Subscribed Topics

* `input_cmd_vel_topic` (`geometry_msgs/msg/Twist`) - 机器人的速度指令
* `odom_topic` (`nav_msgs/msg/Odometry`) - 里程计数据
* `cmd_spin_topic` (`example_interfaces/msg/Float32`) - 控制底盘固定旋转速度，将会叠加到 `output_cmd_vel_topic` 中

## Parameters

* `odom_topic` (`string`, default: "odom") - 里程计话题。里程计的 frame_id 与 `robot_base_frame` 参数保持一致
* `robot_base_frame` (`string`, default: "gimbal_link") - 速度参考坐标系
* `fake_robot_base_frame` (`string`, default: "gimbal_link_fake") - 伪速度参考坐标系
* `cmd_spin_topic` (`string`, default: "cmd_spin") - 控制底盘固定旋转速度的话题
* `input_cmd_vel_topic` (`string`, default: "cmd_vel_nav2_result") - 输入速度指令的话题
* `output_cmd_vel_topic` (`string`, default: "cmd_vel_base_real_yaw") - 输出速度指令的话题。将原本基于 `fake_robot_base_frame` 的速度变换到 `robot_base_frame` 后发布
* `init_spin_speed` (`double`, default: 0.0) - 若没有接收 `cmd_spin_topic`，则使用该值作为固定旋转速度
* `command_timeout_ms` (`double`, default: 300.0) - 速度命令有效期，超时输出零速
* `odom_timeout_ms` (`double`, default: 500.0) - 里程计有效期，未收到或超时时输出零速
* `publish_rate` (`double`, default: 20.0) - 速度输出频率，Hz
