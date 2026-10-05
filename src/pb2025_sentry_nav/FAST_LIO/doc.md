

**# 角色设定**
你是一位顶级自动驾驶 SLAM 算法工程师与 ROS2/C++ 架构师，对 FAST-LIO2 源码结构、多线程数据同步、流形状态机（IKFoM）以及 MPPI 路径规划器的高频状态需求有极深的工业级理解。

**# 任务背景**
我目前在 ROS2 环境下运行 FAST-LIO2 + Livox MID360 双雷达用于全向底盘控制。由于 MPPI 控制器需要极高频且低延迟的里程计（50-60Hz），而 FAST-LIO2 原生的雷达里程计只有 20Hz 且存在 20-40ms 的解算延迟，这导致底盘控制发生滞后。
同时，我之前的架构要求**里程计必须是重力绝对对齐的绝对水平位姿**，并且**不发布任何 TF，只发布 `/odom` 话题**。

**# 核心任务**
请帮我重写 `laserMapping.cpp` 中的相关逻辑，实现一个**“异步 IMU 高频前向预测与重积分（Re-integration）机制”**。
跳过雷达反向传播残差，直接在 IMU 回调线程中利用 IMU 运动学积分输出 50Hz 的平滑里程计，并在每次雷达优化完成后执行重积分以消除漂移。

**# 详细实施规范（请严格按照以下 5 个模块编写 C++ 代码）：**

#### 模块 1：全局变量与独立缓冲区的定义
在 `laserMapping.cpp` 顶部或合适的全局区域，定义以下变量用于高频预测机制（请注意线程安全）：
1. `std::mutex predict_mutex;`：用于保护预测状态和缓冲区的互斥锁。
2. `std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_predict_buffer;`：**独立的 IMU 预测双端队列**（为了不污染 FAST-LIO2 原本的 `imu_buffer`）。
3. `state_ikfom predict_state;`：用于保存当前前向预测的最新状态。
4. `bool has_first_lidar_state = false;`：标志位，表示是否已经接收到第一次雷达优化后的基准状态。
5. 定义上一次雷达优化完成时的状态 `state_ikfom last_optimized_state;` 和其对应的时间戳 `double last_optimized_time;`。
6. 定义一个变量用于降采样发布：`int imu_pub_count = 0;`。

#### 模块 2：修改 IMU 回调函数 `imu_cbk`（核心预测逻辑）
当新的 IMU 数据到来时（MID360 为 200Hz），首先将原始数据 push 到原版 `imu_buffer` 中。然后执行以下“预测与发布”逻辑：
1. **压入独立队列**：将数据压入 `imu_predict_buffer`。
2. **等待初始化**：如果 `!has_first_lidar_state`，直接 return，不进行预测。
3. **高频前向积分**：
   - 提取上一次预测的时间 $t_{k-1}$ 和当前 IMU 时间 $t_k$，计算 $\Delta t$。
   - 使用 `predict_state` 中的 `bg` (陀螺仪 bias) 和 `ba` (加速度计 bias) 去除当前 IMU 测量值的零偏。
   - 利用标准的 IMU 运动学方程（中值积分或欧拉积分均可，注意重力 $g$ 的抵消），更新 `predict_state.pos`, `predict_state.vel`, `predict_state.rot`。
4. **低通滤波（专为 MPPI 设计）**：
   - 对积分计算出的线速度 (`vel`) 和角速度 (`omega`) 应用一阶 RC 低通滤波（例如 $\alpha = 0.2$），滤除底盘的高频机械震动，防止 MPPI 接收到毛刺。
5. **降采样发布（50Hz）**：
   - 累加 `imu_pub_count`。当收到 4 个 IMU 数据（200Hz / 4 = 50Hz）时，触发一次 Odom 发布。
   - **重力正骨正向传播**：复用我们之前的重力对齐逻辑（通过 `state_point.grav` 算出的 `T_align`），对预测出来的 `predict_state.pos` 和 `predict_state.rot` 进行矫正，使其变为绝对水平。
   - 将矫正后的 Pose 和滤波后的 Twist 填充进 `nav_msgs::msg::Odometry` 并发布到 `/Odometry_high_freq`。**绝对不要在此处发布 TF**。

#### 模块 3：修改 LiDAR 优化完成后的回调（重积分机制）
在主循环 `while (rclcpp::ok())` 的末尾（即一次 IKFoM 优化完毕，拿到了新的最优状态 `state_point` 后）：
1. 获取当前主线程的锁 `std::lock_guard<std::mutex> lock(predict_mutex);`。
2. 将当前最新的 `state_point`（含位姿、速度、零偏）以及这帧雷达的时间戳 $t_{lidar}$ 覆盖 `last_optimized_state` 和 `last_optimized_time`。标记 `has_first_lidar_state = true`。
3. **清理过期数据**：遍历 `imu_predict_buffer`，将时间戳小于 $t_{lidar}$ 的历史 IMU 数据全部 `pop_front` 丢弃。
4. **执行重积分（Re-integration 滑动窗口）**：
   - 将 `predict_state` 重置为刚拿到的最优状态 `last_optimized_state`。
   - 遍历 `imu_predict_buffer` 中**剩余的**所有 IMU 数据（这些是发生在 $t_{lidar}$ 之后，但在这 30ms 解算延迟内产生的新 IMU 数据）。
   - 以此最优状态为起点，使用更新后的 Bias 重新进行一遍快速积分，一直积分到 `imu_predict_buffer` 的最后一条数据。
   - 这样就完美消除了 LiDAR 解算延迟带来的时间倒流问题，确保最新的 `predict_state` 是平滑且纠偏过的。

#### 模块 4：代码输出要求
1. 请给出 `laserMapping.cpp` 中需要增加的**全局变量与数据结构定义**。
2. 请给出完整的 `imu_cbk` 函数修改代码。
3. 请给出主线程中 `IKFoM` 优化结束后（约 `publish_odometry` 附近）的**重积分代码片段**。
4. 补充必要的中文注释，着重解释时间戳同步和低通滤波部分的逻辑。确保能够直接在 ROS2 环境编译通过（使用 `tf2::` 进行四元数计算和重力对齐）。
5. 请根据当前代码进行修改，尽量不要删除原有功能

