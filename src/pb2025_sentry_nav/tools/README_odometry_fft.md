# 里程计 FFT 可视化工具

脚本位置: `src/pb2025_sentry_nav/tools/odometry_fft_visualizer.py`

## 功能
- 加载 CSV 里程计数据
- 可视化时域曲线
- 使用 `numpy.fft.rfft` 进行单边频谱分析
- 自动估计采样频率（有时间列时）
- 支持去直流、窗函数（none/hann/hamming）、频率上限显示
- 支持实时订阅 ROS2 `nav_msgs/msg/Odometry` 话题
- 支持在线滚动窗口频谱分析

## CSV 建议格式
- 至少包含一个数值信号列，例如 `vx`/`wz`/`yaw`
- 时间列可选，推荐列名: `time`、`timestamp`、`t`

如果没有时间列，请在界面输入实际采样频率（Hz）。

## 运行
```bash
python3 src/pb2025_sentry_nav/tools/odometry_fft_visualizer.py
```

## 实时模式使用
1. 先 source ROS2 环境和工作区：
```bash
source /opt/ros/<distro>/setup.bash
source install/setup.bash
```

2. 运行脚本后，在界面中选择：
- 数据源：`ROS2 实时`
- ROS2 话题：例如 `/odometry`
- 实时信号：例如 `linear_x` 或 `angular_z`
- 滚动窗口秒数：例如 `10`

3. 点击 `开始订阅`，界面会自动滚动刷新时域与频谱。

说明：
- 若你没有使用 ROS 时间戳，脚本会自动回退到系统时间。
- 频谱主频会随着窗口内最新数据实时更新。

## 依赖
- Python 3
- numpy
- matplotlib
- tkinter（通常随系统 Python 提供）
- rclpy（实时订阅模式需要）
- nav_msgs（实时订阅模式需要）
