#!/usr/bin/env python3
"""里程计数据可视化与 FFT 频谱分析工具。

功能:
- 加载 CSV 里程计数据
- 可选择任意数值列进行时域分析
- 使用 numpy.fft.rfft 进行单边频谱分析
- 支持窗函数、去直流、频率范围限制
"""

from __future__ import annotations

import csv
import math
import os
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from tkinter import filedialog, messagebox, ttk
from typing import Any, Dict, List, Optional, Tuple

import matplotlib
import numpy as np

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

try:
    import rclpy
    from nav_msgs.msg import Odometry
except ImportError:
    rclpy = None
    Odometry = None


TIME_CANDIDATES = [
    "time",
    "timestamp",
    "t",
    "stamp",
    "sec",
    "time_s",
    "seconds",
]

SIGNAL_CANDIDATES = [
    "vx",
    "vy",
    "vz",
    "wx",
    "wy",
    "wz",
    "linear_x",
    "linear_y",
    "linear_z",
    "angular_x",
    "angular_y",
    "angular_z",
    "pose_x",
    "pose_y",
    "pose_z",
    "yaw",
]


@dataclass
class OdomDataset:
    columns: Dict[str, np.ndarray]
    time_key: Optional[str]


class OdomFftApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("里程计 FFT 频谱分析")
        self.root.geometry("1300x820")

        self.dataset: Optional[OdomDataset] = None
        self.current_file: Optional[str] = None

        self.ros_ready = rclpy is not None and Odometry is not None
        self.ros_running = False
        self.ros_init_by_app = False
        self.ros_node = None
        self.ros_executor = None
        self.ros_spin_thread: Optional[threading.Thread] = None
        self.ros_lock = threading.Lock()
        self.ros_window_sec = 10.0
        self.ros_topic = "/odometry"
        self.ros_time_buffer: deque[float] = deque()
        self.ros_signal_buffers: Dict[str, deque[float]] = {}

        self.ros_extractors = {
            "linear_x": lambda m: float(m.twist.twist.linear.x),
            "linear_y": lambda m: float(m.twist.twist.linear.y),
            "linear_z": lambda m: float(m.twist.twist.linear.z),
            "angular_x": lambda m: float(m.twist.twist.angular.x),
            "angular_y": lambda m: float(m.twist.twist.angular.y),
            "angular_z": lambda m: float(m.twist.twist.angular.z),
            "pose_x": lambda m: float(m.pose.pose.position.x),
            "pose_y": lambda m: float(m.pose.pose.position.y),
            "pose_z": lambda m: float(m.pose.pose.position.z),
        }
        for key in self.ros_extractors:
            self.ros_signal_buffers[key] = deque()

        self._build_ui()
        self._schedule_periodic_refresh()

    def _build_ui(self) -> None:
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=1)

        control = ttk.Frame(self.root, padding=10)
        control.grid(row=0, column=0, sticky="ns")
        control.columnconfigure(0, weight=1)

        plot_area = ttk.Frame(self.root, padding=8)
        plot_area.grid(row=0, column=1, sticky="nsew")
        plot_area.rowconfigure(0, weight=1)
        plot_area.columnconfigure(0, weight=1)

        row = 0
        ttk.Label(control, text="数据源").grid(row=row, column=0, sticky="w")
        row += 1
        self.source_var = tk.StringVar(value="csv")
        source_frame = ttk.Frame(control)
        source_frame.grid(row=row, column=0, sticky="ew")
        ttk.Radiobutton(source_frame, text="CSV 离线", value="csv", variable=self.source_var).pack(side="left")
        ttk.Radiobutton(source_frame, text="ROS2 实时", value="ros", variable=self.source_var).pack(side="left")
        row += 1

        ttk.Separator(control, orient="horizontal").grid(row=row, column=0, sticky="ew", pady=(8, 8))
        row += 1

        ttk.Button(control, text="加载 CSV", command=self._load_csv).grid(row=row, column=0, sticky="ew")
        row += 1

        self.file_label = ttk.Label(control, text="未加载文件", wraplength=280)
        self.file_label.grid(row=row, column=0, sticky="w", pady=(8, 12))
        row += 1

        ttk.Label(control, text="信号列").grid(row=row, column=0, sticky="w")
        row += 1
        self.signal_box = ttk.Combobox(control, state="readonly", width=30)
        self.signal_box.grid(row=row, column=0, sticky="ew")
        row += 1

        ttk.Label(control, text="时间列（可选）").grid(row=row, column=0, sticky="w", pady=(10, 0))
        row += 1
        self.time_box = ttk.Combobox(control, state="readonly", width=30)
        self.time_box.grid(row=row, column=0, sticky="ew")
        row += 1

        ttk.Label(control, text="采样频率 Hz（无时间列时使用）").grid(row=row, column=0, sticky="w", pady=(10, 0))
        row += 1
        self.sample_rate_var = tk.StringVar(value="100.0")
        ttk.Entry(control, textvariable=self.sample_rate_var).grid(row=row, column=0, sticky="ew")
        row += 1

        ttk.Label(control, text="窗函数").grid(row=row, column=0, sticky="w", pady=(10, 0))
        row += 1
        self.window_var = tk.StringVar(value="hann")
        ttk.Combobox(
            control,
            state="readonly",
            textvariable=self.window_var,
            values=["none", "hann", "hamming"],
        ).grid(row=row, column=0, sticky="ew")
        row += 1

        self.remove_dc_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(control, text="去除直流分量（减去均值）", variable=self.remove_dc_var).grid(
            row=row, column=0, sticky="w", pady=(10, 0)
        )
        row += 1

        ttk.Label(control, text="频谱最大显示频率 Hz（0=自动）").grid(row=row, column=0, sticky="w", pady=(10, 0))
        row += 1
        self.max_freq_var = tk.StringVar(value="50")
        ttk.Entry(control, textvariable=self.max_freq_var).grid(row=row, column=0, sticky="ew")
        row += 1

        ttk.Label(control, text="自动上限模式（仅当上限=0）").grid(row=row, column=0, sticky="w", pady=(8, 0))
        row += 1
        self.max_freq_mode_var = tk.StringVar(value="fs/2")
        ttk.Combobox(
            control,
            state="readonly",
            textvariable=self.max_freq_mode_var,
            values=["fs/2", "adaptive"],
        ).grid(row=row, column=0, sticky="ew")
        row += 1

        ttk.Separator(control, orient="horizontal").grid(row=row, column=0, sticky="ew", pady=(10, 8))
        row += 1

        ttk.Label(control, text="ROS2 话题（nav_msgs/msg/Odometry）").grid(row=row, column=0, sticky="w")
        row += 1
        self.ros_topic_var = tk.StringVar(value="/odometry")
        ttk.Entry(control, textvariable=self.ros_topic_var).grid(row=row, column=0, sticky="ew")
        row += 1

        ttk.Label(control, text="实时信号").grid(row=row, column=0, sticky="w", pady=(10, 0))
        row += 1
        self.ros_signal_var = tk.StringVar(value="linear_x")
        self.ros_signal_box = ttk.Combobox(
            control,
            state="readonly",
            textvariable=self.ros_signal_var,
            values=list(self.ros_extractors.keys()),
            width=30,
        )
        self.ros_signal_box.grid(row=row, column=0, sticky="ew")
        row += 1

        ttk.Label(control, text="滚动窗口秒数").grid(row=row, column=0, sticky="w", pady=(10, 0))
        row += 1
        self.ros_window_var = tk.StringVar(value="10.0")
        ttk.Entry(control, textvariable=self.ros_window_var).grid(row=row, column=0, sticky="ew")
        row += 1

        ros_btn_frame = ttk.Frame(control)
        ros_btn_frame.grid(row=row, column=0, sticky="ew", pady=(8, 0))
        ros_btn_frame.columnconfigure(0, weight=1)
        ros_btn_frame.columnconfigure(1, weight=1)
        self.start_ros_btn = ttk.Button(ros_btn_frame, text="开始订阅", command=self._start_ros_subscription)
        self.start_ros_btn.grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self.stop_ros_btn = ttk.Button(ros_btn_frame, text="停止订阅", command=self._stop_ros_subscription)
        self.stop_ros_btn.grid(row=0, column=1, sticky="ew", padx=(4, 0))
        row += 1

        self.ros_status_label = ttk.Label(control, text="ROS 状态: 未启动")
        self.ros_status_label.grid(row=row, column=0, sticky="w", pady=(6, 0))
        row += 1

        if not self.ros_ready:
            self.start_ros_btn.state(["disabled"])
            self.stop_ros_btn.state(["disabled"])
            self.ros_status_label.config(text="ROS 状态: 不可用（缺少 rclpy 或 nav_msgs）")

        ttk.Button(control, text="更新图像", command=self._update_plot).grid(row=row, column=0, sticky="ew", pady=(14, 0))
        row += 1

        self.info_text = tk.Text(control, width=36, height=16)
        self.info_text.grid(row=row, column=0, sticky="nsew", pady=(10, 0))
        control.rowconfigure(row, weight=1)

        self.figure = Figure(figsize=(9, 6), dpi=100)
        self.ax_time = self.figure.add_subplot(211)
        self.ax_fft = self.figure.add_subplot(212)
        self.figure.tight_layout(pad=2.0)

        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_area)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        toolbar_frame = ttk.Frame(plot_area)
        toolbar_frame.grid(row=1, column=0, sticky="ew")
        self.toolbar = NavigationToolbar2Tk(self.canvas, toolbar_frame)
        self.toolbar.update()

    def _load_csv(self) -> None:
        file_path = filedialog.askopenfilename(
            title="选择里程计 CSV 文件",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not file_path:
            return

        try:
            dataset = self._read_csv_numeric(file_path)
        except Exception as exc:  # pylint: disable=broad-except
            messagebox.showerror("加载失败", f"无法解析 CSV: {exc}")
            return

        self.dataset = dataset
        self.current_file = file_path
        self.file_label.config(text=os.path.basename(file_path))

        numeric_cols = list(dataset.columns.keys())
        self.signal_box["values"] = numeric_cols
        preferred_signal = self._find_first_match(numeric_cols, SIGNAL_CANDIDATES) or (numeric_cols[0] if numeric_cols else "")
        self.signal_box.set(preferred_signal)

        time_options = ["<none>"] + numeric_cols
        self.time_box["values"] = time_options
        preferred_time = dataset.time_key if dataset.time_key else "<none>"
        self.time_box.set(preferred_time)

        self.source_var.set("csv")

        self._update_plot()

    def _read_csv_numeric(self, file_path: str) -> OdomDataset:
        with open(file_path, "r", encoding="utf-8", newline="") as f:
            reader = csv.DictReader(f)
            if reader.fieldnames is None:
                raise ValueError("CSV 没有表头")

            data_map: Dict[str, List[float]] = {name: [] for name in reader.fieldnames}

            for row in reader:
                for key in reader.fieldnames:
                    val = (row.get(key, "") or "").strip()
                    if val == "":
                        data_map[key].append(np.nan)
                        continue
                    try:
                        data_map[key].append(float(val))
                    except ValueError:
                        data_map[key].append(np.nan)

        numeric_columns: Dict[str, np.ndarray] = {}
        for key, values in data_map.items():
            arr = np.array(values, dtype=float)
            if arr.size == 0:
                continue
            finite_ratio = np.isfinite(arr).sum() / arr.size
            if finite_ratio >= 0.8:
                numeric_columns[key] = arr

        if not numeric_columns:
            raise ValueError("未找到可用数值列（至少 80% 为数值）")

        time_key = self._find_first_match(list(numeric_columns.keys()), TIME_CANDIDATES)
        return OdomDataset(columns=numeric_columns, time_key=time_key)

    @staticmethod
    def _find_first_match(items: List[str], candidates: List[str]) -> Optional[str]:
        lowered = {x.lower(): x for x in items}
        for name in candidates:
            if name in lowered:
                return lowered[name]

        # 次优匹配: 包含关键词
        for cand in candidates:
            for item in items:
                if cand in item.lower():
                    return item
        return None

    def _resolve_time_and_signal(self) -> Tuple[np.ndarray, np.ndarray, float, str]:
        if self.source_var.get().strip() == "ros":
            return self._resolve_ros_time_and_signal()

        if self.dataset is None:
            raise ValueError("请先加载 CSV")

        signal_name = self.signal_box.get().strip()
        if signal_name not in self.dataset.columns:
            raise ValueError("请选择有效的信号列")

        y_all = self.dataset.columns[signal_name]
        time_selected = self.time_box.get().strip()

        if time_selected and time_selected != "<none>" and time_selected in self.dataset.columns:
            t_all = self.dataset.columns[time_selected]
            valid = np.isfinite(t_all) & np.isfinite(y_all)
            t = t_all[valid]
            y = y_all[valid]
            if t.size < 8:
                raise ValueError("有效数据点太少，至少需要 8 个点")

            t = t - t[0]
            dt = np.diff(t)
            dt = dt[np.isfinite(dt) & (dt > 0)]
            if dt.size == 0:
                raise ValueError("时间列无有效递增间隔")

            fs = 1.0 / float(np.median(dt))
            return t, y, fs, signal_name

        valid = np.isfinite(y_all)
        y = y_all[valid]
        if y.size < 8:
            raise ValueError("有效数据点太少，至少需要 8 个点")

        try:
            fs = float(self.sample_rate_var.get().strip())
        except ValueError as exc:
            raise ValueError("采样频率必须是数字") from exc

        if not math.isfinite(fs) or fs <= 0:
            raise ValueError("采样频率必须大于 0")

        t = np.arange(y.size, dtype=float) / fs
        return t, y, fs, signal_name

    def _resolve_ros_time_and_signal(self) -> Tuple[np.ndarray, np.ndarray, float, str]:
        if not self.ros_running:
            raise ValueError("ROS2 订阅未启动")

        signal_name = self.ros_signal_var.get().strip()
        if signal_name not in self.ros_signal_buffers:
            raise ValueError("请选择有效的实时信号")

        with self.ros_lock:
            t = np.array(self.ros_time_buffer, dtype=float)
            y = np.array(self.ros_signal_buffers[signal_name], dtype=float)

        if t.size < 8 or y.size < 8:
            raise ValueError("实时数据不足，至少需要 8 个点")

        valid = np.isfinite(t) & np.isfinite(y)
        t = t[valid]
        y = y[valid]
        if t.size < 8:
            raise ValueError("实时有效数据不足，至少需要 8 个点")

        order = np.argsort(t)
        t = t[order]
        y = y[order]

        dt = np.diff(t)
        dt = dt[np.isfinite(dt) & (dt > 0)]
        if dt.size == 0:
            raise ValueError("实时时间戳无有效递增间隔")

        fs = 1.0 / float(np.median(dt))
        t = t - t[0]
        return t, y, fs, signal_name

    def _apply_window(self, data: np.ndarray) -> np.ndarray:
        win_name = self.window_var.get().strip().lower()
        n = data.size
        if win_name == "hann":
            return data * np.hanning(n)
        if win_name == "hamming":
            return data * np.hamming(n)
        return data

    def _update_plot(self) -> None:
        try:
            t, y, fs, signal_name = self._resolve_time_and_signal()
        except Exception as exc:  # pylint: disable=broad-except
            self._write_info(f"无法更新图像: {exc}")
            return

        y_proc = y.copy()
        if self.remove_dc_var.get():
            y_proc = y_proc - np.mean(y_proc)

        y_proc = self._apply_window(y_proc)

        n = y_proc.size
        freqs = np.fft.rfftfreq(n, d=1.0 / fs)
        fft_vals = np.fft.rfft(y_proc)
        amplitude = 2.0 * np.abs(fft_vals) / n
        if amplitude.size > 0:
            amplitude[0] *= 0.5

        if freqs.size > 1:
            peak_idx = int(np.argmax(amplitude[1:]) + 1)
        else:
            peak_idx = 0
        peak_freq = float(freqs[peak_idx]) if freqs.size else 0.0
        peak_amp = float(amplitude[peak_idx]) if amplitude.size else 0.0

        self.ax_time.clear()
        self.ax_fft.clear()

        self.ax_time.plot(t, y, color="#1f77b4", linewidth=1.1)
        self.ax_time.set_title(f"时域信号: {signal_name}")
        self.ax_time.set_xlabel("Time [s]")
        self.ax_time.set_ylabel(signal_name)
        self.ax_time.grid(True, alpha=0.3)

        self.ax_fft.plot(freqs, amplitude, color="#d62728", linewidth=1.1)
        self.ax_fft.scatter([peak_freq], [peak_amp], color="#2ca02c", s=28, zorder=4)
        self.ax_fft.set_title("FFT 单边幅值谱")
        self.ax_fft.set_xlabel("Frequency [Hz]")
        self.ax_fft.set_ylabel("Amplitude")
        self.ax_fft.grid(True, alpha=0.3)

        try:
            max_freq = float(self.max_freq_var.get().strip())
        except ValueError:
            max_freq = 0.0
        if max_freq > 0:
            self.ax_fft.set_xlim(0, max_freq)
        else:
            auto_max = self._auto_max_freq(freqs, amplitude, fs)
            if auto_max > 0:
                self.ax_fft.set_xlim(0, auto_max)

        self.figure.tight_layout(pad=2.0)
        self.canvas.draw()

        summary = [
            f"数据源: {'ROS2 实时' if self.source_var.get().strip() == 'ros' else 'CSV 离线'}",
            f"文件: {self.current_file or '-'}",
            f"信号列: {signal_name}",
            f"样本数: {n}",
            f"采样频率: {fs:.3f} Hz",
            f"频率分辨率: {fs / n:.5f} Hz",
            f"主频: {peak_freq:.5f} Hz",
            f"主频幅值: {peak_amp:.6f}",
            f"窗函数: {self.window_var.get()}",
            f"去直流: {'是' if self.remove_dc_var.get() else '否'}",
            f"频谱上限: {self._format_max_freq_label(max_freq, fs)}",
        ]
        self._write_info("\n".join(summary))

    def _auto_max_freq(self, freqs: np.ndarray, amplitude: np.ndarray, fs: float) -> float:
        mode = self.max_freq_mode_var.get().strip().lower()
        if mode == "adaptive":
            return self._adaptive_max_freq(freqs, amplitude, fs)
        return 0.5 * fs

    def _adaptive_max_freq(self, freqs: np.ndarray, amplitude: np.ndarray, fs: float) -> float:
        if freqs.size == 0 or amplitude.size == 0:
            return 0.0
        peak_amp = float(np.max(amplitude))
        if not math.isfinite(peak_amp) or peak_amp <= 0:
            return 0.5 * fs
        threshold = peak_amp * 0.05
        valid = np.where(amplitude >= threshold)[0]
        if valid.size == 0:
            return 0.5 * fs
        idx = int(valid.max())
        return float(freqs[idx])

    def _format_max_freq_label(self, max_freq: float, fs: float) -> str:
        if max_freq > 0:
            return f"{max_freq:.3f} Hz"
        mode = self.max_freq_mode_var.get().strip()
        if mode == "adaptive":
            return "自适应"
        return f"fs/2 = {0.5 * fs:.3f} Hz"

    def _write_info(self, text: str) -> None:
        self.info_text.delete("1.0", tk.END)
        self.info_text.insert("1.0", text)

    def _parse_ros_window_sec(self) -> float:
        try:
            window_sec = float(self.ros_window_var.get().strip())
        except ValueError as exc:
            raise ValueError("滚动窗口秒数必须是数字") from exc

        if not math.isfinite(window_sec) or window_sec <= 0:
            raise ValueError("滚动窗口秒数必须大于 0")
        return window_sec

    def _start_ros_subscription(self) -> None:
        if not self.ros_ready:
            messagebox.showerror("错误", "ROS2 Python 依赖不可用，请先安装并 source ROS2 环境")
            return
        if self.ros_running:
            messagebox.showinfo("提示", "ROS2 订阅已在运行")
            return

        try:
            self.ros_window_sec = self._parse_ros_window_sec()
        except ValueError as exc:
            messagebox.showerror("参数错误", str(exc))
            return

        topic = self.ros_topic_var.get().strip()
        if not topic:
            messagebox.showerror("参数错误", "ROS2 话题不能为空")
            return

        self.ros_topic = topic
        self.source_var.set("ros")

        with self.ros_lock:
            self.ros_time_buffer.clear()
            for key in self.ros_signal_buffers:
                self.ros_signal_buffers[key].clear()

        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                self.ros_init_by_app = True
            else:
                self.ros_init_by_app = False

            self.ros_node = rclpy.create_node("odom_fft_visualizer")
            self.ros_node.create_subscription(Odometry, self.ros_topic, self._odom_callback, 50)
            self.ros_executor = rclpy.executors.SingleThreadedExecutor()
            self.ros_executor.add_node(self.ros_node)
            self.ros_running = True
            self.ros_spin_thread = threading.Thread(target=self._ros_spin_loop, daemon=True)
            self.ros_spin_thread.start()
            self.ros_status_label.config(text=f"ROS 状态: 运行中 ({self.ros_topic})")
        except Exception as exc:  # pylint: disable=broad-except
            self.ros_running = False
            self.ros_status_label.config(text="ROS 状态: 启动失败")
            messagebox.showerror("启动失败", f"无法启动 ROS2 订阅: {exc}")

    def _stop_ros_subscription(self) -> None:
        if not self.ros_running and self.ros_node is None:
            self.ros_status_label.config(text="ROS 状态: 未启动")
            return

        self.ros_running = False
        if self.ros_spin_thread is not None:
            self.ros_spin_thread.join(timeout=1.0)
            self.ros_spin_thread = None

        try:
            if self.ros_executor is not None and self.ros_node is not None:
                self.ros_executor.remove_node(self.ros_node)
            if self.ros_node is not None:
                self.ros_node.destroy_node()
        except Exception:
            pass

        self.ros_executor = None
        self.ros_node = None

        if self.ros_init_by_app:
            try:
                rclpy.shutdown()
            except Exception:
                pass
            self.ros_init_by_app = False

        self.ros_status_label.config(text="ROS 状态: 已停止")

    def _ros_spin_loop(self) -> None:
        while self.ros_running and self.ros_executor is not None and rclpy.ok():
            try:
                self.ros_executor.spin_once(timeout_sec=0.1)
            except Exception:
                break

    def _odom_callback(self, msg: Any) -> None:
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if stamp <= 0:
            stamp = time.time()

        values: Dict[str, float] = {}
        for key, extractor in self.ros_extractors.items():
            try:
                values[key] = float(extractor(msg))
            except Exception:
                values[key] = np.nan

        with self.ros_lock:
            self.ros_time_buffer.append(stamp)
            for key, val in values.items():
                self.ros_signal_buffers[key].append(val)

            if self.ros_time_buffer:
                newest_t = self.ros_time_buffer[-1]
                min_t = newest_t - self.ros_window_sec
                while self.ros_time_buffer and self.ros_time_buffer[0] < min_t:
                    self.ros_time_buffer.popleft()
                    for key in self.ros_signal_buffers:
                        if self.ros_signal_buffers[key]:
                            self.ros_signal_buffers[key].popleft()

    def _schedule_periodic_refresh(self) -> None:
        self.root.after(300, self._periodic_refresh)

    def _periodic_refresh(self) -> None:
        if self.source_var.get().strip() == "ros" and self.ros_running:
            self._update_plot()
        self._schedule_periodic_refresh()

    def on_close(self) -> None:
        self._stop_ros_subscription()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    app = OdomFftApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    app._write_info("请先点击“加载 CSV”导入里程计数据。")
    root.mainloop()


if __name__ == "__main__":
    main()
