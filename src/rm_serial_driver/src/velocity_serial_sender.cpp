// Copyright 2026 RMUC contributors
// Licensed under the Apache License, Version 2.0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <stdexcept>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/int32.hpp>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"

namespace rm_serial_driver
{

class VelocitySerialSender : public rclcpp::Node
{
public:
  explicit VelocitySerialSender(const rclcpp::NodeOptions & options)
  : Node("velocity_serial_sender", options),
    io_context_(new IoContext(2)),
    serial_driver_(new drivers::serial_driver::SerialDriver(*io_context_))
  {
    device_name_ = declare_parameter<std::string>("device_name", "/dev/ttyACM0");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);
    flow_control_ = declare_parameter<std::string>("flow_control", "none");
    parity_ = declare_parameter<std::string>("parity", "none");
    stop_bits_ = declare_parameter<std::string>("stop_bits", "1");
    dry_run_ = declare_parameter<bool>("dry_run", false);
    input_topic_ = declare_parameter<std::string>("input_topic", "/cmd_vel_base_real_yaw");
    status_topic_ = declare_parameter<std::string>("status_topic", "/sentry_status");

    if (baud_rate_ <= 0) {
      throw std::invalid_argument("baud_rate must be positive");
    }
    device_config_ = makePortConfig();

    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VelocitySerialSender::onTwist, this, std::placeholders::_1));
    status_sub_ = create_subscription<std_msgs::msg::Int32>(
      status_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VelocitySerialSender::onStatus, this, std::placeholders::_1));
    reconnect_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&VelocitySerialSender::tryOpen, this));

    RCLCPP_INFO(
      get_logger(),
      "Velocity serial sender: input=%s device=%s baud=%d dry_run=%s",
      input_topic_.c_str(), device_name_.c_str(), baud_rate_, dry_run_ ? "true" : "false");
    warnIfLegacyDriverRunning();
    if (!dry_run_) {
      tryOpen();
    } else {
      RCLCPP_WARN(get_logger(), "dry_run enabled: no serial port will be opened");
    }
  }

  ~VelocitySerialSender() override
  {
    try {
      sendPacket(0.0F, 0.0F, 0.0F, status_);
    } catch (const std::exception & ex) {
      RCLCPP_WARN(get_logger(), "Unable to send shutdown zero velocity: %s", ex.what());
    }
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    if (io_context_) {
      io_context_->waitForExit();
    }
  }

private:
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  std::unique_ptr<drivers::serial_driver::SerialPortConfig> makePortConfig()
  {
    FlowControl flow;
    if (flow_control_ == "none") flow = FlowControl::NONE;
    else if (flow_control_ == "hardware") flow = FlowControl::HARDWARE;
    else if (flow_control_ == "software") flow = FlowControl::SOFTWARE;
    else throw std::invalid_argument("flow_control must be none, hardware, or software");

    Parity parity;
    if (parity_ == "none") parity = Parity::NONE;
    else if (parity_ == "odd") parity = Parity::ODD;
    else if (parity_ == "even") parity = Parity::EVEN;
    else throw std::invalid_argument("parity must be none, odd, or even");

    StopBits stop_bits;
    if (stop_bits_ == "1" || stop_bits_ == "1.0") stop_bits = StopBits::ONE;
    else if (stop_bits_ == "1.5") stop_bits = StopBits::ONE_POINT_FIVE;
    else if (stop_bits_ == "2" || stop_bits_ == "2.0") stop_bits = StopBits::TWO;
    else throw std::invalid_argument("stop_bits must be 1, 1.5, or 2");

    return std::make_unique<drivers::serial_driver::SerialPortConfig>(
      static_cast<uint32_t>(baud_rate_), flow, parity, stop_bits);
  }

  void warnIfLegacyDriverRunning()
  {
    for (const auto & name : get_node_names()) {
      if (name == "rm_serial_driver" || name == "/rm_serial_driver") {
        RCLCPP_ERROR(
          get_logger(),
          "rm_serial_driver is also running; stop one sender to avoid duplicate serial writes");
        return;
      }
    }
  }

  void tryOpen()
  {
    if (dry_run_ || serial_driver_->port()->is_open()) return;
    try {
      serial_driver_->init_port(device_name_, *device_config_);
      if (!serial_driver_->port()->is_open()) serial_driver_->port()->open();
      RCLCPP_INFO(get_logger(), "Serial opened: %s", device_name_.c_str());
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Serial unavailable (%s): %s", device_name_.c_str(), ex.what());
    }
  }

  void onStatus(const std_msgs::msg::Int32::SharedPtr msg)
  {
    status_ = static_cast<uint8_t>(std::clamp(msg->data, 0, 255));
  }

  void onTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
      !std::isfinite(msg->angular.z)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring non-finite velocity");
      return;
    }
    sendPacket(
      static_cast<float>(msg->linear.x), static_cast<float>(msg->linear.y),
      static_cast<float>(msg->angular.z), status_);
  }

  void sendPacket(float vx, float vy, float vz, uint8_t status)
  {
    SendPacket packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.status = status;
    packet.checksum = 0;
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    const auto data = toVector(packet);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Send velocity: vx=%.3f vy=%.3f vz=%.3f status=%u%s",
      vx, vy, vz, status, dry_run_ ? " (dry_run)" : "");

    if (dry_run_) return;
    std::lock_guard<std::mutex> lock(send_mutex_);
    if (!serial_driver_->port()->is_open()) {
      tryOpen();
      if (!serial_driver_->port()->is_open()) return;
    }
    try {
      serial_driver_->port()->send(data);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Serial send failed: %s", ex.what());
      if (serial_driver_->port()->is_open()) serial_driver_->port()->close();
    }
  }

  std::string device_name_, flow_control_, parity_, stop_bits_, input_topic_, status_topic_;
  int baud_rate_{115200};
  bool dry_run_{false};
  uint8_t status_{0};
  std::unique_ptr<IoContext> io_context_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::mutex send_mutex_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
};

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::VelocitySerialSender)
