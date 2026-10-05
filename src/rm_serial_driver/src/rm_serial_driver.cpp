#include "rm_serial_driver/rm_serial_driver.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <serial_driver/serial_driver.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();

  // 裁判数据发布方
  referee_event_data_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::EventData>("referee/event_data", 10);
  referee_game_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::GameStatus>("referee/game_status", 10);
  referee_robot_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RobotStatus>("referee/robot_status", 10);
  referee_rfid_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RfidStatus>("referee/rfid_status", 10);
  received_mode_pub_ = this->create_publisher<std_msgs::msg::Int32>("/rm_serial_driver/received_robot_mode", 10);

  if (dry_run_) {
    RCLCPP_WARN(get_logger(), "RMSerialDriver is running in dry_run mode. Serial port is disabled.");
  } else {
    while (rclcpp::ok()) {
      try {
        serial_driver_->init_port(device_name_, *device_config_);
        if (!serial_driver_->port()->is_open()) {
          serial_driver_->port()->open();
        }
        receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
        RCLCPP_INFO(get_logger(), "serial open OK!");
        sendTestData();
        break;
      } catch (const std::exception & ex) {
        RCLCPP_WARN(
          get_logger(), "Serial port not available (%s): %s. Retrying...",
          device_name_.c_str(), ex.what());
        rclcpp::sleep_for(std::chrono::seconds(1));
      }
    }
  }

  //创建订阅方
  //并发送速度消息到下位机
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_base_real_yaw", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendTwistData, this, std::placeholders::_1));
  status_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/sentry_status", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendStatusData, this, std::placeholders::_1));
}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

double RMSerialDriver::quaternionToRadians(const geometry_msgs::msg::Quaternion & quaternion)
{
  // 计算角度的弧度表示
  double roll, pitch, yaw;
  tf2::Quaternion tf_quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
  tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

  return yaw;  // 这里返回yaw角度，你可以根据需要选择其他角度
}

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);  //unsigned int (16)
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket));

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(header);

      if (header[0] == 0x5A) {
        data.resize(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(data);

        data.insert(data.begin(), header[0]);
        ReceivePacket packet = fromVector(data);
        
      // std::ostringstream oss;
      // oss << "Raw Data: ";
      // for (const auto &byte : data) {
      //   oss << std::hex << std::uppercase << static_cast<int>(byte) << " ";
      // }
      // RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());


      bool crc_ok =
        crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
      if (crc_ok) {
          pb_rm_interfaces::msg::GameStatus referee_game_status_msg;
          referee_game_status_msg.game_progress = packet.game_process;
          referee_game_status_msg.stage_remain_time = packet.remain_time;
          referee_game_status_pub_->publish(referee_game_status_msg);

          pb_rm_interfaces::msg::RobotStatus referee_robot_status_msg;

          referee_robot_status_msg.current_hp = packet.current_hp;
          referee_robot_status_msg.projectile_allowance_17mm = packet.projectile_allowance;
          referee_robot_status_msg.robot_id = packet.robot_id;
          //referee_robot_status_msg.shooter_17mm_1_barrel_heat = packet.shooter_heat;


          referee_robot_status_pub_->publish(referee_robot_status_msg);

          pb_rm_interfaces::msg::RfidStatus referee_rfid_status_msg;
          referee_rfid_status_msg.friendly_supply_zone_non_exchange =
            static_cast<bool>(packet.friendly_supply_zone_non_exchange);
          referee_rfid_status_msg.friendly_supply_zone_exchange =
            static_cast<bool>(packet.friendly_supply_zone_exchange);
          referee_rfid_status_msg.friendly_fortress_gain_point =
            static_cast<bool>(packet.friendly_fortress_gain_point);
          // referee_rfid_status_msg.center_gain_point =
          //   static_cast<bool>(packet.center_gain_point);
          referee_rfid_status_pub_->publish(referee_rfid_status_msg);

          pb_rm_interfaces::msg::EventData event_data_msg;
          event_data_msg.base_hp = packet.base_hp;
          event_data_msg.fortress_gain_zone = packet.fortress_status;

          referee_event_data_pub_->publish(event_data_msg);
          // RCLCPP_INFO(get_logger(),
          //   "Robot ID: %d, Current HP: %d, Remaining Time: %d, Game Process: %d, Projectile Allowance: %d",
          //   packet.robot_id, packet.current_hp, packet.remain_time,
          //   packet.game_process, packet.projectile_allowance);
        } else {
          RCLCPP_ERROR(get_logger(), "CRC error!");
        }
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void RMSerialDriver::sendTwistData(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  try {
    {
      std::lock_guard<std::mutex> lock(send_mutex_);
      latest_vx_ = static_cast<float>(msg->linear.x);
      latest_vy_ = static_cast<float>(msg->linear.y);
      latest_vz_ = static_cast<float>(msg->angular.z);
    }

    sendCombinedData();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending twist data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::sendStatusData(const std_msgs::msg::Int32::SharedPtr status_msg)
{
  try {
    const int clamped_status = std::clamp(status_msg->data, 0, 255);

    if (status_msg->data != clamped_status) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Received /sentry_status=%d out of [0,255], clamped to %d",
        status_msg->data, clamped_status);
    }

    {
      std::lock_guard<std::mutex> lock(send_mutex_);
      latest_status_ = static_cast<uint8_t>(clamped_status);
    }

    std_msgs::msg::Int32 received_mode_msg;
    received_mode_msg.data = clamped_status;
    received_mode_pub_->publish(received_mode_msg);

    sendCombinedData();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending status data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::sendCombinedData()
{
  SendPacket packet;
  {
    std::lock_guard<std::mutex> lock(send_mutex_);
    packet.vx = latest_vx_;
    packet.vy = latest_vy_;
    packet.vz = latest_vz_;
    packet.status = latest_status_;
  }

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "Send combined packet: vx=%.3f vy=%.3f vz=%.3f status=%u",
    packet.vx, packet.vy, packet.vz, packet.status);

  if (dry_run_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "dry_run enabled, skip sending combined control data to serial.");
    return;
  }

  if (!serial_driver_->port()->is_open()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Serial port not open, skip sending control data.");
    return;
  }

  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
  std::vector<uint8_t> data = toVector(packet);
  serial_driver_->port()->send(data);
}

void RMSerialDriver::sendTestData()
{
  try {
    SendPacket packet;
    packet.vx = 0.0F;
    packet.vy = 0.0F;
    packet.vz = 0.0F;
    packet.status = 3;

    RCLCPP_INFO(
      get_logger(), "Send test packet: vx=%.2f vy=%.2f vz=%.2f status=%u",
      packet.vx, packet.vy, packet.vz, packet.status);

    if (dry_run_) {
      RCLCPP_INFO(get_logger(), "dry_run enabled, skip sending test data to serial.");
      return;
    }

    if (!serial_driver_->port()->is_open()) {
      RCLCPP_WARN(get_logger(), "Serial port not open, skip sending test data.");
      return;
    }

    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    std::vector<uint8_t> data = toVector(packet);
    serial_driver_->port()->send(data);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending test data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    dry_run_ = declare_parameter<bool>("dry_run", false);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The dry_run provided was invalid");
    throw ex;
  }

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}
}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
