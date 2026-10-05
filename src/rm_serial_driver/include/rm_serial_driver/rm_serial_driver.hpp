#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mutex>
#include <pb_rm_interfaces/msg/event_data.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <pb_rm_interfaces/msg/game_status.hpp>
#include <pb_rm_interfaces/msg/robot_status.hpp>
#include <pb_rm_interfaces/msg/rfid_status.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);

  ~RMSerialDriver() override;

private:
  void getParams();

  double quaternionToRadians(const geometry_msgs::msg::Quaternion &quaternion);

  void sendTwistData(const geometry_msgs::msg::Twist::SharedPtr msg);

  void sendStatusData(const std_msgs::msg::Int32::SharedPtr msg);

  void sendCombinedData();

  void sendTestData();

  void reopenPort();

  void receiveData();

  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  bool dry_run_ {false};
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  //pub_
  rclcpp::Publisher<pb_rm_interfaces::msg::EventData>::SharedPtr referee_event_data_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::GameStatus>::SharedPtr referee_game_status_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::RobotStatus>::SharedPtr referee_robot_status_pub_;
  rclcpp::Publisher<pb_rm_interfaces::msg::RfidStatus>::SharedPtr referee_rfid_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr received_mode_pub_;
  // sub_
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr status_sub_;

  std::mutex send_mutex_;
  float latest_vx_ {0.0F};
  float latest_vy_ {0.0F};
  float latest_vz_ {0.0F};
  uint8_t latest_status_ {0};

  std::thread receive_thread_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
