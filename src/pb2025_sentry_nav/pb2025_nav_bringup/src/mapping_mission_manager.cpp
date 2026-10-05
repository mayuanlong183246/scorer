#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "fast_lio/srv/save_pcd_map.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr const char * kLightGreen = "\033[1;92m";
constexpr const char * kColorReset = "\033[0m";

std::string greenLog(const std::string & message)
{
  return std::string(kLightGreen) + message + kColorReset;
}

geometry_msgs::msg::PoseStamped makeGoal(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = 0.0;
  goal.pose.orientation.z = std::sin(yaw * 0.5);
  goal.pose.orientation.w = std::cos(yaw * 0.5);
  return goal;
}
}  // namespace

class MappingMissionManager : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  explicit MappingMissionManager(const rclcpp::NodeOptions & options)
  : Node("mapping_mission_manager", options)
  {
    declare_parameter("enabled", true);
    declare_parameter("game_start_progress", 4);
    declare_parameter("bootstrap_timeout_sec", 20.0);
    declare_parameter("pcd_save_path", std::string());
    declare_parameter("grid_map_save_path", std::string());
    declare_parameter("goal1", std::vector<double>({2.0, 0.0, 0.0}));
    declare_parameter("goal2", std::vector<double>({8.65, -3.5, 0.0}));
    declare_parameter("goal3", std::vector<double>({4.0, 3.0, 0.0}));

    enabled_ = get_parameter("enabled").as_bool();
    game_start_progress_ = get_parameter("game_start_progress").as_int();
    bootstrap_timeout_sec_ = get_parameter("bootstrap_timeout_sec").as_double();
    pcd_save_path_ = get_parameter("pcd_save_path").as_string();
    grid_map_save_path_ = get_parameter("grid_map_save_path").as_string();

    loadGoal("goal1");
    loadGoal("goal2");
    loadGoal("goal3");

    RCLCPP_INFO(
      get_logger(),
      "%s",
      greenLog(
        std::string("建图引导配置: 启用=") + (enabled_ ? "true" : "false") +
        " 超时=" + std::to_string(bootstrap_timeout_sec_) + "秒 点云保存=" + pcd_save_path_ +
        " 栅格保存=" + grid_map_save_path_).c_str());

    bootstrap_finished_pub_ =
      create_publisher<std_msgs::msg::Bool>("/mapping_bootstrap_finished", rclcpp::QoS(1).transient_local());
    publishBootstrapFinished(false);

    if (!enabled_) {
      RCLCPP_INFO(
        get_logger(),
        "%s", greenLog("未启用建图引导，直接放行正常行为树").c_str());
      publishBootstrapFinished(true);
      return;
    }

    game_status_sub_ = create_subscription<pb_rm_interfaces::msg::GameStatus>(
      "referee/game_status", 10,
      std::bind(&MappingMissionManager::onGameStatus, this, std::placeholders::_1));

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    pcd_save_client_ = create_client<fast_lio::srv::SavePcdMap>("/map_save");
    grid_map_save_client_ = create_client<nav2_msgs::srv::SaveMap>("/save_map");
  }

private:
  void loadGoal(const std::string & parameter_name)
  {
    const auto values = get_parameter(parameter_name).as_double_array();
    if (values.size() != 3) {
      throw std::runtime_error("建图引导导航点必须恰好包含 3 个数值");
    }
    goals_.push_back(makeGoal(values[0], values[1], values[2]));
  }

  void onGameStatus(const pb_rm_interfaces::msg::GameStatus::SharedPtr msg)
  {
    if (bootstrap_started_) {
      return;
    }

    if (msg->game_progress != game_start_progress_) {
      return;
    }

    bootstrap_started_ = true;
    RCLCPP_INFO(
      get_logger(), "%s", greenLog("比赛开始，启动建图引导任务，并开始 20 秒建图计时").c_str());

    bootstrap_deadline_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(bootstrap_timeout_sec_)),
      std::bind(&MappingMissionManager::onBootstrapTimeout, this));

    sendNextGoal();
  }

  void sendNextGoal()
  {
    if (!nav_client_->wait_for_action_server(2s)) {
      RCLCPP_WARN(
        get_logger(), "%s",
        greenLog("导航动作服务器尚未就绪，1 秒后重试发送建图引导导航点").c_str());
      retry_goal_timer_ = create_wall_timer(1s, std::bind(&MappingMissionManager::retrySendGoal, this));
      return;
    }

    if (goal_index_ >= goals_.size()) {
      RCLCPP_INFO(
        get_logger(), "%s",
        greenLog("建图引导导航点已全部下发，等待 20 秒到时后保存地图").c_str());
      return;
    }

    goals_[goal_index_].header.stamp = now();
    NavigateToPose::Goal goal_msg;
    goal_msg.pose = goals_[goal_index_];

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.result_callback =
      std::bind(&MappingMissionManager::onNavResult, this, std::placeholders::_1);

    RCLCPP_INFO(
      get_logger(), "%s",
      greenLog(
        "发送建图引导导航点 " + std::to_string(goal_index_ + 1) + "/3: x=" +
        std::to_string(goal_msg.pose.pose.position.x) + " y=" +
        std::to_string(goal_msg.pose.pose.position.y)).c_str());
    nav_client_->async_send_goal(goal_msg, options);
  }

  void retrySendGoal()
  {
    retry_goal_timer_.reset();
    sendNextGoal();
  }

  void onNavResult(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result)
  {
    if (bootstrap_finished_) {
      return;
    }

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      ++goal_index_;
      sendNextGoal();
      return;
    }

    RCLCPP_WARN(
      get_logger(), "建图引导导航点 %zu 返回结果码 %d，继续下一个导航点",
      goal_index_ + 1, static_cast<int>(result.code));
    ++goal_index_;
    sendNextGoal();
  }

  void onBootstrapTimeout()
  {
    if (bootstrap_finished_) {
      return;
    }

    bootstrap_deadline_timer_.reset();
    RCLCPP_INFO(
      get_logger(), "%s",
      greenLog("建图 20 秒已到，取消引导导航并开始保存点云地图和栅格地图").c_str());
    nav_client_->async_cancel_all_goals();
    saveMaps();
  }

  void saveMaps()
  {
    if (!pcd_save_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(get_logger(), "FAST_LIO 点云地图保存服务不可用");
      return;
    }
    if (!grid_map_save_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(get_logger(), "Nav2 栅格地图保存服务不可用");
      return;
    }

    auto pcd_req = std::make_shared<fast_lio::srv::SavePcdMap::Request>();
    pcd_req->file_path = pcd_save_path_;

    RCLCPP_INFO(
      get_logger(), "%s",
      greenLog(std::string("开始保存建图点云地图到: ") + pcd_save_path_).c_str());

    pcd_save_client_->async_send_request(
      pcd_req,
      std::bind(&MappingMissionManager::onPcdSaved, this, std::placeholders::_1));
  }

  void onPcdSaved(rclcpp::Client<fast_lio::srv::SavePcdMap>::SharedFuture future)
  {
    const auto response = future.get();
    if (!response->success) {
      RCLCPP_ERROR(get_logger(), "保存点云地图失败: %s", response->message.c_str());
      return;
    }

    RCLCPP_INFO(
      get_logger(), "%s",
      greenLog(std::string("点云地图保存成功: ") + response->message).c_str());

    auto map_req = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
    map_req->map_topic = "map";
    map_req->map_url = grid_map_save_path_;
    map_req->image_format = "pgm";
    map_req->map_mode = "trinary";
    map_req->free_thresh = 0.25f;
    map_req->occupied_thresh = 0.65f;

    RCLCPP_INFO(
      get_logger(), "%s",
      greenLog(std::string("开始保存栅格地图到: ") + grid_map_save_path_).c_str());

    grid_map_save_client_->async_send_request(
      map_req,
      std::bind(&MappingMissionManager::onGridMapSaved, this, std::placeholders::_1));
  }

  void onGridMapSaved(rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedFuture future)
  {
    const auto response = future.get();
    if (!response->result) {
      RCLCPP_ERROR(get_logger(), "保存栅格地图失败");
      return;
    }

    RCLCPP_INFO(
      get_logger(), "%s", greenLog("栅格地图保存成功，建图引导阶段完成").c_str());

    bootstrap_finished_ = true;
    publishBootstrapFinished(true);
    RCLCPP_INFO(
      get_logger(), "%s", greenLog("已放行正常行为树，后续进入常规决策流程").c_str());
  }

  void publishBootstrapFinished(bool value)
  {
    std_msgs::msg::Bool msg;
    msg.data = value;
    bootstrap_finished_pub_->publish(msg);
  }

  bool enabled_{true};
  bool bootstrap_started_{false};
  bool bootstrap_finished_{false};
  int game_start_progress_{4};
  double bootstrap_timeout_sec_{20.0};
  std::string pcd_save_path_;
  std::string grid_map_save_path_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
  size_t goal_index_{0};

  rclcpp::Subscription<pb_rm_interfaces::msg::GameStatus>::SharedPtr game_status_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bootstrap_finished_pub_;
  rclcpp::TimerBase::SharedPtr bootstrap_deadline_timer_;
  rclcpp::TimerBase::SharedPtr retry_goal_timer_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Client<fast_lio::srv::SavePcdMap>::SharedPtr pcd_save_client_;
  rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr grid_map_save_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MappingMissionManager>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
