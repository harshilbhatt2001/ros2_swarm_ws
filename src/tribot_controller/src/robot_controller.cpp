#include "tribot_controller/robot_controller.hpp"

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>
#include <set>

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"

namespace tribot_controller
{
TribotController::TribotController(bool use_bond)
: nav2_util::LifecycleNode("tribot_controller", "", false),
  use_bond_(use_bond)
{
  RCLCPP_INFO(get_logger(), "Creating");
  const std::vector<std::string> plugin_libs = {
    "nav2_compute_path_to_pose_action_bt_node",
    "nav2_follow_path_action_bt_node",
    "nav2_back_up_action_bt_node",
    "nav2_spin_action_bt_node",
    "nav2_wait_action_bt_node",
    "nav2_clear_costmap_service_bt_node",
    "nav2_is_stuck_condition_bt_node",
    "nav2_goal_reached_condition_bt_node",
    "nav2_initial_pose_received_condition_bt_node",
    "nav2_goal_updated_condition_bt_node",
    "nav2_reinitialize_global_localization_service_bt_node",
    "nav2_rate_controller_bt_node",
    "nav2_distance_controller_bt_node",
    "nav2_speed_controller_bt_node",
    "nav2_truncate_path_action_bt_node",
    "nav2_change_goal_node_bt_node",
    "nav2_recovery_node_bt_node",
    "nav2_pipeline_sequence_bt_node",
    "nav2_round_robin_node_bt_node",
    "nav2_transform_available_condition_bt_node",
    "nav2_time_expired_condition_bt_node",
    "nav2_distance_traveled_condition_bt_node"
  };

  declare_parameter("controller_bt_xml_filename");
  declare_parameter("plugin_lib_names", plugin_libs);
  declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter("global_frame", std::string("map"));
  declare_parameter("robot_base_frame", std::string("base_link"));
  declare_parameter("odom_topic", std::string("odom"));
}

TribotController::~TribotController()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn TribotController::on_configure(const rclcpp_lifecycle::State &)
{
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args",
      "-r", std::string("__node:=") + get_name() + "_rclcpp_node",
      "--"});
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  action_server_ = std::make_unique<ActionServer>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "follow_path", std::bind(&TribotController::followPath, this));

  plugin_lib_name_ = get_parameter("plugin_lib_names").as_string_array();

  // Create class that registers custom nodes and executed bt
  bt_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_lib_name_);

  blackboard_ = BT::Blackboard::create();

  // TODO(harshil): Put items on the balckboard
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);

  std::string controller_bt_xml_filename;
  get_parameter("controller_bt_xml_filename", controller_bt_xml_filename);

  if (!loadBehaviorTree(controller_bt_xml_filename)) {
    return nav2_util::CallbackReturn::FAILURE;
  }
  return nav2_util::CallbackReturn::SUCCESS;
}

bool TribotController::loadBehaviorTree(const std::string & controller_bt_xml_filename)
{
  std::ifstream xml_file(controller_bt_xml_filename);

  if (!xml_file.good()) {
    return false;
  }

  auto xml_string_ = std::string(
    std::istreambuf_iterator<char>(xml_file),
    std::istreambuf_iterator<char>());

  tree_ = bt_->createTreeFromText(xml_string_, blackboard_);
  return true;
}

nav2_util::CallbackReturn TribotController::on_activate(const rclcpp_lifecycle::State &)
{
  action_server_->activate();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TribotController::on_deactivate(const rclcpp_lifecycle::State &)
{
  action_server_->deactivate();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TribotController::on_cleanup(const rclcpp_lifecycle::State &)
{
  client_node_.reset();
  action_server_.reset();
  plugin_lib_name_.clear();
  xml_string_.clear();
  blackboard_.reset();
  bt_->haltAllActions(tree_.rootNode());
  bt_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TribotController::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void TribotController::followPath()
{
}

void TribotController::initializeBlackboard(std::shared_ptr<const Action::Goal> goal)
{
  // TODO(harshil): Update goals on the blackboard
  (void)goal;
}

}  // namespace tribot_controller
