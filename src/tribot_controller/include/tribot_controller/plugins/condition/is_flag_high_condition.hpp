#ifndef TRIBOT_CONTROLLER__PLUGINS__CONDITION__IS_FLAG_HIGH_CONDITION_HPP_
#define TRIBOT_CONTROLLER__PLUGINS__CONDITION__IS_FLAG_HIGH_CONDITION_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{
class IsFlagHighCondition : public BT::ConditionNode
{
public:
  IsFlagHighCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsFlagHighCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("rearrange_flag", false, "True if bots need to rearrange after charging"),
      BT::InputPort<std::string>("/rearrange_flag", "Rearrange Flag Topic")
    };
  }

private:
  void flagCallback(std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::string rearrange_flag_topic_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_sub_;
  bool is_flag_high_;
};
} // namespace nav2_behavior_tree

#endif // TRIBOT_CONTROLLER__PLUGINS__CONDITION__IS_FLAG_HIGH_CONDITION_HPP_
