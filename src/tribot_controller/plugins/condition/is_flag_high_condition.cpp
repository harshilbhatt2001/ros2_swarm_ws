#include <string>

#include "tribot_controller/plugins/condition/is_flag_high_condition.hpp"

namespace nav2_behavior_tree
{
IsFlagHighCondition::IsFlagHighCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  rearrange_flag_topic_("rearrange_flag"),
  is_flag_high_(false)
{
  getInput("rearrange_flag", rearrange_flag_topic_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_executor_.add_node(node_);

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  flag_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    rearrange_flag_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsFlagHighCondition::flagCallback, this, std::placeholders::_1),
    sub_option);
}

BT::NodeStatus IsFlagHighCondition::tick()
{
  callback_group_executor_.spin_some();
  if (is_flag_high_) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

void IsFlagHighCondition::flagCallback(std_msgs::msg::Bool::SharedPtr msg)
{
  is_flag_high_ = msg->data;
}


} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsFlagHighCondition>("IsFlagHigh");
}
