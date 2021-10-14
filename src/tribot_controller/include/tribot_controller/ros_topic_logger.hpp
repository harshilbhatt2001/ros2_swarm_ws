#ifndef TRIBOT_CONTROLLER__ROS_TOPIC_LOGGER_HPP_
#define TRIBOT_CONTROLLER__ROS_TOPIC_LOGGER_HPP_

#include <vector>
#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#include "nav2_msgs/msg/behavior_tree_status_change.h"

namespace tribot_controller
{

class RosTopicLogger : public BT::StatusChangeLogger
{
public:
  RosTopicLogger(const rclcpp::Node::SharedPtr & ros_node, const BT::Tree & tree);

  void callback(
    BT::Duration timestamp,
    const BT::TreeNode & node,
    BT::NodeStatus prev_status,
    BT::NodeStatus status) override;

  void flush() override;

protected:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr log_pub_;
  std::vector<nav2_msgs::msg::BehaviorTreeStatusChange> event_log_;
};

}   // namespace tribot_controller

#endif  // TRIBOT_CONTROLLER__ROS_TOPIC_LOGGER_HPP_
