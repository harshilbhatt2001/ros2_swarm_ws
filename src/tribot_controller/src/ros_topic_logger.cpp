#include <memory>
#include <utility>
#include "tribot_controller/ros_topic_logger.hpp"
#include "tf2_ros/buffer_interface.h"

namespace tribot_controller
{

RosTopicLogger::RosTopicLogger(
  const rclcpp::Node::SharedPtr & ros_node, const BT::Tree & tree)
: StatusChangeLogger(tree.rootNode()), ros_node_(ros_node)
{
  log_pub_ = ros_node_->create_publisher<nav2_msgs::msg::BehaviorTreeLog>(
    "behavior_tree_log",
    rclcpp::QoS(10));
}

void RosTopicLogger::callback(
  BT::Duration timestamp,
  const BT::TreeNode & node,
  BT::NodeStatus prev_status,
  BT::NodeStatus status)
{
  nav2_msgs::msg::BehaviorTreeStatusChange event;

  // BT timestamps are a duration since the epoch. Need to convert to a time_point
  // before converting to a msg.
#ifndef _WIN32
  event.timestamp =
    tf2_ros::toMsg(std::chrono::time_point<std::chrono::high_resolution_clock>(timestamp));
#else
  event.timestamp = tf2_ros::toMsg(timestamp);
#endif
  event.node_name = node.name();
  event.previous_status = toStr(prev_status, false);
  event.current_status = toStr(status, false);
  event_log_.push_back(std::move(event));

  RCLCPP_DEBUG(
    ros_node_->get_logger(), "[%.3f]: %25s %s -> %s",
    std::chrono::duration<double>(timestamp).count(),
    node.name().c_str(),
    toStr(prev_status, true).c_str(),
    toStr(status, true).c_str() );
}

void RosTopicLogger::flush()
{
  if (event_log_.size() > 0) {
    auto log_msg = std::make_unique<nav2_msgs::msg::BehaviorTreeLog>();
    log_msg->timestamp = ros_node_->now();
    log_msg->event_log = event_log_;
    log_pub_->publish(std::move(log_msg));
    event_log_.clear();
  }
}

}   // namespace tribot_controller
