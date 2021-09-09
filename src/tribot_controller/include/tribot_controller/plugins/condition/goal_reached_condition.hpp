#ifndef TRIBOT_CONTROLLER__PLUGINS__CONDITION__IS_GOAL_REACHED_CONDITION_HPP_
#define TRIBOT_CONTROLLER__PLUGINS__CONDITION__IS_GOAL_REACHED_CONDITION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{
class GoalReachedCondition : public BT::ConditionNode
{
public:
  GoalReachedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  GoalReachedCondition() = delete;

  ~GoalReachedCondition() override;

  BT::NodeStatus tick() override;

  bool isGoalReached();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame")
    };
  }

protected:
  void cleanup()
  {}

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  double goal_reached_tol_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;

};
} // namespace nav2_behavior_tree

#endif // TRIBOT_CONTROLLER__PLUGINS__CONDITION__IS_GOAL_REACHED_CONDITION_HPP_
