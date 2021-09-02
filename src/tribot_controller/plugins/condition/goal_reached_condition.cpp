#include <string>
#include <memory>

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/node_utils.hpp"

#include "tribot_controller/plugins/condition/goal_reached_condition.hpp"

namespace nav2_behavior_tree
{
GoalReachedCondition::GoalReachedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf),
    global_frame_("map"),
    robot_base_frame_("base_link")
{
    getInput("global_frame", global_frame_);
    getInput("robot_base_frame", robot_base_frame_);

    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    nav2_util::declare_parameter_if_not_declared(
        node_, "goal_reached_tol",
        rclcpp::ParameterValue(0.25));
    node_->get_parameter_or<double>("goal_reached_tol", goal_reached_tol_, 0.25);
    tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

    node_->get_parameter("transform_tolerance", transform_tolerance_);
}

GoalReachedCondition::~GoalReachedCondition()
{
    cleanup();
}

BT::NodeStatus GoalReachedCondition::tick()
{
    if (isGoalReached()) 
    {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

bool GoalReachedCondition::isGoalReached()
{
    geometry_msgs::msg::PoseStamped current_pose;
    if (!nav2_util::getCurrentPose(
        current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
    {
        RCLCPP_DEBUG(node_->get_logger(), "Current robot pose not available!");
        return false;
    }

    geometry_msgs::msg::PoseStamped goal;
    getInput("goal", goal);
    double dx = goal.pose.position.x - current_pose.pose.position.x;
    double dy = goal.pose.position.y - current_pose.pose.position.y;
    return (dx*dx + dy*dy) <= (goal_reached_tol_*goal_reached_tol_);
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::GoalReachedCondition>("GoalReached");
}