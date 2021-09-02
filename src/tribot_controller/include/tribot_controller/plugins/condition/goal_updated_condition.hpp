#ifndef TRIBOT_CONTROLLER__PLUGINS__CONDITION__IS_GOAL_UPDATED_CONDITION_HPP_
#define TRIBOT_CONTROLLER__PLUGINS__CONDITION__IS_GOAL_UPDATED_CONDITION_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{
class GoalUpdatedCondition : public BT::ConditionNode
{
public:
    GoalUpdatedCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf);
    
    GoalUpdatedCondition() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {};
    }

private:
    geometry_msgs::msg::PoseStamped goal_;
    std::vector<geometry_msgs::msg::PoseStamped> goals_;

};
} // namespace nav2_behavior_tree

#endif // TRIBOT_CONTROLLER__PLUGINS__CONDITION__IS_GOAL_UPDATED_CONDITION_HPP_