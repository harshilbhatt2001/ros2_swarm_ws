#ifndef TRIBOT_CONTROLLER__PLUGINS__ACTION__FOLLOW_PATH_ACTION_HPP_
#define TRIBOT_CONTROLLER__PLUGINS__ACTION__FOLLOW_PATH_ACTION_HPP_

#include <string>

#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace nav2_behavior_tree
{
class FollowPathAction : public BtActionNode<nav2_msgs::action::FollowPath>
{
public:
  FollowPathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  void on_wait_for_result() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<nav_msgs::msg::Path>("path", "path to follow"),
        BT::InputPort<std::string>("controller_id", ""),
        BT::InputPort<std::string>("goal_checker_id", "")
      }
    );
  }
};

}  // namespace nav2_behavior_tree

#endif  // TRIBOT_CONTROLLER__PLUGINS__ACTION__FOLLOW_PATH_ACTION_HPP_
