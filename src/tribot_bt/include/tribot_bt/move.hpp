#ifndef TRIBOT_BT__MOVE_HPP_
#define TRIBOT_BT__MOVE_HPP_

#include <string>
#include <map>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"


namespace nav2_behavior_tree
{

class Move : public BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  Move(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  // BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("goal", "Pallet ID"),

        // BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
      });
  }

private:
  int goal_reached_;
  std::map<std::string, geometry_msgs::msg::Pose> waypoints_;
};

}  // namespace nav2_behavior_tree

#endif  // TRIBOT_BT__MOVE_HPP_
