#ifndef TRIBOT_CONTROLLER__TRIBOT_CONTROLLER_HPP_
#define TRIBOT_CONTROLLER__TRIBOT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"


namespace tribot_controller
{
class TribotController : public nav2_util::LifecycleNode
{
public:
    explicit TribotController(bool use_bond = true);
    ~TribotController();

protected:
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

    // TODO: Add a custom action for clustering
    using Action = nav2_msgs::action::FollowPath;

    using ActionServer = nav2_util::SimpleActionServer<Action>;

    std::unique_ptr<ActionServer> action_server_;

    /**
     * @brief Action server callbacks
     */
    void followPath();

    void initializeBlackboard(std::shared_ptr<const Action::Goal> goal);
    
    bool loadBehaviorTree(const std::string &bt_id);

    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;
    std::string xml_string_;
    std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_;
    std::vector<std::string> plugin_lib_name_;
    rclcpp::Node::SharedPtr client_node_;

    bool use_bond_;
};
} // namespace tribot_controller


#endif // TRIBOT_CONTROLLER__TRIBOT_CONTROLLER_HPP_